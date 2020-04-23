/*
 * BrightSign Tetsuo FPGA driver
 *
 * "Tetsuo" is the FPGA that is used for six-channel audio on
 * BrightSign Pantera. Its initial bitstream is provided over SPI and
 * thereafter it communicates via SPI and RMX.
 *
 * By default Pantera boots expecting audio to be on I2S so we must
 * apply some pinmux changes from the device tree to switch the pins
 * to SPI mode.
 *
 * Additionally, loading the bitstream over SPI requires a complex
 * dance with the chip select so we have to temporarily reconfigure it
 * to be a GPIO using information from the device tree.
 */
 
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/serial_core.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>

#include <sound/core.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/pcm-indirect.h>
#include <sound/asoundef.h>
#include <sound/initval.h>

static int index = -1;
static char *id = NULL;
static bool enable = 1;

#define CARD_NAME "Tetsuo"
module_param(index, int, 0444);
MODULE_PARM_DESC(index, "Index value for " CARD_NAME " soundcard.");
module_param(id, charp, 0444);
MODULE_PARM_DESC(id, "ID string for " CARD_NAME " soundcard.");
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Enable " CARD_NAME " soundcard.");

typedef int AudioSample;

#define CHANNEL_COUNT	6
#define VOLUME_RANGE	(16*256)		// This must be a power of 2 so avoid having to do (long long) divides.
										// When de-zippering the volume is changed by one unit per sample, so to get from mute
										// to full volume takes VOLUME_RANGE samples.

#define FPGA_FIFO_SIZE		256												// Audio samples buffered in FPGA
#define RMX_PACKET_SIZE		(CHANNEL_COUNT * sizeof(AudioSample) * 8)		// Whole number of samples
#define TETSUO_PERIOD		RMX_PACKET_SIZE									// Number of audio samples to write out on each interrupt

// Input data from ALSA is 32 bits, as is the output data over the RMX bus, so both have the same period size in bytes
#define TETSUO_PERIOD_IN_BYTES		 	(CHANNEL_COUNT * sizeof(AudioSample) * TETSUO_PERIOD)

/* Playback buffer size - must be bigger than the trigger level used by the application. 
The application code currently allows 500ms + 125ms of delay, which is 30000 samples rounded up to the nearest period size*/
#define TETSUO_BUFFER_SIZE (TETSUO_PERIOD_IN_BYTES * ((30000 + TETSUO_PERIOD - 1) / TETSUO_PERIOD))

// SPI control address
#define TETSUO_READ			0x80
#define TETSUO_WRITE		0x00

#define TETSUO_IRQ			0x04
#define TETSUO_CONTROL		0x03
#define TETSUO_FILL_LOW		0x02
#define TETSUO_ID			TETSUO_FILL_LOW
#define TETSUO_IN_PTR		0x01
#define TETSUO_OUT_PTR		0x00

#define TETSUO_TEST_IRQ		0X04
#define TETSUO_INT_ENABLE	0x02
#define TETSUO_INT_RESET	0x01

// Our current approach is that we get an low water mark interrupt at 88 samples at which we write out 168 samples.

static struct snd_pcm_hardware snd_tetsuo_info =
{
	.info =		(SNDRV_PCM_INFO_MMAP | 
				 SNDRV_PCM_INFO_MMAP_VALID |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_NONINTERLEAVED | 
				 SNDRV_PCM_INFO_SYNC_START),
	.formats =	(SNDRV_PCM_FMTBIT_S32_LE),
	.rates =	(SNDRV_PCM_RATE_44100 | 
				 SNDRV_PCM_RATE_48000 | 
				 SNDRV_PCM_RATE_88200 | 
				 SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000),
	.rate_min =	44100,
	.rate_max =	192000,
	.channels_min =	CHANNEL_COUNT,
	.channels_max =	CHANNEL_COUNT,
	.buffer_bytes_max = TETSUO_BUFFER_SIZE,
	.period_bytes_min = TETSUO_PERIOD_IN_BYTES,
	.period_bytes_max = TETSUO_PERIOD_IN_BYTES,
	.periods_min =	TETSUO_BUFFER_SIZE / TETSUO_PERIOD_IN_BYTES,
	.periods_max =	TETSUO_BUFFER_SIZE / TETSUO_PERIOD_IN_BYTES,
};

struct tetsuo_sound {
	spinlock_t lock;

	u8 __iomem *xpt_base;
	u8 __iomem *aud_base;

	struct snd_pcm_substream *playback_substream;

	int running;
	int draining;

	unsigned int pos;
	unsigned int buffer_size;
	unsigned int period_size;

	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_kcontrol *volctl;
	
	int target_pcm_volume_level[CHANNEL_COUNT];
	int current_pcm_volume_level[CHANNEL_COUNT];
	spinlock_t pcm_volume_level_lock;
	
	struct tetsuo_device *device;

	void *base_addr;
	unsigned char *temp_buffer;
	dma_addr_t dma_buffer;

	struct mcpb_dma_desc *descriptor;
	dma_addr_t descriptor_dma_address;
	int active_descriptor;
};

/* When developing it can be useful to see the MD5 hash of the bitstream */
#define DESCRIBE_BITSTREAM 1

#define DEBUG 0

#define tetsuo_err(...) pr_err("tetsuo: " __VA_ARGS__)
#define tetsuo_err_ratelimited(...) pr_err_ratelimited("tetsuo: " __VA_ARGS__)
#if DEBUG
#define tetsuo_dbg(...) pr_err("tetsuo: " __VA_ARGS__)
#define tetsuo_io(...) pr_err("tetsuo: " __VA_ARGS__)
#else // !DEBUG
#define tetsuo_dbg(dev, ...) do { } while(0)
#define tetsuo_io(dev, ...) do { } while(0)
#endif // !DEBUG

struct tetsuo_device {
	struct spi_device *spi;

	int irq;
	int gpio_irq;
	struct tetsuo_sound *ssound;
};

/*

	Tetsuo SPI accesses	

*/
static int tetsuo_read(struct tetsuo_sound *ssound, unsigned char reg)
{
	ssize_t ret;
	ret = spi_w8r8(ssound->device->spi, TETSUO_READ | reg);
	
	if(ret < 0)
	{
		tetsuo_dbg("spi_w8r8 failed: %d\n", ret);
		return 0;
	}
	
	return ret;
}

static int tetsuo_write(struct tetsuo_sound *ssound, unsigned char reg, unsigned char value)
{
	int ret;
	unsigned char buf[2];
	
	buf[0] = TETSUO_WRITE | reg;
	buf[1] = value;

	ret = spi_write(ssound->device->spi, buf, 2);
	if(ret != 0)
		tetsuo_dbg("spi_write failed: %d\n", ret);
	else
		tetsuo_dbg("spi_write wrote: %02x to %02x\n", (int)buf[1], (int)buf[0]);
		
	return ret;
}



/*

	Tetsuo audio DMA accesses	

*/
#include <linux/brcmstb/brcmstb.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/delay.h>

struct mcpb_dma_desc {
	uint32_t buf_hi;
	uint32_t buf_lo;
	uint32_t next_offs;
	uint32_t size;
	uint32_t opts1;
	uint32_t opts2;
	uint32_t pid_channel;
	uint32_t reserved;
};

// All offset from 0x00a00000

#define XPT_PMU_CLK_CTRL				0x00200
#define XPT_PMU_CLK_CTRL_RMX0_DISABLE_MASK		0x00004000
#define XPT_PMU_CLK_CTRL_XPT_IO_DISABLE_MASK		0x00001000

#define XPT_MCPB_CH0_REG_START				0x70c00
#define XPT_MCPB_CH1_REG_START				0x70e00
#define XPT_MCPB_RUN_SET_CLEAR				0x70800
#define XPT_MCPB_WAKE_SET				0x70804

#define XPT_MCPB_CH0_DMA_DESC_CONTROL			0x70c00
#define XPT_MCPB_CH0_DMA_DATA_CONTROL			0x70c04
#define XPT_MCPB_CH0_SP_PKT_LEN				0x70c74
#define XPT_MCPB_CH0_SP_PARSER_CTRL			0x70c78
#define XPT_MCPB_CH0_SP_PARSER_CTRL1			0x70c7c
#define XPT_MCPB_CH0_SP_TS_CONFIG			0x70c80
#define XPT_MCPB_CH0_DMA_BBUFF_CTRL			0x70cc8
#define XPT_MCPB_CH0_TMEU_BLOCKOUT_CTRL			0x70ce0

#define MCPB_DW5_PID_CHANNEL_VALID (1 << 19)

#define XPT_FULL_PID_PARSER_PBP_ACCEPT_ADAPT_00 	0x14008
#define XPT_FULL_PID_PARSER_STATE_CONFIG_0_i_ARRAY_BASE	0x10000

#define XPT_FE_PID_TABLE_i_ARRAY_BASE                   0x21000
#define XPT_FE_SPID_TABLE_i_ARRAY_BASE                  0x23000
#define XPT_FE_SPID_EXT_TABLE_i_ARRAY_BASE              0x25000

// Offset from "BCHP_AUD_FMM_IOP_MISC_REG_START", base via "aud-registers" in device tree
#define AUD_FMM_IOP_MISC_MCLK_CFG_i_ARRAY_BASE		0x20

/* descriptor flags and shifts */
#define MCPB_DW2_LAST_DESC			(1 << 0)
#define MCPB_DW4_PUSH_PARTIAL_PKT	(1 << 28)

#define MEMDMA_DRAM_REQ_SIZE	256

#define MCPB_CHx_SPACING(channel) 	((XPT_MCPB_CH1_REG_START - XPT_MCPB_CH0_REG_START) * (channel))

#define XPT_IO_FORMAT                       0x00400
#define XPT_RMX0_CTRL                       0x02d00

#define XPT_CHANNEL	31
#define PID_CHANNEL	767

#define DMA_TIMEOUT_USECONDS	(100 * 1000)

static inline u32 tetsuo_xpt_read(struct tetsuo_sound *ssound, u32 offset)
{
	return readl(ssound->xpt_base + offset);
}

static inline void tetsuo_xpt_write(struct tetsuo_sound *ssound, u32 offset, u32 value)
{
	writel(value, ssound->xpt_base + offset);
}

static inline void tetsuo_xpt_write_rb(struct tetsuo_sound *ssound, u32 offset, u32 value)
{
	tetsuo_xpt_write(ssound, offset, value);
	(void)tetsuo_xpt_read(ssound, offset);
}

static void mcpb_run(struct tetsuo_sound *ssound, int enable, int channel)
{
	tetsuo_xpt_write_rb(ssound, XPT_MCPB_RUN_SET_CLEAR, (!!enable << 8) | channel);
}

static void init_mcpb_channel(struct tetsuo_sound *ssound, int xpt_channel, int pid_channel)
{
	unsigned long offs = MCPB_CHx_SPACING(xpt_channel);
	unsigned long parser_ctrl = XPT_MCPB_CH0_SP_PARSER_CTRL + offs;
	unsigned long parser_ctrl1 = XPT_MCPB_CH0_SP_PARSER_CTRL1 + offs;
	unsigned long packet_len = XPT_MCPB_CH0_SP_PKT_LEN + offs;
	unsigned long dma_buf_ctrl = XPT_MCPB_CH0_DMA_BBUFF_CTRL + offs;
	unsigned long dma_data_ctrl = XPT_MCPB_CH0_DMA_DATA_CONTROL + offs;
	unsigned long blockout_ctrl = XPT_MCPB_CH0_TMEU_BLOCKOUT_CTRL + offs;

	mcpb_run(ssound, 0, xpt_channel);

	/* setup for block mode */
	tetsuo_xpt_write(ssound, parser_ctrl,
			(1 << 0) |							/* parser enable */
			(6 << 1) |							/* block mode */
			(1 << 4) |							/* all pass pre mpod */
			(xpt_channel << 6) |						/* band ID */
			(1 << 14));							/* select playback parser */
	/* We set the lengths to TETSUO_PERIOD as that guarantees that a whole number of packets
	   will contain a period of samples. */
	tetsuo_xpt_write(ssound, packet_len, RMX_PACKET_SIZE);				/* packet length */
	tetsuo_xpt_write(ssound, dma_buf_ctrl, RMX_PACKET_SIZE);			/* stream feed size */

	tetsuo_xpt_write(ssound, dma_data_ctrl, (MEMDMA_DRAM_REQ_SIZE << 0) |
						(0 << 11));				/* disable run version match */

	{
		// Set the PID used for all pass mode
		int reg = tetsuo_xpt_read(ssound, parser_ctrl1);
		reg &= 0xfffff000;
		reg |= pid_channel;
		tetsuo_xpt_write(ssound, parser_ctrl1, pid_channel);
	}
	tetsuo_xpt_write(ssound, blockout_ctrl, 0);									/* zero blockout */
}

static void xpt_init_ctx(struct tetsuo_sound *ssound, unsigned int xpt_channel, unsigned int pid_channel)
{
	// Clear out error checking in pid parser for our pid
	int config_reg = tetsuo_xpt_read(ssound, XPT_FULL_PID_PARSER_STATE_CONFIG_0_i_ARRAY_BASE + (4 * pid_channel));
	config_reg &= ~0xda000000;
	tetsuo_xpt_write(ssound, XPT_FULL_PID_PARSER_STATE_CONFIG_0_i_ARRAY_BASE + (4 * pid_channel), config_reg);

	/* Accept null packets */
	tetsuo_xpt_write(ssound, XPT_FULL_PID_PARSER_PBP_ACCEPT_ADAPT_00, (1 << xpt_channel));

	/* configure PID channel */
	{
		// The documentation has some private values which we have to write in order for RMX to appear...!
		int reg = 0x1b804000 | (xpt_channel << 16);
		tetsuo_xpt_write(ssound, XPT_FE_PID_TABLE_i_ARRAY_BASE + (4 * pid_channel), reg);
	}

	{
		int reg = tetsuo_xpt_read(ssound, XPT_FE_SPID_TABLE_i_ARRAY_BASE + (4 * pid_channel));
		reg &= 0x00ff0000;
		reg |= (1 << 24); 						/* Send to REMUX0 */
		tetsuo_xpt_write(ssound, XPT_FE_SPID_TABLE_i_ARRAY_BASE + (4 * pid_channel), reg);
	}

	tetsuo_xpt_write(ssound, XPT_FE_SPID_EXT_TABLE_i_ARRAY_BASE + (4 * pid_channel),  0);
}

static void memdma_init_hw(struct tetsuo_sound *ssound, int xpt_channel, int pid)
{
	init_mcpb_channel(ssound, xpt_channel, pid);
	xpt_init_ctx(ssound, xpt_channel, pid);
	mb();
}

static int mcpb_init_descs(struct mcpb_dma_desc *desc, dma_addr_t buf, size_t len, unsigned int pid_channel)
{
	int i;
	pr_err("mcpb_init_descs: %p, %08x %d %04x\n", desc, (int)buf, len, pid_channel);
	
	// We setup two DMA descriptors, both pointing at our temporary buffer and marked as the last DMA in the list.
	// These will be ping ponged as the DMA engine is woken up on each audio DMA.
	for(i = 0; i < 2; ++i)
	{
		memset(desc, 0, sizeof(*desc));

		desc->buf_hi = upper_32_bits(buf);
		desc->buf_lo = lower_32_bits(buf); 			// Physical address of DMA */
		desc->next_offs = MCPB_DW2_LAST_DESC;		// Mark as last DMA in list
		desc->opts1 = MCPB_DW4_PUSH_PARTIAL_PKT;	// Without this, the last packet doesn't appear until the following DMA
		desc->size = len; 							// Length of DMA*/
		
		++desc;
	}

	return 0;
}

static void memdma_start(struct tetsuo_sound *ssound, int channel)
{
	struct mcpb_dma_desc *desc = ssound->descriptor;
	if(ssound->active_descriptor == -1)
	{
		unsigned long reg = XPT_MCPB_CH0_DMA_DESC_CONTROL + MCPB_CHx_SPACING(channel);
		tetsuo_xpt_write(ssound, reg, (uint32_t)ssound->descriptor_dma_address);

		mcpb_run(ssound, 1, channel);
	}
	else
	{
		dma_sync_single_for_cpu(NULL, ssound->descriptor_dma_address, sizeof(struct mcpb_dma_desc) * 2, DMA_TO_DEVICE);
		
		// We're already running, now we want to ping pong and wake
		if(ssound->active_descriptor == 0)
		{
			desc->next_offs = lower_32_bits(ssound->descriptor_dma_address + sizeof(struct mcpb_dma_desc));
			++desc;
			desc->next_offs = MCPB_DW2_LAST_DESC;
		}
		else
		{
			desc->next_offs = MCPB_DW2_LAST_DESC;
			++desc;
			desc->next_offs = lower_32_bits(ssound->descriptor_dma_address);
		}
		dma_sync_single_for_device(NULL, ssound->descriptor_dma_address, sizeof(struct mcpb_dma_desc) * 2, DMA_TO_DEVICE);
		
		tetsuo_xpt_write(ssound, XPT_MCPB_WAKE_SET, (1 << 8) | channel);
		(void)tetsuo_xpt_read(ssound, XPT_MCPB_WAKE_SET);
	}	
	
	++ssound->active_descriptor;
	if(ssound->active_descriptor == 2)
		ssound->active_descriptor = 0;
}

/*

	Tetsuo ALSA

*/
static int tetsuo_pcm_playback_transfer_chunk(struct tetsuo_sound *ssound)
{
	int i;
	int channel;
	
	// Non-interleaved input from ALSA, so interleave and apply volume 

	AudioSample *c1 = (AudioSample *)(ssound->playback_substream->runtime->dma_area + ((ssound->pos + (0 * TETSUO_BUFFER_SIZE)) / CHANNEL_COUNT));
	AudioSample *c2 = (AudioSample *)(ssound->playback_substream->runtime->dma_area + ((ssound->pos + (1 * TETSUO_BUFFER_SIZE)) / CHANNEL_COUNT));
	AudioSample *c3 = (AudioSample *)(ssound->playback_substream->runtime->dma_area + ((ssound->pos + (2 * TETSUO_BUFFER_SIZE)) / CHANNEL_COUNT));
	AudioSample *c4 = (AudioSample *)(ssound->playback_substream->runtime->dma_area + ((ssound->pos + (3 * TETSUO_BUFFER_SIZE)) / CHANNEL_COUNT));
	AudioSample *c5 = (AudioSample *)(ssound->playback_substream->runtime->dma_area + ((ssound->pos + (4 * TETSUO_BUFFER_SIZE)) / CHANNEL_COUNT));
	AudioSample *c6 = (AudioSample *)(ssound->playback_substream->runtime->dma_area + ((ssound->pos + (5 * TETSUO_BUFFER_SIZE)) / CHANNEL_COUNT));

	unsigned int *output_ptr = (unsigned int *)ssound->temp_buffer;
	unsigned int volume_delta[CHANNEL_COUNT];
	bool volume_target_hit = true;

	dma_sync_single_for_cpu(NULL, ssound->dma_buffer, TETSUO_PERIOD_IN_BYTES, DMA_TO_DEVICE);

	// De-zippering of volume changes
	for(channel = 0; channel < CHANNEL_COUNT; ++channel)
	{
		if(ssound->target_pcm_volume_level[channel] > ssound->current_pcm_volume_level[channel])
		{
			volume_delta[channel] = 1;
			volume_target_hit = false;
		}
		else if(ssound->target_pcm_volume_level[channel] < ssound->current_pcm_volume_level[channel])
		{
			volume_delta[channel] = -1;
			volume_target_hit = false;
		}
		else
			volume_delta[channel] = 0;
	}

	// Move the audio data into our DMA buffer, applying volume and byte packing it as we go
	for(i = 0; i < TETSUO_PERIOD; ++i)
	{
		// 1st channel pair
		long long left = *c1++;
		long long right = *c2++;

		left *= ssound->current_pcm_volume_level[0];
		left /= VOLUME_RANGE;
		right *= ssound->current_pcm_volume_level[1];
		right /= VOLUME_RANGE;		

		*output_ptr++ = (unsigned int)left;
		*output_ptr++ = (unsigned int)right;

		// 2nd channel pair
		left = *c3++;
		right = *c4++;

		left *= ssound->current_pcm_volume_level[2];
		left /= VOLUME_RANGE;
		right *= ssound->current_pcm_volume_level[3];
		right /= VOLUME_RANGE;		

		*output_ptr++ = (unsigned int)left;
		*output_ptr++ = (unsigned int)right;

		// 3rd channel pair
		left = *c5++;
		right = *c6++;

		left *= ssound->current_pcm_volume_level[4];
		left /= VOLUME_RANGE;
		right *= ssound->current_pcm_volume_level[5];
		right /= VOLUME_RANGE;		

		*output_ptr++ = (unsigned int)left;
		*output_ptr++ = (unsigned int)right;

		if(unlikely(!volume_target_hit))
		{
			volume_target_hit = true;
			
			// Move volumes towards their target
			for(channel = 0; channel < CHANNEL_COUNT; ++channel)
			{
				if(unlikely(ssound->current_pcm_volume_level[channel] != ssound->target_pcm_volume_level[channel]))
				{
					ssound->current_pcm_volume_level[channel] += volume_delta[channel];
					volume_target_hit = false;
				}
			}
		}
	}

	dma_sync_single_for_device(NULL, ssound->dma_buffer, TETSUO_PERIOD_IN_BYTES, DMA_TO_DEVICE);

	// Kick off the DMA to send out the data
	memdma_start(ssound, XPT_CHANNEL);

	// Update our read position from the DMA buffer
	ssound->pos += TETSUO_PERIOD_IN_BYTES;
	ssound->pos %= TETSUO_BUFFER_SIZE;
	
	return 0;
}

static irqreturn_t tetsuo_quick_interrupt(int irq, void *d)
{
	struct tetsuo_device *tetsuo = d;
	static int update_count = 0;
	
	if(tetsuo->ssound->running)
	{
		if(tetsuo->ssound->playback_substream)
		{
			int count = 0;

			// Send out a block of TETSUO_PERIOD samples
			tetsuo_pcm_playback_transfer_chunk(tetsuo->ssound);
			
			// Poll the GPIO that the interrupt is on to wait for it to go low.
			while(gpio_get_value(tetsuo->gpio_irq))
			{
				udelay(10);
				++count;
				if(count > 10000)
					break;
			}

			// Wake the thread to update the ALSA idea of position 'every so often'
			++update_count;
			if(update_count > ((48000 / 10)/ TETSUO_PERIOD))
			{
				update_count = 0;
				return IRQ_WAKE_THREAD;
			}
			return IRQ_HANDLED;
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t tetsuo_threaded_interrupt(int irq, void *d)
{
	struct tetsuo_device *tetsuo = d;

	if(tetsuo->ssound->running)
	{
		if(tetsuo->ssound->playback_substream)
		{
			// Move ALSA time along.
			snd_pcm_period_elapsed(tetsuo->ssound->playback_substream);
		}
	}

	return IRQ_HANDLED;
}

static int snd_tetsuo_playback_open(struct snd_pcm_substream *substream)
{
	struct tetsuo_sound *ssound = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	snd_pcm_set_sync(substream);

	spin_lock_irq(&ssound->lock);
	if (ssound->playback_substream != NULL)
	{
		spin_unlock_irq(&ssound->lock);
		return -EBUSY;
	}

	ssound->draining = 1;

	ssound->playback_substream = substream;
	spin_unlock_irq(&ssound->lock);

	runtime->hw = snd_tetsuo_info;

	return 0;
}

static int snd_tetsuo_playback_close(struct snd_pcm_substream *substream)
{
	struct tetsuo_sound *ssound = snd_pcm_substream_chip(substream);

	spin_lock_irq(&ssound->lock);
	ssound->playback_substream = NULL;
	
	ssound->period_size = 0;
	ssound->buffer_size = 0;
	
	spin_unlock_irq(&ssound->lock);

	return 0;
}

static int
snd_tetsuo_playback_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	int err;
	struct tetsuo_sound *ssound = snd_pcm_substream_chip(substream);

	spin_lock_irq(&ssound->lock);
	tetsuo_dbg("snd_tetsuo_playback_hw_params: %d bytes\n", params_buffer_bytes(params));

	err = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (err < 0)
	{
		tetsuo_dbg("snd_pcm_lib_malloc_pages failed: %d, %d bytes\n", err, params_buffer_bytes(params));
		return err;
	}
	
	spin_unlock_irq(&ssound->lock);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	substream->runtime->dma_bytes = params_buffer_bytes(params);

	return 0;
}

static int snd_tetsuo_pcm_hw_free(struct snd_pcm_substream *substream)
{
	tetsuo_dbg("%s\n", __FUNCTION__);
	snd_pcm_set_runtime_buffer(substream, NULL);
	return snd_pcm_lib_free_pages(substream);
}

static int snd_tetsuo_playback_prepare(struct snd_pcm_substream *substream)
{
	struct tetsuo_sound *ssound = snd_pcm_substream_chip(substream);

	tetsuo_dbg("%s\n", __FUNCTION__);

	spin_lock_irq(&ssound->lock);

	ssound->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	ssound->period_size = snd_pcm_lib_period_bytes(substream);
	ssound->pos = 0;

	tetsuo_dbg("buffer_size=%d, period_size=%d pos=%d frame_bits=%d\n", ssound->buffer_size,
																		ssound->period_size,
																		ssound->pos,
																	  	substream->runtime->frame_bits);

	spin_unlock_irq(&ssound->lock);
	return 0;
}

static int
snd_tetsuo_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct tetsuo_sound *ssound = snd_pcm_substream_chip(substream);
	int err = 0;

	tetsuo_dbg("%s\n", __FUNCTION__);
	spin_lock(&ssound->lock);

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			tetsuo_dbg("SNDRV_PCM_TRIGGER_START running=%d\n", ssound->running);
			if (!ssound->running)
			{
				// Control the MCLK PLL channel selection to switch up to (512 x fs) for over 48kHz
				if(ssound->playback_substream->runtime->rate > 48000)
				{
					writel(0x00040002, ssound->aud_base + AUD_FMM_IOP_MISC_MCLK_CFG_i_ARRAY_BASE);
				}
				else
				{
					writel(0x00020001, ssound->aud_base + AUD_FMM_IOP_MISC_MCLK_CFG_i_ARRAY_BASE);
				}
				(void)readl(ssound->aud_base + AUD_FMM_IOP_MISC_MCLK_CFG_i_ARRAY_BASE);
				
				// Configure the RMX output - this can't be done earlier, as the registers may get blatted by another user
				
				// Power up RMX
				tetsuo_xpt_write(ssound, XPT_PMU_CLK_CTRL, tetsuo_xpt_read(ssound, XPT_PMU_CLK_CTRL) & ~XPT_PMU_CLK_CTRL_RMX0_DISABLE_MASK);
				tetsuo_xpt_write(ssound, XPT_PMU_CLK_CTRL, tetsuo_xpt_read(ssound, XPT_PMU_CLK_CTRL) & ~XPT_PMU_CLK_CTRL_XPT_IO_DISABLE_MASK);

				// Format RMX output with byte long syncs
				tetsuo_xpt_write(ssound, XPT_IO_FORMAT, 1 << 1);

				// Enable RMX with RMX_PACKET_SIZE byte packets, bypass on, and 54MHz clock
				tetsuo_xpt_write(ssound, XPT_RMX0_CTRL, 0x10000031 | (RMX_PACKET_SIZE << 16));

				// Write the CONTROL_0 reg to set ENABLE low and RESET high (stops outputting audio and clocks and resets FIFO pointers to 0)
				tetsuo_write(ssound, TETSUO_CONTROL, TETSUO_INT_RESET);
				
				// The SPI writes are then clocked by the audio clock, so not exactly synchronous.
				// Delay slightly whilst they take effect
				msleep(1);
				
				// Write the RESET bit low too in order to re-enable the output of the clocks to the DAC6 IC.
				tetsuo_write(ssound, TETSUO_CONTROL, 0);

				msleep(1);
				
				// Read the ID and VERSION registers from the FPGA
				{
					unsigned char id;
					unsigned char version;

					id = tetsuo_read(ssound, TETSUO_ID);
					version = tetsuo_read(ssound, TETSUO_CONTROL);

					tetsuo_err("TETSUO id %02x, version %02x\n", id, version);
					tetsuo_err("IRQ STATUS %02x\n", tetsuo_read(ssound, TETSUO_IRQ));
				}

				// Setup the partial fill and low water mark. We send out 180 samples at a time, so we want an interrupt when we hit (256-180) = 76 samples remaining
#define PARTIAL_FILL_REQUIRED_LEVEL		3							// interrupt resets when this many samples are written
#define LOW_WATERMARK_LEVEL				(256 - TETSUO_PERIOD)
				
				tetsuo_write(ssound, TETSUO_FILL_LOW, ((PARTIAL_FILL_REQUIRED_LEVEL >> 1) << 4) | ((LOW_WATERMARK_LEVEL - 0x7) >> 3));

				// The SPI writes are then clocked by the audio clock, so not exactly synchronous
				// Delay slightly whilst they take effect
				msleep(1);

				memdma_init_hw(ssound, XPT_CHANNEL, PID_CHANNEL);

				// We only need to initialize the descriptors once, as we continually reuse them
				dma_sync_single_for_cpu(NULL, ssound->descriptor_dma_address, sizeof(struct mcpb_dma_desc) * 2, DMA_TO_DEVICE);
				mcpb_init_descs(ssound->descriptor, ssound->dma_buffer, TETSUO_PERIOD_IN_BYTES, PID_CHANNEL);
				ssound->active_descriptor = -1;

				dma_sync_single_for_device(NULL, ssound->descriptor_dma_address, sizeof(struct mcpb_dma_desc) * 2, DMA_TO_DEVICE);

				// Kick off an initial DMA to prime the FPGA 
				tetsuo_pcm_playback_transfer_chunk(ssound);
				msleep(10);

				if (err == 0)
				{
					ssound->running = 1;
					ssound->draining = 1;

					// Write the ENABLE bit to start outputting the new audio samples and enable interrupts
					tetsuo_write(ssound, TETSUO_CONTROL, TETSUO_INT_ENABLE);
					
					tetsuo_dbg("REALLY REALLY STARTed tetsuo!!\n");
				}
				else
				{
					tetsuo_err("Failed to START\n");
				}
			}

			break;

		case SNDRV_PCM_TRIGGER_STOP:
			tetsuo_dbg("SNDRV_PCM_TRIGGER_STOP running=%d draining=%d\n", ssound->running,
																		  substream->runtime->status->state == SNDRV_PCM_STATE_DRAINING);
			// Disable interrupts and audio output
			tetsuo_write(ssound, TETSUO_CONTROL, 0);
			
			if (substream->runtime->status->state == SNDRV_PCM_STATE_DRAINING)
			{
				tetsuo_dbg("DRAINING\n");
				ssound->draining = 1;
			}
			else
			{
				tetsuo_dbg("DROPPING\n");
				ssound->draining = 0;
			}
			if (ssound->running)
			{
				if (err != 0)
					tetsuo_err(" Failed to STOP alsa device (%d)\n", err);
				ssound->running = 0;
			}
			break;
		default:
			err = -EINVAL;
	}

	tetsuo_dbg("%s complete\n", __FUNCTION__);
	
	spin_unlock(&ssound->lock);
	return 0;
}

static snd_pcm_uframes_t
snd_tetsuo_playback_pointer(struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t frames;
	struct tetsuo_sound *ssound = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

/*	tetsuo_dbg("pcm_pointer... (%d) hwptr=%d appl=%d pos=%d\n", 0,
																frames_to_bytes(substream->runtime, substream->runtime->status->hw_ptr),
																frames_to_bytes(substream->runtime, substream->runtime->control->appl_ptr),
																ssound->pos);
*/																

	frames = bytes_to_frames(runtime, ssound->pos);
	
	return frames;
}

static int snd_tetsuo_pcm_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	tetsuo_dbg("snd_tetsuo_pcm_volume_info");
	
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 6;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = VOLUME_RANGE;
	return 0;
}

static int snd_tetsuo_pcm_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct tetsuo_sound *ssound = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int i;
	
	spin_lock_irqsave(&ssound->pcm_volume_level_lock, flags);
	
	for(i = 0; i < 6; ++i)
	{
		ucontrol->value.integer.value[i] = ssound->target_pcm_volume_level[i];
	}
	
	spin_unlock_irqrestore(&ssound->pcm_volume_level_lock, flags);
	return 0;
}

static int snd_tetsuo_pcm_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct tetsuo_sound *ssound = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int change;
	int i;
	
	spin_lock_irqsave(&ssound->pcm_volume_level_lock, flags);

	change = 0;
	for(i = 0; i < 6; ++i)
	{
		change |= (ucontrol->value.integer.value[i] != ssound->target_pcm_volume_level[i]);
		ssound->target_pcm_volume_level[i] = ucontrol->value.integer.value[i];
	}
	spin_unlock_irqrestore(&ssound->pcm_volume_level_lock, flags);

	return change;
}

static struct snd_kcontrol_new snd_tetsuo_pcm_volume_control =
{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "PCM Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.info = snd_tetsuo_pcm_volume_info,
	.get = snd_tetsuo_pcm_volume_get,
	.put = snd_tetsuo_pcm_volume_put
};

static struct snd_pcm_ops snd_tetsuo_playback_ops =
{
	.open =			snd_tetsuo_playback_open,
	.close =		snd_tetsuo_playback_close,
	.ioctl =		snd_pcm_lib_ioctl,
	.hw_params =	snd_tetsuo_playback_hw_params,
	.hw_free =		snd_tetsuo_pcm_hw_free,
	.prepare =		snd_tetsuo_playback_prepare,
	.trigger =		snd_tetsuo_pcm_trigger,
	.pointer =		snd_tetsuo_playback_pointer,
};

static void snd_tetsuo_free(void *private_data)
{
	struct tetsuo_sound *ssound = (struct tetsuo_sound *) private_data;

	if (ssound == NULL)
	{
		return;
	}
}

static void
snd_tetsuo_proc_read(struct snd_info_entry * entry, struct snd_info_buffer *buffer)
{
	struct tetsuo_sound *ssound = (struct tetsuo_sound *) entry->private_data;

	{
		unsigned char id;
		unsigned char version;
		
		id = tetsuo_read(ssound, TETSUO_ID);
		version = tetsuo_read(ssound, TETSUO_CONTROL);

		pr_err("TETSUO id %02x, version %02x\n", id, version);
	}
	{
		unsigned char in_ptr = tetsuo_read(ssound, TETSUO_IN_PTR);
		unsigned char out_ptr = tetsuo_read(ssound, TETSUO_OUT_PTR);
		unsigned char irqs= tetsuo_read(ssound, TETSUO_IRQ);
		
		unsigned char remaining;
		if(in_ptr < out_ptr)
			remaining = (FPGA_FIFO_SIZE - out_ptr) + in_ptr;
		else
			remaining = in_ptr - out_ptr;
			
		pr_err("IN_PTR %d OUT_PTR %d (%d) IRQ_SOURCE %02x\n", in_ptr, out_ptr, remaining, irqs);
	}
}

static void snd_tetsuo_proc_init(struct tetsuo_sound * ssound)
{
	struct snd_info_entry *entry;

	if (! snd_card_proc_new(ssound->card, "tetsuo", &entry))
		snd_info_set_text_ops(entry, ssound, snd_tetsuo_proc_read);
}

static void snd_tetsuo_free_pcm(struct snd_pcm *pcm)
{
	struct tetsuo_sound *ssound = (struct tetsuo_sound *) pcm->private_data;
	ssound->pcm = NULL;
}

static int tetsuo_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;

	// We don't DMA from this buffer - we copy out of it at interrupt time and apply volume
	substream->dma_buffer.bytes = TETSUO_BUFFER_SIZE;
	substream->dma_buffer.area = kzalloc(TETSUO_BUFFER_SIZE, GFP_KERNEL);
	substream->dma_buffer.addr = 0;
	substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_CONTINUOUS;
	substream->dma_buffer.dev.dev = pcm->card->dev;
	substream->dma_buffer.private_data = NULL;

	return 0;
}

static int snd_tetsuo_create(struct tetsuo_sound * ssound)
{
	int err;

	tetsuo_dbg("%s\n", __FUNCTION__);

	/* set up ALSA pcm device */
	if ((err = snd_pcm_new(ssound->card, "Tetsuo PCM", 0, 1, 0, &ssound->pcm)) < 0)
	{
		tetsuo_dbg("Error at line %d\n", __LINE__);
		return err;
	}
	ssound->pcm->private_data = ssound;
	ssound->pcm->private_free = snd_tetsuo_free_pcm;
	strcpy(ssound->pcm->name, "Tetsuo PCM");
	snd_pcm_set_ops(ssound->pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_tetsuo_playback_ops);
	
	// Because we don't DMA directly from the ALSA audio buffer, we allocate 
	// our own which is cache accessible
	if (ssound->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream)
	{
		err = tetsuo_pcm_preallocate_dma_buffer(ssound->pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if(err)
		{
			tetsuo_dbg("Error at line %d\n", __LINE__);
			return err;
		}
	}

	// Allocate temporary buffer which we will process the samples from ALSA into and then DMA from
	ssound->temp_buffer = dma_alloc_coherent(NULL,
											 TETSUO_PERIOD_IN_BYTES,
											 &ssound->dma_buffer,
											 GFP_KERNEL);	
	
	tetsuo_dbg("ssound->temp_buffer %p\n", ssound->temp_buffer);
	tetsuo_dbg("ssound->dma_buffer %08x\n", (unsigned int)ssound->dma_buffer);

	// Allocate descriptor for DMA engine to use
	ssound->descriptor = dma_alloc_coherent(NULL,
										 	sizeof(struct mcpb_dma_desc) * 2,
											&ssound->descriptor_dma_address,
											GFP_KERNEL);
	ssound->active_descriptor = -1;
		
	tetsuo_dbg("ssound->descriptor %p\n", ssound->descriptor);
	tetsuo_dbg("ssound->descriptor_dma_address %08x\n", (unsigned int)ssound->descriptor_dma_address);

	ssound->volctl = snd_ctl_new1(&snd_tetsuo_pcm_volume_control, ssound);
	if ((err = snd_ctl_add(ssound->card, ssound->volctl)) < 0)
	{
		tetsuo_dbg("Error at line %d\n", __LINE__);
		return err;
	}

	/* init proc interface */
	snd_tetsuo_proc_init(ssound);

	ssound->playback_substream = NULL;
	
	return 0;
}

static void snd_tetsuo_card_free(struct snd_card *card)
{
	snd_tetsuo_free(card->private_data);
}


/* 

	Linux driver pieces - firmware loading etc.

*/

static int tetsuo_boot(struct tetsuo_device *tetsuo, const struct firmware *fw)
{
	int result;
	struct device_node *of = tetsuo->spi->dev.of_node;
	u32 cs_override_info[4];
	u32 pinmux_tweaks[3];
	u8 padding[16];

	int gpio_creset_b = of_get_named_gpio_flags(of, "gpio-creset_b", 0, NULL);
	int gpio_cdone = of_get_named_gpio_flags(of, "gpio-cdone", 0, NULL);
	int gpio_sel0 = of_get_named_gpio_flags(of, "gpio-sel0", 0, NULL);
	u32 __iomem *spi_pinmux;
	u32 __iomem *cs_pinmux; 
	
	memset(padding, 0, sizeof(padding));
	tetsuo_dbg("BrightSign Tetsuo GPIOs: sel0=%u, cdone=%u, creset_b=%u\n",
		   gpio_sel0, gpio_cdone, gpio_creset_b);

	if (gpio_creset_b < 0 || gpio_cdone < 0 || gpio_sel0 < 0) {
		tetsuo_err("Failed to determine GPIOs for initialising FPGA\n");
		return -ENODEV;
	}

	result = of_property_read_u32_array(of, "chip-select-override", cs_override_info, sizeof(cs_override_info)/sizeof(cs_override_info[0]));
	if (result < 0) {
		tetsuo_err("Failed to determine chip-select-override\n");
		return -ENODEV;
	}

	tetsuo_dbg("BrightSign Tetsuo chip-select-override: %08x %08x %08x %08x\n",
	       cs_override_info[0],
	       cs_override_info[1],
	       cs_override_info[2],
	       cs_override_info[3]);

	result = of_property_read_u32_array(of, "pinmux-tweaks", pinmux_tweaks, sizeof(pinmux_tweaks)/sizeof(pinmux_tweaks[0]));
	if (result < 0) {
		tetsuo_err("Failed to determine pinmux tweaks %d\n", result);
		return -ENODEV;
	}

	tetsuo_dbg("BrightSign Tetsuo pinmux-tweaks: %08x %08x %08x\n",
		   pinmux_tweaks[0],
		   pinmux_tweaks[1],
		   pinmux_tweaks[2]);

	spi_pinmux = ioremap_nocache(pinmux_tweaks[0], 4);
	if (!spi_pinmux) {
		tetsuo_err("Failed to map memory region for pinmux\n");
		return -ENODEV;
	}

	iowrite32((ioread32(spi_pinmux) & ~pinmux_tweaks[1]) | pinmux_tweaks[2], spi_pinmux);
	iounmap(spi_pinmux);

	/* We only need these GPIOs during booting. We can free them
	 * when we've finished. */
	result = gpio_request(gpio_creset_b, "tetsuo_reset");
	if (result < 0) {
		tetsuo_err("Failed to acquire tetsuo_reset GPIO\n");
		return -ENODEV;
	}

	result = gpio_request(gpio_cdone, "tetsuo_cdone");
	if (result < 0) {
		tetsuo_err("Failed to acquire tetsuo_cdone GPIO\n");
		gpio_free(gpio_creset_b);
		return -ENODEV;
	}

	result = gpio_request(gpio_sel0, "tetsuo_sel0");
	if (result < 0) {
		tetsuo_err("Failed to acquire tetsuo_sel0 GPIO\n");
		gpio_free(gpio_creset_b);
		gpio_free(gpio_cdone);
		return -ENODEV;
	}

	gpio_direction_input(gpio_cdone);

	cs_pinmux = ioremap_nocache(cs_override_info[0], 4);
	if (!cs_pinmux) {
		tetsuo_err("Failed to map memory region\n");
		goto out;
	}
	// Pin mux the chip select as a GPIO so we can override it.
	writel((readl(cs_pinmux) & ~cs_override_info[1]) | cs_override_info[2], cs_pinmux);

	/*
		CS needs to be low at reset to trigger SPI peripheral mode.
		After that, we need to switch CS to be proper SPI CS for
		actually sending the data.
	*/
	gpio_direction_output(gpio_sel0, 0);

	/* Get IRQ we need to use */
	tetsuo->irq = of_irq_get(of, 0);
	if (tetsuo->irq < 0) {
		tetsuo_err("Failed to get IRQ %d\n", tetsuo->irq);
		result = tetsuo->irq;
		goto out;
	} else
		tetsuo_dbg("Got IRQ %d\n", tetsuo->irq);
		
	tetsuo->gpio_irq = of_get_named_gpio_flags(of, "gpio-interrupt", 0, NULL);
	if (tetsuo->gpio_irq < 0) {
		tetsuo_err("Failed to get interrupt GPIO\n");
		return -ENODEV;
	}

	/* Hold reset low for at least 200ns */
	gpio_direction_output(gpio_creset_b, 0);
	ndelay(200);

	/* Now set it high while chip select is so the FPGA wakes
	 * up. */
	gpio_set_value(gpio_creset_b, 1);

	/* Then we need to wait for it to be high for at least	
	 * 800us */
	udelay(800);

	/* Return the chip select to SPI control */
	gpio_direction_input(gpio_sel0);
	writel((readl(cs_pinmux) & ~cs_override_info[1]) | cs_override_info[3], cs_pinmux);

	iounmap(cs_pinmux);

	if (gpio_get_value(gpio_cdone)) {
		tetsuo_err("Unexpected state at boot: CDONE=1\n");
		result = -ENODEV;
		goto out;
	}

	/* Now we can start clocking out the data. */
	result = spi_write(tetsuo->spi, fw->data, fw->size);
	if (result) {
		tetsuo_err("SPI bitstream send failure: %d\n", result);
		goto out;
	}

	/* Now we need to send at least 49 more clock
	 * cycles with arbitrary data to get things
	 * going */
	result = spi_write(tetsuo->spi, padding, sizeof(padding));
	if (result) {
		tetsuo_err("SPI padding send failure: %d\n", result);
		goto out;
	}

	/* Now CDONE should be high to indicate success */
	if (!gpio_get_value(gpio_cdone)) {
		tetsuo_err("No CDONE\n");
		result = -ENODEV;
		goto out;
	}

	/* Everything has been reset, so install interrupt handler */
	tetsuo_dbg("Request interrupt %d\n", tetsuo->irq);
	result = request_threaded_irq(tetsuo->irq, tetsuo_quick_interrupt, tetsuo_threaded_interrupt, IRQF_TRIGGER_HIGH, "tetsuo", tetsuo);
	if (result < 0)
	{
		tetsuo_err("Failed to request Tetsuo IRQ: %d (%d)\n", tetsuo->irq, result);
		return result;
	}

out:
	gpio_free(gpio_cdone);
	gpio_free(gpio_creset_b);
	gpio_free(gpio_sel0);

	tetsuo_dbg("tetsuo_boot result %d\n", result);
	return result;
}

#if DESCRIBE_BITSTREAM
static inline char hex_digit(u8 nibble)
{
	if (nibble < 10)
		return '0' + nibble;
	else
		return 'a' + nibble - 10;
}

static void tetsuo_describe_firmware(const struct firmware *fw)
{
#if defined(CONFIG_CRYPTO_MD5)
	int result;
	struct crypto_hash *tfm = crypto_alloc_hash("md5", 0, CRYPTO_ALG_ASYNC);
	if (tfm) {
		u8 hash_bin[16];
		struct scatterlist sg[1];
		struct hash_desc desc = { .tfm = tfm };
		size_t hash_size = crypto_hash_digestsize(tfm);
		BUG_ON(hash_size > ARRAY_SIZE(hash_bin));

		result = crypto_hash_init(&desc);
		if (!result) {
			sg_init_table(sg, ARRAY_SIZE(sg));
			sg_set_buf(&sg[0], fw->data, fw->size);
			result = crypto_hash_digest(&desc, sg, fw->size, hash_bin);
			if (!result) {
				size_t i;
				char hash_str[ARRAY_SIZE(hash_bin)*2 + 1];
				for(i = 0; i < crypto_hash_digestsize(tfm); ++i) {
					hash_str[i*2] = hex_digit(hash_bin[i] >> 4);
					hash_str[i*2+1] = hex_digit(hash_bin[i] & 0xf);
				}
				hash_str[hash_size*2] = 0;

				pr_info("Initialising Tetsuo with bitstream of size %zu, MD5=%s\n", fw->size, hash_str);
				return;

			}
		}

		crypto_free_hash(tfm);
	}
#endif
	pr_info("Initialising Tetsuo with bitstream of size %zu\n", fw->size);
}
#endif // DESCRIBE_BITSTREAM

static void tetsuo_firmware_cont(const struct firmware *fw, void *context)
{
	struct tetsuo_device *tetsuo = context;
	struct tetsuo_sound *ssound;
	struct snd_card *card;
	struct spi_device *spi = tetsuo->spi;

	int result;
	int retries = 5;
	if (!fw)
	{
		// Can't find the firmware, so hold the FPGA in reset
		int gpio_creset_b = of_get_named_gpio_flags(spi->dev.of_node, "gpio-creset_b", 0, NULL);
		if (gpio_creset_b >= 0)
		{
			result = gpio_request(gpio_creset_b, "tetsuo_reset");
			if (result >= 0)
			{
				// Set the reset low
				gpio_direction_output(gpio_creset_b, 0);
				gpio_free(gpio_creset_b);
			}
		}

		tetsuo_err("Userspace failed to provide Tetsuo firmware\n");
		return;
	}

#if DESCRIBE_BITSTREAM
	tetsuo_describe_firmware(fw);
#endif

	do {
		result = tetsuo_boot(tetsuo, fw);
	} while (result && (retries-- > 0));

	/* We've finished with the firmware now regardless. */
	release_firmware(fw);

	if (result < 0) {
		tetsuo_err("Failed to initialise Tetsuo\n");
//		BUG();
		return;
	}

	////////////////////////////
	//
	// Sound card
	//
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
	tetsuo_dbg("Call snd_card_create\n");
	result = snd_card_create(index, id, THIS_MODULE, sizeof(struct tetsuo_sound), &card);
#else
	tetsuo_dbg("Call snd_card_new\n");
	result = snd_card_new(&spi->dev, index, id, THIS_MODULE, sizeof(struct tetsuo_sound), &card);
#endif
	if (result < 0)
		return;
	card->private_free = snd_tetsuo_card_free;
	ssound = (struct tetsuo_sound *) card->private_data;
	ssound->card = card;
	ssound->device = tetsuo;

	u32 xpt_regs_range[2];
	result = of_property_read_u32_array(spi->dev.of_node, "xpt-registers", xpt_regs_range, sizeof(xpt_regs_range)/sizeof(xpt_regs_range[0]));
	if (result < 0) {
		tetsuo_err("Failed to find XPT register window\n");
		return;
	}

	ssound->xpt_base = ioremap_nocache(xpt_regs_range[0], xpt_regs_range[1]);
	if (!ssound->xpt_base) {
		tetsuo_err("Failed to map memory for XPT register window\n");
		return;
	}

	u32 aud_regs_range[2];
	result = of_property_read_u32_array(spi->dev.of_node, "aud-registers", aud_regs_range, sizeof(aud_regs_range)/sizeof(aud_regs_range[0]));
	if (result < 0) {
		tetsuo_err("Failed to find AUD register window\n");
		return;
	}

	ssound->aud_base = ioremap_nocache(aud_regs_range[0], aud_regs_range[1]);
	if (!ssound->aud_base) {
		tetsuo_err("Failed to map memory for AUD register window\n");
		return;
	}

	// Reset our volumes
	memset(ssound->target_pcm_volume_level, 0, sizeof(ssound->target_pcm_volume_level));
	memset(ssound->current_pcm_volume_level, 0, sizeof(ssound->current_pcm_volume_level));
	
	tetsuo->ssound = ssound;
	snd_card_set_dev(card, &spi->dev);
	tetsuo_dbg("Call snd_tetsuo_create\n");
	if ((result = snd_tetsuo_create(ssound)) < 0)
	{
		snd_card_free(card);
		return;
	}

	strcpy(card->driver, "Tetsuo");
	strcpy(card->longname, "BrightsignTetsuo");

	tetsuo_dbg("Call snd_card_register: %p %p %d\n", card, card->dev, card->number);
	if ((result = snd_card_register(card)) < 0)
	{
		snd_card_free(card);
		return;
	}

	pr_info("BrightSign Tetsuo ready\n");
}

static int tetsuo_probe(struct spi_device *spi)
{
	static int created;
	int result;
	struct tetsuo_device *tetsuo;
	
	tetsuo_dbg("probe\n");

	if(created)
	{
		tetsuo_err("Tetsuo has already been created\n");
		return -ENODEV;
	}

	tetsuo = kzalloc(sizeof(struct tetsuo_device), GFP_KERNEL);
	if (!tetsuo)
		return -ENOMEM;

	tetsuo->irq = -1;
	tetsuo->spi = spi;
	dev_set_drvdata(&spi->dev, tetsuo);

	tetsuo_dbg("Requesting Tetsuo firmware\n");
	result = request_firmware_nowait(THIS_MODULE,
					 FW_ACTION_HOTPLUG, "tetsuo.bin", &spi->dev,
					 GFP_KERNEL, tetsuo, tetsuo_firmware_cont);

	if (result) {
		tetsuo_err("Failed to request Tetsuo firmware\n");
		goto fail;
	}
	created = 1;

	return 0;
fail:
	tetsuo_err("Freeing tetsuo_device %p\n", tetsuo);
	kfree(tetsuo);
	return -ENODEV;
}

static int tetsuo_remove(struct spi_device *spi)
{
	tetsuo_dbg("tetsuo_remove\n");
	return 0;
}

static const struct of_device_id tetsuo_of_match[] = {
	{ .compatible = "brightsign,tetsuo" },
	{},
};

static struct spi_driver tetsuo_spi_driver = {
	.driver = {
		.name	= "brightsign-tetsuo",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = tetsuo_of_match,
	},
	.probe	= tetsuo_probe,
	.remove	= tetsuo_remove,
};

module_spi_driver(tetsuo_spi_driver);

MODULE_DESCRIPTION("BrightSign Tetsuo FPGA Platform driver");
MODULE_AUTHOR("BrightSign Digital Ltd.");
MODULE_LICENSE("GPL v2");
