/*****************************************************************************/
/* Copyright (c) 2009 NXP B.V.                                               */
/*                                                                           */
/* This program is free software; you can redistribute it and/or modify      */
/* it under the terms of the GNU General Public License as published by      */
/* the Free Software Foundation, using version 2 of the License.             */
/*                                                                           */
/* This program is distributed in the hope that it will be useful,           */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of            */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              */
/* GNU General Public License for more details.                              */
/*                                                                           */
/* You should have received a copy of the GNU General Public License         */
/* along with this program; if not, write to the Free Software               */
/* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307       */
/* USA.                                                                      */
/*                                                                           */
/*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/timedoctor.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>

#include <asm/time.h>
#include <asm/brcmstb/brcmstb.h>

/******************************************************************************
* LOCAL MACROS                                                                *
*******************************************************************************/

#define TIME_DOCTOR_DESCRIPTION "Time doctor event logger"
#define TIME_DOCTOR_DEVNAME "timedoctor"

//  ----------------------- Log MODULE Debug Level  ------------------------
static int debug_level = 0;
module_param(debug_level, int, 0);
MODULE_PARM_DESC(debug_level, "Debug level (0-1)");

/**** Module Setup ****/
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(TIME_DOCTOR_DESCRIPTION);
MODULE_AUTHOR("NXP B.V.");

#define NB_DATA              (CONFIG_TIME_DOCTOR_SAMPLES)
#define NB_FIELDS_PER_RECORD (4)
#define DATA_SIZE            (NB_DATA*sizeof(timeDoctorRecord_t))

/******************************************************************************
* LOCAL TYPEDEFS                                                              *
*******************************************************************************/
typedef unsigned int timeDoctorRecord_t[NB_FIELDS_PER_RECORD];

/******************************************************************************
* STATIC FUNCTION PROTOTYPES                                                  *
*******************************************************************************/

/* Declare funcions needed by the structures below */
static void   *timeDoctor_SeqStart(struct seq_file *, loff_t *);
static void   *timeDoctor_SeqNext(struct seq_file *, void *, loff_t *);
static void    timedoctor_SeqStop(struct seq_file *, void *);
static int     timedoctor_SeqShow(struct seq_file *, void *);

#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
static void   *timeDoctor_LastSeqStart(struct seq_file *, loff_t *);
static void   *timeDoctor_LastSeqNext(struct seq_file *, void *, loff_t *);
static void    timedoctor_LastSeqStop(struct seq_file *, void *);
static int     timedoctor_LastSeqShow(struct seq_file *, void *);
static int     timeDoctor_LastOpenProc(struct inode*, struct file *);
#endif // CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT

static int     timeDoctor_OpenProc(struct inode*, struct file *);
static ssize_t timeDoctor_WriteProc(struct file*, const char __user *, size_t, loff_t *);

/******************************************************************************
* EXPORTED DATA                                                               *
*******************************************************************************/

/******************************************************************************
* LOCAL DATA                                                                  *
*******************************************************************************/

#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
/* static location for time doctor data so that it is preserved after
 * a reboot. Ensure it is aligned to a cache line. */
timeDoctorRecord_t gtimeDoctorStaticData[NB_DATA] __attribute__ ((section (".bss_noinit"), aligned(32)));
#define MAYBE_PRESERVE __attribute__ ((section (".bss_noinit")))

/* where the last load of time doctor data is copied to at boot time. */
static timeDoctorRecord_t *gtimeDoctorLastData;
static unsigned int gtimeDoctorLastIndex;
static unsigned int gtimeDoctorLastDataWrapped;
#else
#define MAYBE_PRESERVE
#endif // !CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT

/* This array stores NB_DATA records, each of which is 4 words */
timeDoctorRecord_t *gtimeDoctorData MAYBE_PRESERVE;

#if !defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
static struct page  *gtimeDoctorSharedPage;
#endif // CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT

/* Index pointing to the next available word in the array */
unsigned int gtimeDoctorIndex MAYBE_PRESERVE = 0;

/* Indicates whether we're allowed to wrap around our buffer.
   Default value is on. */
unsigned int gtimeDoctorDataWrapAllowed = 1;

/* Indicates whether we've wrapped around our buffer. */
unsigned int gtimeDoctorDataWrapped MAYBE_PRESERVE = 0;

/* Operations for reading /proc/timedoctor one record at a time */
static struct seq_operations timedoctor_seq_ops = {
    .start = timeDoctor_SeqStart,
    .next  = timeDoctor_SeqNext,
    .stop  = timedoctor_SeqStop,
    .show  = timedoctor_SeqShow
};

/* Operations for reading/writing /proc/timedoctor */
static struct file_operations timedoctor_proc_fops = {
    .owner   = THIS_MODULE,
    .open    = timeDoctor_OpenProc,
    .read    = seq_read,
    .write   = timeDoctor_WriteProc,
    .llseek  = seq_lseek,
    .release = seq_release
};

#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
/* Operations for reading /proc/last_timedoctor one record at a time */
static struct seq_operations timedoctor_last_seq_ops = {
    .start = timeDoctor_LastSeqStart,
    .next  = timeDoctor_LastSeqNext,
    .stop  = timedoctor_LastSeqStop,
    .show  = timedoctor_LastSeqShow
};

/* Operations for reading /proc/last_timedoctor */
static struct file_operations timedoctor_proc_last_fops = {
		.owner = THIS_MODULE,
		.open = timeDoctor_LastOpenProc,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = seq_release
};
#endif // CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT

static spinlock_t td_spinlock = SPIN_LOCK_UNLOCKED;

/******************************************************************************
* FUNCTION IMPLEMENTATION                                                     *
*******************************************************************************/

static int
timeDoctor_BufferInit(
        void)
{
#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
    gtimeDoctorLastData = vmalloc(DATA_SIZE);
    gtimeDoctorLastIndex = gtimeDoctorIndex;
    gtimeDoctorLastDataWrapped = gtimeDoctorDataWrapped;

    /* Copy from the uninitialised static buffer to our copy buffer
     * and then clear out the uninitialised buffer ready to be used
     * again. */
    memcpy(gtimeDoctorLastData, gtimeDoctorStaticData, DATA_SIZE);
    memset(gtimeDoctorStaticData, 0, DATA_SIZE);

    /* Now we're free to use the buffer */
    gtimeDoctorData = gtimeDoctorStaticData;

    pr_info("Static time doctor buffer from %p to %p\n", gtimeDoctorData, gtimeDoctorData + NB_DATA);
#else
    gtimeDoctorData = (timeDoctorRecord_t*)__get_free_pages(GFP_KERNEL, get_order(DATA_SIZE));
    gtimeDoctorSharedPage = virt_to_page(gtimeDoctorData);
#endif

    if (gtimeDoctorData == 0)
    {
        printk(KERN_ERR "Unable to allocate timedoctor memory\n");
    }

    return 0;
}    /* Log init */

/* seq_read takes care of all reading for us */
static int timeDoctor_OpenProc(struct inode* inode, struct file *file)
{
    return seq_open(file, &timedoctor_seq_ops);
}

#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
/* seq_read takes care of all reading for us */
static int timeDoctor_LastOpenProc(struct inode* inode, struct file *file)
{
    return seq_open(file, &timedoctor_last_seq_ops);
}
#endif // CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT

/* Start a seq read, convert initial record# to pointer */
static void *timeDoctor_SeqStart(struct seq_file *s, loff_t *pos)
{
    loff_t localPos = *pos;
    unsigned int dataLimit = gtimeDoctorIndex;

    if (gtimeDoctorDataWrapped)
    {
        localPos = (gtimeDoctorIndex + (*pos)) % NB_DATA;
        dataLimit = NB_DATA;
    }

    if ((*pos) >= dataLimit)
        return NULL; /* No more to read */
    return gtimeDoctorData + localPos;
}

/* Increment to next record# & convert to pointer */
static void *timeDoctor_SeqNext(struct seq_file *s, void *v, loff_t *pos)
{
    loff_t localPos;
    unsigned int dataLimit = gtimeDoctorIndex;

    (*pos)++;
    localPos = *pos;
    if (gtimeDoctorDataWrapped)
    {
        localPos = (gtimeDoctorIndex + (*pos)) % NB_DATA;
        dataLimit = NB_DATA;
    }
    if ((*pos) >= dataLimit)
        return NULL;
    return gtimeDoctorData + localPos;
}

static void timedoctor_SeqStop(struct seq_file *f, void *v)
{
    /* Nothing to do */
}

/* Display a single record, from pointer prepared by start/next */
static int timedoctor_SeqShow(struct seq_file *s, void *v)
{
    unsigned int* rec = (unsigned int*)v;
    seq_printf(s, "%08x %08x %08x %08x\n", rec[0], rec[1], rec[2], rec[3] );
    return 0;
}

#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
/* Start a seq read, convert initial record# to pointer */
static void *timeDoctor_LastSeqStart(struct seq_file *s, loff_t *pos)
{
    loff_t localPos = *pos;
    unsigned int dataLimit = gtimeDoctorLastIndex;

    if (gtimeDoctorLastDataWrapped)
    {
        localPos = (gtimeDoctorLastIndex + (*pos)) % NB_DATA;
        dataLimit = NB_DATA;
    }

    if ((*pos) >= dataLimit)
        return NULL; /* No more to read */
    return gtimeDoctorLastData + localPos;
}

/* Increment to next record# & convert to pointer */
static void *timeDoctor_LastSeqNext(struct seq_file *s, void *v, loff_t *pos)
{
    loff_t localPos;
    unsigned int dataLimit = gtimeDoctorLastIndex;

    (*pos)++;
    localPos = *pos;
    if (gtimeDoctorLastDataWrapped)
    {
        localPos = (gtimeDoctorLastIndex + (*pos)) % NB_DATA;
        dataLimit = NB_DATA;
    }
    if ((*pos) >= dataLimit)
        return NULL;
    return gtimeDoctorLastData + localPos;
}

static void timedoctor_LastSeqStop(struct seq_file *f, void *v)
{
    /* Nothing to do */
}

/* Display a single record, from pointer prepared by start/next */
static int timedoctor_LastSeqShow(struct seq_file *s, void *v)
{
    unsigned int* rec = (unsigned int*)v;
    seq_printf(s, "%08x %08x %08x %08x\n", rec[0], rec[1], rec[2], rec[3] );
    return 0;
}
#endif // CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT

static ssize_t
timeDoctor_WriteProc(
        struct file          * file,
        const char __user    * buffer,
        size_t                 count,
        loff_t               * data)
{
    char command[10];

    if (!count)
    {
        return 0;
    }
    else if(count > 10)
    {
        printk("timeDoctor_WriteProc: Problem ! count = %u\n", count);
        return -EFAULT;
    }

	memset(command, ' ', sizeof(command));

    if (copy_from_user(&command, buffer, count))
    {
        return -EFAULT;
    }

    if (!strncmp(command, "off", 3))
    {
        timeDoctor_SetLevel(0);
        printk("debug_level:off\n");
    }
    else if (!strncmp(command, "on", 2))
    {
        timeDoctor_SetLevel(1);
        printk("debug_level:on\n");
    }
    else if (!strncmp(command, "wrapon", 6))
    {
        gtimeDoctorDataWrapAllowed = 1;
        printk("wrap:on\n");
    }
    else if (!strncmp(command, "wrapoff", 7))
    {
        gtimeDoctorDataWrapAllowed = 0;
        gtimeDoctorDataWrapped = 0;
        printk("wrap:off\n");
    }
    else if (!strncmp(command, "status", 6))
    {
        if (debug_level)
        {
            printk("debug_level:on\n");
        }
        else
        {
            printk("debug_level:off\n");
        }
    }
    else if (!strncmp(command, "reset", 5))
    {
        timeDoctor_Reset();
    }
    else if (count >= 9)
    {
		unsigned char *ptr = (unsigned char *)(command + 1);
		if(command[0] == 'E')
		{
			timeDoctor_Info(TDI_COMMAND(TDI_GENERIC, TDI_STOP), CHAR_ARRAY_TO_U32((command + 1)), CHAR2_ARRAY_TO_U32(ptr));
		}
		else if(command[0] == 'O')
		{
			timeDoctor_Info(TDI_COMMAND(TDI_START, TDI_EVENT), CHAR2_ARRAY_TO_U32((command+1)), CHAR_ARRAY_TO_U32((command+1)));
		}
		else if(command[0] == 'C')
		{
			timeDoctor_Info(TDI_COMMAND(TDI_STOP, TDI_EVENT), CHAR2_ARRAY_TO_U32((command+1)), CHAR_ARRAY_TO_U32((command+1)));
		}
		else if(command[0] == 'N')
		{
			// Force naming of this user space task
			timeDoctor_task_create(current->pid, (command + 1));
		}
    }

    return count;
} /* End of timeDoctor_WriteProc */

/*=============================================================================
| Module functions                                                            |
==============================================================================*/

void
timeDoctor_Info(
        unsigned int data1,
        unsigned int data2,
        unsigned int data3)
{
    unsigned int time;
    unsigned long status;

    spin_lock_irqsave (&td_spinlock, status);

    if (debug_level)
    {
        /* Read the time counter (config: free running counter) */
        struct wktmr_time t;
		wktmr_read (&t);
		time = t.pre + (WKTMR_FREQ * t.sec);

		// Add the CPU id into the type field (bit 23)
		if(smp_processor_id())
		{
			data1 |= 0x00800000;
			if((data1 & 0x007f0000) == TDI_TASK)
			{
				// Set the bottom bit of data so that the TID is unique per CPU
				data1 |= 0x00000001;
			}
		}

        gtimeDoctorData[gtimeDoctorIndex][0] = time;
        gtimeDoctorData[gtimeDoctorIndex][1] = data1;
        gtimeDoctorData[gtimeDoctorIndex][2] = data2;
        gtimeDoctorData[gtimeDoctorIndex][3] = data3;
		dma_cache_wback((unsigned long)(gtimeDoctorData[gtimeDoctorIndex]), sizeof(gtimeDoctorData[0]));
        gtimeDoctorIndex++;
		dma_cache_wback((unsigned long)(&gtimeDoctorIndex), sizeof(gtimeDoctorIndex));

        if (gtimeDoctorIndex >= NB_DATA)
        {
            if (gtimeDoctorDataWrapAllowed)
            {
                gtimeDoctorIndex = 0;
                gtimeDoctorDataWrapped = 1;
				dma_cache_wback((unsigned long)(&gtimeDoctorDataWrapped), sizeof(gtimeDoctorDataWrapped));
            }
            else
            {
                debug_level = 0;
            }
        }
    }

    spin_unlock_irqrestore (&td_spinlock, status);
} /* End of log_info */

EXPORT_SYMBOL(timeDoctor_Info);

void
timeDoctor_SetLevel(
        int level)
{
    unsigned long status;
    spin_lock_irqsave (&td_spinlock, status);
    debug_level = level;
    spin_unlock_irqrestore (&td_spinlock, status);
} /* End of timeDoctor_SetLevel */
EXPORT_SYMBOL(timeDoctor_SetLevel);

void
timeDoctor_Reset(
        void)
{
    unsigned long status;
    spin_lock_irqsave (&td_spinlock, status);
    gtimeDoctorIndex = 0;
    gtimeDoctorDataWrapped = 0;
    spin_unlock_irqrestore (&td_spinlock, status);
} /* End of timeDoctor_Reset */
EXPORT_SYMBOL(timeDoctor_Reset);

static int timeDoctor_GetEntries(void)
{
    if (gtimeDoctorDataWrapped)
    {
        return NB_DATA * NB_FIELDS_PER_RECORD;
    }
    else
    {
        return gtimeDoctorIndex * NB_FIELDS_PER_RECORD;
    }
}

static long timedoctor_Ioctl( struct file           *filp,
                             unsigned int           cmd,
                             unsigned long          arg );

#if !defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
static int timedoctor_Mmap ( struct file           *file,
                             struct vm_area_struct *vma );
#endif

static struct file_operations gtimeDoctorFops = {
     .unlocked_ioctl =  timedoctor_Ioctl,
#if !defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
     .mmap  =  timedoctor_Mmap
#endif
};

static struct miscdevice gtimeDoctorMiscDev = {
     .minor =    MISC_DYNAMIC_MINOR,
     .name  =    TIME_DOCTOR_DEVNAME,
     .fops  =   &gtimeDoctorFops
};

static long timedoctor_Ioctl( struct file   *filp,
                             unsigned int   cmd,
                             unsigned long  arg )
{
    switch (cmd)
    {
        case TIMEDOCTOR_IOCTL_RESET:
            timeDoctor_Reset();
            return 0;

        case TIMEDOCTOR_IOCTL_START:
            timeDoctor_SetLevel(1);
            return 0;

        case TIMEDOCTOR_IOCTL_STOP:
            timeDoctor_SetLevel(0);
            return 0;

        case TIMEDOCTOR_IOCTL_GET_ENTRIES:
            return timeDoctor_GetEntries();

        case TIMEDOCTOR_IOCTL_GET_MAX_ENTRIES:
            return (NB_DATA*NB_FIELDS_PER_RECORD);

        case TIMEDOCTOR_IOCTL_INFO:
            {
                unsigned int data[TIMEDOCTOR_INFO_DATASIZE];
                copy_from_user(&data, (unsigned int *) arg, sizeof(unsigned int) * TIMEDOCTOR_INFO_DATASIZE);
                timeDoctor_Info(data[0], data[1], data[2]);
            }
            return 0;
    }

    return -ENOSYS;
}

#if !defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
static int timedoctor_Mmap ( struct file           *file,
                             struct vm_area_struct *vma )
{
     unsigned int size;
     if (vma->vm_pgoff != 0)
     {
          return -EINVAL;
     }

     size = vma->vm_end - vma->vm_start;
     if (size != PAGE_ALIGN(DATA_SIZE))
     {
          return -EINVAL;
     }

     /* Prevent the swapper from considering these pages for swap and touching them */
     vma->vm_flags |= VM_RESERVED | VM_DONTEXPAND;

     return remap_pfn_range(vma, vma->vm_start, page_to_pfn(gtimeDoctorSharedPage), size, vma->vm_page_prot);
}
#endif

/** Module initialisation. */
static int __init
timeDoctor_Init(
        void)
{
    int ret;

    static struct proc_dir_entry *entry;

    timeDoctor_BufferInit();
    timeDoctor_Reset();

#if defined(CONFIG_TIME_DOCTOR_AUTO_START)
	// auto start with wrap enabled
	gtimeDoctorDataWrapAllowed = 1;
	timeDoctor_SetLevel(1);
#endif // CONFIG_TIME_DOCTOR_AUTO_START

    /* Register a misc device called "timedoctor". */
    ret = misc_register( &gtimeDoctorMiscDev );
    if (ret < 0)
    {
        printk("can't register misc device (minor %d)!\n", gtimeDoctorMiscDev.minor );
        return ret;
    }

    entry = create_proc_entry(TIME_DOCTOR_DEVNAME,
            S_IFREG | S_IRUGO | S_IWUSR,           // protection mode
            NULL);       // parent dir: /proc

    if (!entry)
    {
        printk("%screate_proc_entry : failed\n",
                TIME_DOCTOR_DESCRIPTION);
        return -ENOMEM;
    }
    else
    {
        entry->proc_fops = &timedoctor_proc_fops;
    }

#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
	entry = create_proc_entry("last_" TIME_DOCTOR_DEVNAME,
							  S_IFREG | S_IRUGO | S_IWUSR, NULL);
	if (!entry)
	{
        printk("%screate_proc_entry : failed\n",
                TIME_DOCTOR_DESCRIPTION);
        return -ENOMEM;
	}
	else
	{
		entry->proc_fops = &timedoctor_proc_last_fops;
	}
#endif // CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT

    printk("%s (%s-%s) [%i events]\n", TIME_DOCTOR_DESCRIPTION, __DATE__, __TIME__, NB_DATA);
    return 0;
} /* End of timeDoctor_Init */

/** Module deinitialisation */
static void __exit
timeDoctor_Exit(
        void)
{
    printk("Exiting %s\n", TIME_DOCTOR_DESCRIPTION);
    debug_level = 0;

    misc_deregister( &gtimeDoctorMiscDev );

#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
    if (gtimeDoctorLastData) {
	    vfree(gtimeDoctorLastData);
	    gtimeDoctorLastData = NULL;
    }
#else
    ClearPageReserved(gtimeDoctorSharedPage);
    free_pages((int)gtimeDoctorData, get_order(DATA_SIZE));
#endif

    /* undo proc stuff */
    remove_proc_entry(TIME_DOCTOR_DEVNAME, NULL);
#if defined(CONFIG_TIME_DOCTOR_PRESERVE_OVER_REBOOT)
    remove_proc_entry("last_" TIME_DOCTOR_DEVNAME, NULL);
#endif

} /* End of timeDoctor_Exit */

module_init(timeDoctor_Init);
module_exit(timeDoctor_Exit);
