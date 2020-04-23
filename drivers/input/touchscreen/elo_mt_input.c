// --------------------------------------------------------------
// Name:    Linux input module for Elo MultiTouch(MT) devices
// License: GPL
// Author:  Elo Touch Solutions Inc
// Version: 1.0.0 
// Date:    26 August 2013
// --------------------------------------------------------------

//Header Files
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38))
#include <linux/input/mt.h>
#else
#include <linux/input.h>
#endif

//Driver Information
#define DRIVER_VERSION  "1.0.0"
#define DRIVER_AUTHOR   "Elo Touch Solutions Inc and BrightSign LLC"
#define DRIVER_DESC     "Linux input module for Elo MultiTouch(MT) devices"
#define DRIVER_LICENSE  "GPL"

// Kernel Module Information
MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

// Global Definitions
#ifdef CONFIG_BRIGHTSIGN	
/* All BrightSign kernels support multi-touch (with the latest input_mt_init_slots()) */
#define _MULTITOUCH_KERNEL_
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38))
  #define _MULTITOUCH_KERNEL_
#else
  #undef _MULTITOUCH_KERNEL_
#endif
#endif


#ifdef _MULTITOUCH_KERNEL_
#define ELO_MT_MAXCONTACTS 2	// Maximum number of multi touch contacts
#endif

#define MIN_X (0)		// Min X Axis resolution
#define MAX_X (4095)		// Max X Axis resolution

#define MIN_Y (0)		// Min Y Axis resolution
#define MAX_Y (4095)		// Max Y Axis resolution

#define MIN_PRESSURE (0)	// Min Pressure
#define MAX_PRESSURE (1)	// Max Pressure

// Global Data Structures
struct touch_event {
	u16 touch_count;	// Total touch count in the entire frame
	u16 touch_index;	// Current Touch Index for this contact
	u16 contact_id;		// Contact ID for this contact
	u16 touch_status;	// Indicates if touch is active or not for this contact
	u16 x;			// ABS X value for this contact
	u16 y;			// ABS Y value for this contact
};

struct input_dev *elo_input_device = NULL;
struct proc_dir_entry *proc_entry = NULL;
struct proc_dir_entry *proc_control_entry = NULL;

#ifdef __MIPSEL__
/* The MIPS user-space daemon flips the axes, so we put them back */
static int flip_axis = 1;
#else
static int flip_axis = 0;
#endif
module_param(flip_axis, int, 0);

DEFINE_MUTEX(elo_mutex);

// Functions
static int elo_create_device(void);
static int elo_destroy_device(void);

static ssize_t elo_input_read(struct file *file, char __user * buf,
			      size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t elo_input_write(struct file *file, const char __user * buf,
			       size_t count, loff_t * ppos)
{
	int rc = -1;
	const u8 *ptr = NULL;
	u8 *buffer = NULL;
	size_t size = count;
	struct touch_event *event = NULL;

	if (!(buffer = kzalloc(count, GFP_KERNEL))) {
		printk(KERN_ERR
		       "elo_mt_input: Unable to allocate memory\n");
		rc = -ENOMEM;
		goto done;
	}

	if (copy_from_user(buffer, buf, count)) {
		printk(KERN_ERR "elo_mt_input: Unable to copy data\n");
		rc = -EFAULT;
		goto done;
	}

	mutex_lock(&elo_mutex);

	/* Process each touch event */
	ptr = buffer;

	while (size >= sizeof(*event)) {
		event = (struct touch_event *) ptr;
		// printk(KERN_INFO "elo_mt_input: mt event touch#:%d cid=%d x=%d y=%d status=%d\n", event->touch_index, event->contact_id, event->x, event->y, event->touch_status);

		if (flip_axis) {
			event->x = MAX_X - event->x;
			event->y = MAX_Y - event->y;
		}

#ifdef _MULTITOUCH_KERNEL_
		// Actions for sending Multitouch events  
		input_mt_slot(elo_input_device, event->contact_id);	//Use contact_id for slot numbering [ max 2 slots ]
		input_mt_report_slot_state(elo_input_device,
					   MT_TOOL_FINGER,
					   event->touch_status);

		if (event->touch_status) {	// If touch is active                  
			input_event(elo_input_device, EV_ABS,
				    ABS_MT_POSITION_X, event->x);
			input_event(elo_input_device, EV_ABS,
				    ABS_MT_POSITION_Y, event->y);
			input_event(elo_input_device, EV_ABS,
				    ABS_MT_PRESSURE, event->touch_status);
		}
		// Send primary touch events to move the pointer
		input_mt_report_pointer_emulation(elo_input_device, true);

		// Check if this is the last touch event based on touch count and touch id, then send SYN_REPORT 
		if (event->touch_count == (event->touch_index + 1)) {
			input_sync(elo_input_device);
		}
#else
		// Actions for sending Single Touch as Mouse events for primary touch only 
		if (event->touch_index == 0) {
			if (event->touch_status) {	// If touch is active                  
				input_event(elo_input_device, EV_ABS,
					    ABS_X, event->x);
				input_event(elo_input_device, EV_ABS,
					    ABS_Y, event->y);
				input_event(elo_input_device, EV_KEY,
					    BTN_LEFT, event->touch_status);
			} else {
				input_event(elo_input_device, EV_ABS,
					    ABS_X, event->x);
				input_event(elo_input_device, EV_ABS,
					    ABS_Y, event->y);
				input_event(elo_input_device, EV_KEY,
					    BTN_LEFT, 0);
			}
			input_sync(elo_input_device);
		}
#endif

		ptr += sizeof(*event);
		size -= sizeof(*event);
	}

	mutex_unlock(&elo_mutex);

	rc = count;

      done:
	if (buffer)
		kfree(buffer);

	*ppos += count;
	return rc;
}

static const struct file_operations elo_input_fops = {
	.read = elo_input_read,
	.write = elo_input_write
};

/* This takes an integer value: 0 == disable, 1 == enable */
static ssize_t elo_control_write(struct file *file, const char __user * buf,
			       size_t count, loff_t * ppos)
{
	char kernel_buf[32];
	size_t count_to_copy = count < 31 ? count : 31;
	int enable = 0;

	if (copy_from_user(kernel_buf, buf, count_to_copy))
		return -EFAULT;	

	kernel_buf[count_to_copy] = '\0';

	if (sscanf(kernel_buf, "%d", &enable) == 1) {
		if (enable) {
			int rc = elo_create_device();
			if (rc) {
				printk(KERN_DEBUG "elo_mt_input: unable to create device (%d)\n", rc);
			} else {
				printk(KERN_DEBUG "elo_mt_input: created device\n");
			}
		} else {
			int rc = elo_destroy_device();
			if (rc) {
				printk(KERN_DEBUG "elo_mt_input: unable to destroy device (%d)\n", rc);
			} else {
				printk(KERN_DEBUG "elo_mt_input: destroyed device\n");
			}
		}
	} else {
		printk(KERN_DEBUG "elo_mt_input: unexpected control value %s\n", kernel_buf);
		return -EINVAL;
	}

	return count;
}

static const struct file_operations elo_control_fops = {
	.write = elo_control_write
};

static int elo_create_device(void)
{
	int rc = 0;

	if (elo_input_device)
		return -EEXIST;	/* already done */

	if (!(elo_input_device = input_allocate_device())) {
		printk(KERN_ERR
		       "elo_mt_input: Unable to create input_dev\n");
		rc = -EBUSY;
		goto done;
	}
	// Input device will report ABS events
	set_bit(EV_ABS, elo_input_device->evbit);

#ifdef _MULTITOUCH_KERNEL_
	// Setup for direct devices [touchscreens]
	set_bit(INPUT_PROP_DIRECT, elo_input_device->propbit);
	// Corresponding to HID TIPSWITCH field (Pointer emulation)
	input_set_capability(elo_input_device, EV_KEY, BTN_TOUCH);
	elo_input_device->name = "Elo MultiTouch(MT) Device Input Module";
#else
	// Corresponding to HID TIPSWITCH field - Send LEFT Mouse button event
	input_set_capability(elo_input_device, EV_KEY, BTN_LEFT);
	elo_input_device->name = "Elo Single Touch Device Input Module";
#endif


	// Set appropriate dimensions
	input_set_abs_params(elo_input_device, ABS_X, MIN_X, MAX_X, 0, 0);
	input_set_abs_params(elo_input_device, ABS_Y, MIN_Y, MAX_Y, 0, 0);

#ifdef _MULTITOUCH_KERNEL_
	input_set_abs_params(elo_input_device, ABS_MT_POSITION_X, MIN_X,
			     MAX_X, 0, 0);
	input_set_abs_params(elo_input_device, ABS_MT_POSITION_Y, MIN_Y,
			     MAX_Y, 0, 0);
	input_set_abs_params(elo_input_device, ABS_MT_PRESSURE,
			     MIN_PRESSURE, MAX_PRESSURE, 0, 0);
	input_set_abs_params(elo_input_device, ABS_PRESSURE, MIN_PRESSURE,
			     MAX_PRESSURE, 0, 0);

	// Corresponding to HID contact ID
#if defined(CONFIG_BRIGHTSIGN) || (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0))
	input_mt_init_slots(elo_input_device, ELO_MT_MAXCONTACTS, INPUT_MT_DIRECT);	// Kernel 3.7 or later [Added 3rd parameter - flags] 
#else
	input_mt_init_slots(elo_input_device, ELO_MT_MAXCONTACTS);
#endif
#endif

	if ((rc = input_register_device(elo_input_device))) {
		printk(KERN_ERR
		       "elo_mt_input: Unable to register input_dev\n");
		input_free_device(elo_input_device);
		elo_input_device = NULL;
	}
done:
	return rc;
}

static int elo_destroy_device(void)
{
	int rc = 0;
	mutex_lock(&elo_mutex);
	if (elo_input_device) {
		input_unregister_device(elo_input_device);
		input_free_device(elo_input_device);
		elo_input_device = NULL;
	} else {
		rc = -ENODEV;
	}
	mutex_unlock(&elo_mutex);
	return rc;
}

static int __init elo_input_init(void)
{
	int rc = 0;

	// Create the proc entries
	proc_entry = proc_create("elo_mt_input", S_IWUSR | S_IRUSR, NULL,
			&elo_input_fops);

	if (proc_entry == NULL) {
		printk(KERN_ERR
		       "elo_mt_input: Failed to create /proc/elo_mt_input\n");
		rc = -EBUSY;
		goto done;
	}
	proc_entry = proc_create("elo_mt_control", S_IWUSR, NULL, 
			&elo_control_fops);
	if (proc_entry == NULL) {
		printk(KERN_ERR
		       "elo_mt_input: Failed to create /proc/elo_mt_control\n");
		remove_proc_entry("elo_mt_input", NULL);
		rc = -EBUSY;
		goto done;
	}

	printk(KERN_INFO "elo_mt_input: loaded\n");

done:
	return rc;
}

static void __exit elo_input_exit(void)
{
	remove_proc_entry("elo_mt_input", NULL);
	elo_destroy_device();
}

module_init(elo_input_init);
module_exit(elo_input_exit);


//---------------------------------EOF---------------------------------
