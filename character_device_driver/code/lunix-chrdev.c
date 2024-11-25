/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < pap pap >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */

static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */
	if (sensor->msr_data[state->type]->last_update <= state->buf_timestamp)
        	return 0;
    	else
        	return 1;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	unsigned long state_flags;
	
	long formatted_data, integer, fractional;
	uint32_t raw_data; 

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	
	WARN_ON(!(sensor = state->sensor));

	/* ? */

	spin_lock_irqsave(&sensor->lock, state_flags);


	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */
	/* ? */
	if(!(lunix_chrdev_state_needs_refresh(state))) {
		spin_unlock_irqrestore(&sensor->lock, state_flags);
		return -EAGAIN;
	}

	raw_data = sensor->msr_data[state->type]->values[0]; 
	state->buf_timestamp = sensor->msr_data[state->type]->last_update;
 	spin_unlock_irqrestore(&sensor->lock, state_flags);
	

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */	

	switch (state->type) {
    		case BATT:
        		formatted_data = lookup_voltage[raw_data];
			formatted_data = (formatted_data * 100000) / 3500;
        		integer = formatted_data / 1000; 
        		fractional = formatted_data % 100; 
   		    	state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%02ld%%\n", integer, fractional);
			break;
    		case TEMP:
        		formatted_data = lookup_temperature[raw_data];
        		integer = formatted_data / 1000; 
        		fractional = formatted_data % 1000; 
        		state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%03ld\n", integer, fractional);
        		break;
    		case LIGHT:
        		formatted_data = lookup_light[raw_data];
        		integer = formatted_data / 1000; 
        		fractional = formatted_data % 1000; 
        		state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%03ld\n", integer, fractional);
        		break;
    		default:
        		return -EINVAL;
	}

	/* ? */
	
	debug("leaving\n");
	
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
       	int ret;
	int minor = iminor(inode);

        struct lunix_chrdev_state_struct *state;
	
	debug("entering\n");
	
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;
	
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if(!state) {
		ret = -ENOMEM;
		goto out;
	}

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	state->sensor = &lunix_sensors[minor / 8];
	state->type = minor % 8;
	state->buf_data[LUNIX_CHRDEV_BUFSZ - 1]='\0'; 
	state->buf_lim = strnlen(state->buf_data, LUNIX_CHRDEV_BUFSZ);
	state->buf_timestamp = (uint32_t)(ktime_get_ns()/1000000);
	
	//initialize the semaphore
	sema_init(&state->lock, 1);
	

	/* Allocate a new Lunix character device private state structure */
	/* ? */
	filp->private_data = state;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	struct lunix_chrdev_state_struct *state;
	state = filp->private_data;
	kfree(state);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
	size_t available;
	
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* 
	 * Lock?
	 */
	if(down_interruptible(&state->lock)) 
		return -ERESTARTSYS;
	
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			up(&state->lock);  //unlock before sleeping
		        if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state))) {
		        	return -ERESTARTSYS;
		        }
			//loop but first reacquire the lock
		        if (down_interruptible(&state->lock)){
				return -ERESTARTSYS;
			}
		}
	}
	/* End of file */
	/* ? */
	available = state->buf_lim - *f_pos;
	/*
	if(available < 0) {
		ret = 0;
		goto out;
	}
	*/

	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
	
	cnt = min(cnt, available);
		
	if(copy_to_user(usrbuf, (state->buf_data + *f_pos), cnt)) {
		ret = -EFAULT;
	        up(&state->lock);
		goto out;
	}
	
	*f_pos += cnt; 
	ret = cnt;
	/* Auto-rewind on EOF mode? */
	/* ? */
	if(*f_pos == state->buf_lim) 
		*f_pos = 0;

	debug("Read %li bytes \n", (long)cnt);
	/*
	 * The next two lines  are just meant to suppress a compiler warning
	 * for the "unused" out: label, and for the uninitialized "ret" value.
	 * It's true, this helpcode is a stub, and doesn't use them properly.
	 * Remove them when you've started working on this code.
	 */

out:
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* register_chrdev_region? */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix_chrdev");
	/* Since this code is a stub, exit early */
	//return 0;

	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* ? */
	/* cdev_add? */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
