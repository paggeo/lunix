/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Your name here >
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
 * Returns the data that needs to be refreshed, or -1 if no
 * refresh needed.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON( !(sensor = state->sensor));
	
	/* Compare buf_timestamp of the state with the last_update of the sensor */
	if(state->buf_timestamp < sensor->msr_data[state->type]->last_update){
		return sensor->msr_data[state->type]->values[0];
	}

	/* The timestamps are equal so no update required */
	return -1;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t data;
	long value = 0;

	debug("leaving\n");
	
	sensor = state->sensor;

	/* Why use spinlocks? See LDD3, p. 119 */
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	spin_lock_irq(&sensor->lock);

	/*
	 * Any new data available?
	 */
	data = lunix_chrdev_state_needs_refresh(state);

	/* Fix buf_timestamp without interrupts */
	state->buf_timestamp = sensor->msr_data[state->type]->last_update;

	/* Got the data so unlock*/
	spin_unlock_irq(&sensor->lock);

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	down_interruptible(&state->lock);
	
	if(data == -1) {
		return -EAGAIN;
	}
	
	if(state->type == BATT) value = lookup_voltage[data];
	else if(state->type == TEMP) value = lookup_temperature[data];
	else if(state->type == LIGHT) value = lookup_light[data];

	/* FORMAT DATA INTO buf_data */
	sprintf(state->buf_data, "%ld.%ld", value/1000, value%1000);
	
	/* CHANGE buf_lim ???*/
	
	up(&state->lock);

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
	unsigned int minor, type, sensor_index;
	struct lunix_chrdev_state_struct *chrdev_state;
	struct lunix_sensor_struct *sensor;
	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	minor = iminor(inode);	 
	type = minor % 8;	// The type of sensor (BATT, TEMP, LIGHT)
	sensor_index = minor / 8;	// The index of the sensor in lunix_sensors
	sensor = &lunix_sensors[sensor_index];	// Find correct sensor in array

	/* Allocate a new Lunix character device private state structure */
	chrdev_state = kmalloc(sizeof(*chrdev_state), GFP_KERNEL);
	if(!chrdev_state) {
		debug("kmalloc failed\n");
		ret = -ENOMEM;
		goto out;
	}

	/*Initialize everything in lunix_chrdev_state_struct except buf_data*/
	chrdev_state->type = /*(lunix_msr_enum)*/ type;
	chrdev_state->sensor = sensor;
	chrdev_state->buf_lim = 0;
	chrdev_state->buf_timestamp = 0;
	sema_init(&chrdev_state->lock, 1);

	filp->private_data = chrdev_state;		// private_data now holds a pointer to the state
	
	ret = 0;
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/*Doesn't check whether it is NULL but kfree will work regardless*/
	kfree(filp->private_data);
	debug("released sensor\n");
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock */
	down_interruptible(&state->lock);

	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* Unlock */
			up(&state->lock);

			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			if(wait_event_interruptible(sensor->wq, (lunix_chrdev_state_update(state) != -EAGAIN))){
				return -ERESTARTSYS;
			}

			/* 
			 * Process has been awaken.
			 * Lock again for next loop 
			 */
			down_interruptible(&state->lock);
		}
	}

	/* End of file */
	if(*f_pos >= state->buf_lim) {
		ret = -EFAULT;
		goto out;
	}

	/* Determine the number of cached bytes to copy to userspace */
	if(*f_pos + cnt >= state->buf_lim){
		cnt = state->buf_lim - *f_pos;
		if(copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)){
			ret = -EFAULT;
			goto out;
		}
		filp->f_pos = state->buf_lim;
		ret = cnt;
	}

	else {
		if(copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)){
			ret = -EFAULT;
			goto out;
		}
		filp->f_pos = *f_pos + cnt;
		ret = cnt;
	}

	/* Auto-rewind on EOF mode? */
	if(*f_pos == state->buf_lim) filp->f_pos = 0;

out:
	/* Unlock */
	up(&state->lock);

	/* Wake up the rest of the processes (this happens in lunix_sensor_update) */
	//wake_up(sensor->wq);

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
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "Lunix:TNG");
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
