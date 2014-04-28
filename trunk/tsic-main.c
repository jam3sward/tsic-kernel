/*
  * Experimental Kernel module for a single TSIC 306 temperature sensor
  *
  * Copyright (C) 2014 James Ward
  *
  * Released under the GPL
  *
  */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/irq.h>

/*---------------------------------------------------------------------------*/

/* The Kernel object */
static struct kobject *s_kernelObject = NULL;

/* GPIO pin used for temperature sensor */
static int gpio = 4;
module_param(gpio, int, S_IRUGO);

/* Scale factor for reporting values: this avoids the need for floating
 * point maths inside the kernel module, and is consistent with the scale
 * factor used for other temperature sensors (such as thermal_zone0)
 */
static const int SCALE_FACTOR = 1000;

/* The number of bits expected from the TSIC 306 */
static const int TSIC_BITS = 20;

/* The lower and upper limits of temperature range for the TSIC 306 */
static const int MIN_TEMP = -50;
static const int MAX_TEMP = 150;

/* Represents invalid temperature reading */
static const int INVALID_TEMP = -100000;

/* Watchdog timeout period in ms */
static const int WATCHDOG_TIMEOUT = 110;

/*---------------------------------------------------------------------------*/

/* calculate parity for an eight bit value */
static int parity8( int value )
{
	value = (value ^ (value >> 4)) & 0x0F;
	return (0x6996 >> value) & 1;
}

/*---------------------------------------------------------------------------*/

typedef struct {
	int count;		/* Number of interrupts received */
	int word;		/* The data word, updated by interrupt handler */
	int reset;		/* Request to reset irqCount/irqWord and retry */
} IRQData;

static IRQData irqData;

/*---------------------------------------------------------------------------*/

static irqreturn_t gpioInterruptHandler( int irq, void *dev_id )
{
    int value;

    /* Assuming that each bit frame is 125us duration, and logic 1 is 75%
     * duty, and logic 0 is 25% duty, we sample between 31.25us and 93.75us.
     * Here we use 40us, allowing for some tolerance in case of delay.
     */
    udelay(40);

    /* Sample the data line */
    value = gpio_get_value(gpio) & 1;

	/* Check the cookie */
	if ( dev_id != &irqData ) return IRQ_HANDLED;

    /* We have been asked to start again from bit zero */
    if ( irqData.reset ) {
        irqData.count = 0;
        irqData.word  = 0;
        irqData.reset = 0;
    }

    /* Store the data bit */
    if (irqData.count < TSIC_BITS) irqData.word = (irqData.word << 1) | value;

    /* Count the number of interrupts (bits) handled */
    ++irqData.count;

    return IRQ_HANDLED;
}

/*---------------------------------------------------------------------------*/

/* Read two packets from the device, strips the start bit off each and returns
 * an 18 bit word consisting of two 9 bit values (data byte and even parity
 * bit in the least significant bit). Returns -1 in case of failure.
 */
int readPacket( void )
{
	int word  = 0;  /* the received word, for return to the caller */
    int irq   = 0;  /* the IRQ number */
    int old   = 0;  /* used to store old IRQ counter value */
    int retry = 0;  /* number of retries */

    unsigned long timeout = 0;  /* used to store timeout (end in jiffies) */

    /* Request an IRQ for the sensor GPIO pin, so that we can detect falling
     * edge of each transmitted bit. The interrupt handler will then sample
     * and store each bit.
     */
    irq = gpio_to_irq( gpio );
    irqData.count = 0;
    irqData.word  = 0;
	irqData.reset = 0;
    if ( request_irq(
		irq,
		gpioInterruptHandler,
		IRQF_TRIGGER_FALLING | IRQF_SHARED,
		"tsic_irq",
		&irqData				/* Pointer to data structure */
	) )
        return -1;

    /* Calculate the end time in jiffies when we should time out */
    timeout = jiffies + msecs_to_jiffies(WATCHDOG_TIMEOUT);

    /* Sleep until the interrupt handler has collected all TSIC_BITS, or we
     * have timed out waiting for this to happen.
     */
    while ( (irqData.count < TSIC_BITS) && time_before(jiffies,timeout) ) {
        /* Sleep for 1ms */
        msleep(1);

        /* If we have received at least one bit, and have received no other
         * bits since the previous time step, we must have run off the end
         * of a packet (i.e. were called part way through a packet). In this
         * case, we ask the interrupt handler to reset and try again.
         */
        if (
			(irqData.count >  0)   &&
			(irqData.count == old) &&
			(irqData.reset == 0)
		) {
            /* if we haven't exceeded the maximum number of retries */
            if ( ++retry < 2 ) {
                /* ask the interrupt handler to retry */
                irqData.reset++;
            } else {
                /* give up (this is a precaution against infinite retries) */
                break;
            }
        }

        /* Remember the number of bits for next time step */
        old = irqData.count;
    }

    /* Free the interrupt */
	free_irq( irq, &irqData );

	/* If we didn't receive all the bits, return an error */
	if ( irqData.count < TSIC_BITS ) return -1;

	/* Copy the data word from the interrupt handler */
	word = irqData.word;

	/* contains: <data><parity><start><data><parity> */

	/* strip out start bit in middle */
	word = (word & 0x1FF) | ((word >> 1) & (0x1FF << 9));

    /*printk( KERN_ALERT "tsic: %X\n", word );*/

    /* now contains: <data><parity><data><parity> */

    /* return the result */
	return word;
}

/*---------------------------------------------------------------------------*/

/* Reads the raw temperature value from the sensor */
int readTemperature( void )
{
    int packet0 = 0;
    int packet1 = 0;
    int parity0 = 0;
    int parity1 = 0;
    int valid = 0;
    int raw = 0;
    int temp = 0;

	/* read packets from the sensor: this is time critical */
	raw = readPacket();
	/*printk( KERN_ALERT "tsic: %X\n", raw );*/
	
	if ( raw < 0 ) return INVALID_TEMP;
	
	/* separate into two 9 bit words */
	packet0 = (raw >> 9) & 0x1FF;	
	packet1 = raw & 0x1FF;
	
	/* strip off the parity bit */
	parity0 = packet0 & 1;
	packet0 >>= 1;
	parity1 = packet1 & 1;
	packet1 >>= 1;
	
	/* check the parity on both bytes */
	valid =
		( parity0 == parity8(packet0) ) &&
		( parity1 == parity8(packet1) );
	
	/* if the parity is wrong, return INVALID_TEMP */
	if ( !valid ) {
    	printk( KERN_ALERT "tsic: parity error (%X %03X %03X)\n", raw, packet0, packet1 );
	    return INVALID_TEMP;
	}
	
	/* if any of the top 5 bits of packet 0 are high, that's an error */
	if ( (packet0 & 0xF8) != 0 ) {
	    printk( KERN_ALERT "tsic: prefix error (%03X %03X)\n", packet0, packet1 );
	    return INVALID_TEMP;
	}

	/* this is our raw 11 bit word */
	raw = (packet0 << 8) | packet1;

	/* convert raw integer to temperature in degrees C */
	temp = (MAX_TEMP - MIN_TEMP) * SCALE_FACTOR * raw / 2047 + MIN_TEMP * SCALE_FACTOR;

	/* check that the temperature lies in the measurable range */
	if ( (temp >= MIN_TEMP * SCALE_FACTOR) && (temp <= MAX_TEMP * SCALE_FACTOR) ) {
		/* all looks good */
		/*printk( KERN_ALERT "tsic: looks good (%03X %03X %d)\n", packet0, packet1, temp );*/
		return temp;
	} else {
		/* parity looked good, but the value is out of the valid range */
		printk( KERN_ALERT "tsic: range error (%03X %03X)\n", packet0, packet1 );
		return INVALID_TEMP;
	}
}

/*---------------------------------------------------------------------------*/

/* This function is called when the 'temp' kernel object is read */
static ssize_t temp_show(
    struct kobject *object,
    struct kobj_attribute *attribute,
	char *buffer
) {
    /* read the temperature sensor */
    int temperature = readTemperature();
    
	/* allow one retry in case of failure */
	if ( temperature == INVALID_TEMP ) {
		msleep(2);
		temperature = readTemperature();
	}

    /* output the temperature if valid */
    if ( temperature != INVALID_TEMP )
        return sprintf( buffer, "%d\n", temperature );
    else
        return sprintf( buffer, "error\n" );
}

/*---------------------------------------------------------------------------*/

/* Attribute representing the 'temp' kernel object, which is read only */
static struct kobj_attribute tempAttribute = __ATTR_RO(temp);

/*---------------------------------------------------------------------------*/

/* List of all attributes */
static struct attribute *attrs[] = {
	&tempAttribute.attr,
	NULL    /* terminate the list */
};

/*---------------------------------------------------------------------------*/

/* Attribute group */
static struct attribute_group attributeGroup = {
	.attrs = attrs
};

/*---------------------------------------------------------------------------*/

/* Initialise GPIO */
static int gpioInit( void )
{
    /* check that GPIO is valid */
    if ( !gpio_is_valid(gpio) ){
        printk( KERN_ALERT "GPIO %d is not valid\n", gpio );
        return -EINVAL;
    }

    /* request the GPIO pin */
    if( gpio_request(gpio, "tsic") != 0 ) {
        printk( KERN_ALERT "Unable to request GPIO %d\n", gpio );
        return -EINVAL;
    }

    /* make GPIO an input */
    if( gpio_direction_input(gpio) != 0 ) {
        printk( KERN_ALERT "Failed to make GPIO %d an input\n", gpio );
        return -EINVAL;
    }
    
    return 0;
}

/*---------------------------------------------------------------------------*/

/* Free GPIO */
static void gpioFree( void )
{
    gpio_free(gpio);
}

/*---------------------------------------------------------------------------*/

/* Module initialisation function */
static int __init moduleInit( void )
{
	int result = -1;

	/* Create /sys/kernel/tsic representing the TSIC sensor */
	s_kernelObject = kobject_create_and_add( "tsic", kernel_kobj );
	if ( s_kernelObject == NULL )
		return -ENOMEM;

	/* Create the files associated with this kobject */
	result = sysfs_create_group( s_kernelObject, &attributeGroup );
	if ( result ) {
	    /* Failed: clean up */
		kobject_put( s_kernelObject );
		return result;
	}
	
	/* Set up the GPIO */
	if ( gpioInit() < 0 )
	    return -EINVAL;

	return result;
}

/*---------------------------------------------------------------------------*/

/* Module exit function */
static void __exit moduleExit( void )
{
    /* Decrement refcount and clean up if zero */
	kobject_put( s_kernelObject );
	
	/* Free GPIO */
	gpioFree();
}

/*---------------------------------------------------------------------------*/

module_init(moduleInit);
module_exit(moduleExit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("James Ward");

/*---------------------------------------------------------------------------*/

