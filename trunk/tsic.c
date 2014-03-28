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
#include <linux/time.h>
#include <linux/delay.h>

/*---------------------------------------------------------------------------*/

/* The Kernel object */
static struct kobject *s_kernelObject = NULL;

/* GPIO pin used for temperature sensor */
static const int tempGPIO = 4;

/* Represents invalid temperature reading */
static const int INVALID_TEMP = -100000;

/*---------------------------------------------------------------------------*/

/* calculate parity for an eight bit value */
static int parity8( int value )
{
	value = (value ^ (value >> 4)) & 0x0F;
	return (0x6996 >> value) & 1;
}

/*---------------------------------------------------------------------------*/

/* Read two packets from the device, strips the start bit off each and returns
 * an 18 bit word consisting of two 9 bit values (data byte and even parity
 * bit in the least significant bit)
 */
int readPacket( void )
{
	int word  = 0;  /* the received word, for return to the caller */
	int i     = 0;  /* loop variable */
	int low   = 0;  /* time spent low within one frame (bit) */
	int high  = 0;  /* time spent high within one frame (bit) */
	int half  = 0;  /* time for half one frame */
	int frame = 0;  /* time for one frame */
	int retry = 0;  /* count number of retries */

    /* There's a transmission lasting about 2.6ms every 100ms, and we could
     * be anywhere in that cycle when we enter this function. There's a
     * small chance (about 3%) that we could be inside a packet already.
     * We try to detect the situation and retry if needed.
     */
    for (;;) {
	    /* wait for the line to go low */
	    while ( gpio_get_value(tempGPIO) ) ++high;
	
	    /* time the low portion of the start bit, which is half the duty cycle */
	    while ( !gpio_get_value(tempGPIO) ) ++half;
	    	
	    /* duration of a complete frame (usually around 125us) */
	    frame = 2*half;
	
	    /* if the line was high for less than two frames, then it looks as if
	     * we started sampling part way through a packet (in fact, this very
	     * rarely happens in practice)
	     */
	    if ( high < 2*frame ) {
	        /* have we exceeded the maximum number of retries? */
	        if ( ++retry > 1 ) return -1;
	        
	        /* updates are every 100ms and last around 2.6ms, we want to delay
	         * until just before the next update. this could sleep for longer.
	         */
	        msleep( 95 );
	    } else {
	        /* this looks like the start of a good packet */
	        break;
	    }
    }

    /* read the remaining 19 bits */
	for (i=0; i<19; ++i) {
	    low  = 0;
	    high = 0;
	    
	    /* wait for the line to go low */
		while ( gpio_get_value(tempGPIO) ) ++high;	

	    /* time how long the line is low */
		while ( !gpio_get_value(tempGPIO) ) ++low;
		
		/* if the line is low for less than half the duty cycle, this must
		 * be a high bit, which we store. because we only examine the
		 * low pulses, this will ignore the stop bit between the two packets
		 */	
		if ( low < half ) word |= 1<<(18-i);
		
		/* if the line is high for more than 2 frames, something is wrong
		 * note: it could legitimately be high for up to 1.75 frames if the
	     * first packet ends with high parity (75% duty) followed by the 100%
	     * duty stop bit between the two packets
	     */
		if ( high > 2*frame ) return -1;
	}
	
	/* contains: <data><parity><start><data><parity> */
	
	/* strip out start bit in middle */
	word = (word & 0x1FF) | ((word >> 1) & (0x1FF << 9));

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
    
	/* scale factor for reporting values: this avoids the need for floating
	 * point maths inside the kernel module, and is consistent with the scale
	 * factor used for other temperature sensors (such as thermal_zone0)
	 */
	const int scale = 1000;
	
	/* lower and upper limits of temperature range */
	const int minTemp = -50 * scale;
	const int maxTemp = 150 * scale;

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
    	/*printk( KERN_ALERT "tsic: parity error (%03X %03X)\n", packet0, packet1 );*/
	    return INVALID_TEMP;
	}
	
	/* if any of the top 5 bits of packet 0 are high, that's an error */
	if ( (packet0 & 0xF8) != 0 ) {
	    /*printk( KERN_ALERT "tsic: prefix error (%03X %03X)\n", packet0, packet1 );*/
	    return INVALID_TEMP;
	}

	/* this is our raw 11 bit word */
	raw = (packet0 << 8) | packet1;

	/* convert raw integer to temperature in degrees C */
	temp = (maxTemp - minTemp) * raw / 2047 + minTemp;

	/* check that the temperature lies in the measurable range */
	if ( (temp >= minTemp) && (temp <= maxTemp) ) {
		/* all looks good */
		/*printk( KERN_ALERT "tsic: looks good (%03X %03X %d)\n", packet0, packet1, temp );*/
		return temp;
	} else {
		/* parity looked good, but the value is out of the valid range */
		/*printk( KERN_ALERT "tsic: range error (%03X %03X)\n", packet0, packet1 );*/
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
    
    /* if that failed, have one more attempt */
    if ( temperature == INVALID_TEMP )
        temperature = readTemperature();
    
    /* output the temperature if valid */
    if ( temperature != INVALID_TEMP )
        return sprintf( buffer, "%d\n", temperature );
    else
        return sprintf( buffer, "error\n" );
}

/*---------------------------------------------------------------------------*/

/* This function is called when the 'temp' kernel object is written */
static ssize_t temp_store(
    struct kobject *object,
    struct kobj_attribute *attribute,
    const char *buffer, size_t count
) {
	return count;
}

/*---------------------------------------------------------------------------*/

/* Attribute representing the 'temp' kernel object, which is read only */
static struct kobj_attribute tempAttribute =
    __ATTR(temp, 0666, temp_show, temp_store);
    
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
    if ( !gpio_is_valid(tempGPIO) ){
        printk( KERN_ALERT "GPIO %d is not valid\n", tempGPIO );
        return -EINVAL;
    }

    /* request the GPIO pin */
    if( gpio_request(tempGPIO, "tsic") != 0 ) {
        printk( KERN_ALERT "Unable to request GPIO %d\n", tempGPIO );
        return -EINVAL;
    }

    /* make GPIO an input */
    if( gpio_direction_input(tempGPIO) != 0 ) {
        printk( KERN_ALERT "Failed to make GPIO %d an input\n", tempGPIO );
        return -EINVAL;
    }
    
    return 0;
}

/*---------------------------------------------------------------------------*/

/* Free GPIO */
static void gpioFree( void )
{
    gpio_free(tempGPIO);
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

