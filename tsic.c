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

/* Has the GPIO been successfully initialised? */
static int s_initGPIO = 0;

/*---------------------------------------------------------------------------*/

/* calculate parity for an eight bit value */
static int parity8( int value )
{
	value = (value ^ (value >> 4)) & 0x0F;
	return (0x6996 >> value) & 1;
}

/*---------------------------------------------------------------------------*/

/* Reads a 10 bit packet: 1 start bit, 8 data bits and 1 parity bit.
 * Only 9 bits are returned, consisting of the data bits and even parity bit
 * in bit 0 (the start bit is omitted).
 */
int readPacket( void )
{
	int word = 0;
	int i = 0;
	int low = 0;
	int half = 0;
	
	/* wait for the line to go low */
	while ( gpio_get_value(tempGPIO) );
	
	/* time the low portion of the start bit, which is half the duty cycle */
	while ( !gpio_get_value(tempGPIO) ) ++half;

    /* read the remaining 9 bits: data and parity */
	for (i=0; i<9; ++i) {
	    low = 0;
	    
	    /* wait for the line to go low */
		while ( gpio_get_value(tempGPIO) );	

	    /* time how long the line is low */
		while ( !gpio_get_value(tempGPIO) ) ++low;
		
		/* if the line is low for less than half the duty cycle, this must
		 * be a high bit, which we store
		 */
		word <<= 1;		
		if ( low < half ) word |= 1;
	}

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

	/* read two packets: this is time critical */
	packet0 = readPacket();
	packet1 = readPacket();
	
	/* printk( KERN_ALERT "tsic %X %X\n", packet0, packet1 ); */

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
	if ( !valid ) return INVALID_TEMP;

	/* this is our raw 8 bit word */
	raw = (packet0 << 8) | packet1;

	/* convert raw integer to temperature in degrees C */
	temp = (maxTemp - minTemp) * raw / 2047 + minTemp;

	/* check that the temperature lies in the measurable range */
	if ( (temp >= minTemp) && (temp <= maxTemp) ) {
		/* all looks good */
		return temp;
	} else {
		/* parity looked good, but the value is out of the valid range */
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
    if ( s_initGPIO ) {
        /* read the temperature sensor */
        int temperature = readTemperature();
        
        /* output the temperature if valid */
        if ( temperature != INVALID_TEMP )
            return sprintf( buffer, "%d\n", temperature );
        else
            return 0;
    } else
        return 0;
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
    
    /* GPIO is initialised */
    s_initGPIO = 1;
    
    return 0;
}

/*---------------------------------------------------------------------------*/

/* Free GPIO */
static void gpioFree( void )
{
    gpio_free(tempGPIO);
    s_initGPIO = 0;
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

