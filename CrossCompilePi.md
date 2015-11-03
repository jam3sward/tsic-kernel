# Cross-compiling the Kernel Module for Raspberry Pi #

This is a very brief summary of how to cross-compile the kernel module on Ubuntu, for use on the Raspberry Pi (Raspbian).

From your Raspberry Pi, take a copy of /proc/config.gz and copy it to Ubuntu. Unzip it using gunzip.

Set up the tools as described here: http://elinux.org/RPi_Kernel_Compilation

Then download the Raspberry Pi source, clean it and copy in your Raspberry Pi config file:
```
git clone --depth 1 git://github.com/raspberrypi/linux.git
cd linux
make mrproper
cp your/raspi/config .config
```

Now we need the correct version of the Module.symvers file. From the Pi, find out the kernel version you are using:
```
uname -r
```

Then take a look here to find the corresponding version of Module.symvers:
https://github.com/raspberrypi/firmware/commits/master/extra/Module.symvers

Browse to the raw file, save the URL and use wget to download it into your linux source folder (there is no doubt an easier way).

Now we can make it:
```
ARCH=arm CROSS_COMPILE=${CCPREFIX} make oldconfig
ARCH=arm CROSS_COMPILE=${CCPREFIX} make menuconfig
# Save.. Exit..
ARCH=arm CROSS_COMPILE=${CCPREFIX} make modules_prepare
```

Having done this, you have a minimal source tree which allows you to build the module.