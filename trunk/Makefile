obj-m += tsic.o

all:
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ../linux M=$(PWD) modules

clean:
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ../linux M=$(PWD) clean
