#!/bin/sh

dir=`dirname $0`
module="mvloki"
device="mvloki"
cdevice="mvwam"
blkdrv="nvdisk"
mode="664"
#args="intr_coalesce_count=16"

if [ ! -f "$dir/$module.ko" ]
then
	echo "Driver file $module.ko missing. Please build the driver"
	exit 1
fi

# remove previous version
/sbin/rmmod $module 2> /dev/null

# install driver module
/sbin/insmod $dir/$module.ko $args $* || exit 1

# get new major number
cdev_major=$(grep $device /proc/devices | awk '{print $1}')
bdev_major=$(grep $blkdrv /proc/devices | awk '{print $1}')

if [ -z $cdev_major ]
then
    echo "Driver load failed"
    exit 1
fi

#echo cdev_major is $cdev_major
nr_hbas=`lspci | grep Marvell | grep 8180 | wc -l`
nr_hbas=`expr $nr_hbas - 1`
for i in `seq 0 $nr_hbas`; do
	# remove stale device node
	cdev_file="/dev/${cdevice}$i"
	bdev_file="/dev/${blkdrv}$i"
	rm -f $cdev_file
	rm -f $bdev_file

	echo "Creating device file $cdev_file"
	mknod $cdev_file c $cdev_major $i

	blk_minor=`expr $i \* 16`
	echo "Creating blk device file $bdev_file"
	mknod $bdev_file b $bdev_major $blk_minor

	chmod $mode $cdev_file
	chmod $mode $bdev_file

#	Not necessary to modify group info
#	group="staff"
#	grep -q '^staff:' /etc/group || group="wheel"
#	chgrp $group $dev_file
done

echo "Marvell NVRAM driver ${device} loaded at major: ${cdev_major}"

