#!/bin/bash
root=`dirname $0`
drv_module=mvloki.ko
arch="x86_64"

sys_bin=/usr/bin/
sys_lib=/usr/lib/

base=`basename $PWD`
package_dir=/opt/mvwam
pkg_drv=$package_dir/driver
pkg_bin=$package_dir/bin
pkg_lib=$package_dir/lib
log_file=$package_dir/install.log


log_msg()
{
	echo $1;
	echo $1 >> $log_file
}

wam_install_drv()
{
	ker_ver=$1;
	machine=$2;

	echo "Installing driver for version $ker_ver" >> $log_file

	if ! [ -d "/lib/modules/$ker_ver" ]
	then
		echo "Kernel version $ker_ver not installed!. Skipping" >> $log_file
		return;
	fi

	src_path="$pkg_drv/$machine/$ker_ver/$drv_module"
	dst_path="/lib/modules/$ker_ver/kernel/drivers/scsi/"
	echo "cp $src_path $dst_path" >> $log_file

	# copy the driver
	cp -f $src_path $dst_path

	# Run depmod -a for the given kernel version
	depmod -a $ker_ver
}

wam_install_all_ver()
{
	for version in `ls $pkg_drv/$arch`
	do
		wam_install_drv $version $arch
	done
}

wam_update_bin()
{
	log_msg "Updating driver/tools binaries"

	echo "cp $pkg_bin/wamdiag $sys_bin" >> $log_file
	echo "cp $pkg_bin/wamctl $sys_bin" >> $log_file
	echo "cp $pkg_lib/libmvwam.so $sys_lib" >> $log_file
	echo "cp $package_dir/wam_startup /etc/rc.d/init.d/wam_startup" >> \
		$log_file

	# copy the binary to tool path
	cp $pkg_bin/wamdiag $sys_bin
	cp $pkg_bin/wamctl $sys_bin

	# copy the library to lib path
	cp $pkg_lib/libmvwam.so $sys_lib

	# add wam_startup to /etc/rcS.d/
	cp $package_dir/wam_startup /etc/rc.d/init.d/wam_startup

	for i in 1 2 3 5; do
		cd /etc/rc${i}.d
		rm -f S80wam_startup
		ln -s ../init.d/wam_startup S80wam_startup
		cd - >> /dev/null
	done
}

wam_load_drv()
{
	log_msg "Loading driver"
	$package_dir/wam_startup
}


log_msg "Upgrading package"
echo "Upgrading package" > $package_dir/.pkg_upgrade

wam_update_bin
wam_install_all_ver

drv_loaded=`lsmod | grep -c mvloki`
if [ $drv_loaded -eq 1 ]; then
	# ask admin to reboot
	log_msg "Please reboot the system for the driver update to take effect"
else
	# load driver
	wam_load_drv
fi

rm -f $package_dir/.pkg_upgrade
echo "Marvell WAM Installation complete"
