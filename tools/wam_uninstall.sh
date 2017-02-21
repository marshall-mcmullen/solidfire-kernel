#!/bin/bash
root=/opt/mvwam
pkg_drv=$root/driver
log_file=$root/uninstall.log

drv_module=mvloki.ko
sys_bin=/usr/bin/
sys_lib=/usr/lib/
arch="x86_64"

echo "WAM uninstall" > $log_file

log_msg()
{
	echo $1;
	echo $1 >> $log_file
}

wam_uninstall_drv()
{
	ker_ver=$1;
	machine=$2;

	echo "Uninstalling driver for version $ker_ver" >> $log_file

	if ! [ -d "/lib/modules/$ker_ver" ]
	then
		echo "Kernel version $ker_ver not installed!. Skipping" >> $log_file
		return;
	fi

	drv_path="/lib/modules/$ker_ver/kernel/drivers/scsi/$drv_module"

	log_msg "rm -f $drv_path"
	rm -f $drv_path

	# Run depmod -a for the given kernel version
	log_msg "depmod -a $ker_ver"
	depmod -a $ker_ver
}

wam_uninstall_all_ver()
{
	for version in `ls $pkg_drv/$arch`
	do
		wam_uninstall_drv $version $arch
	done
}

wam_uninstall_all_ver;

log_msg "rm -f /etc/rc.d/init.d/wam_startup"
rm -f /etc/rc.d/init.d/wam_startup

for i in 1 2 3 5; do
	log_msg "cd /etc/rc${i}.d"
	cd /etc/rc${i}.d

	log_msg "rm -f S80wam_startup"
	rm -f S80wam_startup

	cd - >> /dev/null
done


# rm the binary to tool path
log_msg "rm -f $sys_bin/wamdiag"
rm -f $sys_bin/wamdiag

log_msg "rm -f $sys_bin/wamctl"
rm -f $sys_bin/wamctl

# rm the library to lib path
log_msg "rm -f $sys_lib/libmvwam.so"
rm -f $sys_lib/libmvwam.so
