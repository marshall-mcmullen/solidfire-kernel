#!/bin/bash

dev_string="11ab:8180"
dev_loki="8180"
driver_string=mvloki
dev_cnt=0
dev_idx=
log_dir=
WAMDIAG=wamdiag
WAMCTL=wamctl

#For standalone driver mode
dir=`dirname $0`
export PATH=$PATH:$dir

wam_check_tool()
{
	found=`ls -l | grep -c wamdiag`

	if [ $found -eq 0 ]; then
		echo "Please run under tools/ dictory of the driver package."
		exit 1
	fi

	found=`ls -l | grep -c wamctl`
	if [ $found -eq 0 ]; then
		echo "Please run under tools/ dictory of the driver package."
		exit 1
	fi
}

wam_check_dev()
{
	dev_cnt=`lspci -d "$dev_string" | wc -l`
	if [ $dev_cnt -eq 0 ]; then
		echo "No WAM device found."
		exit 1
	fi

	dev_idx=`expr $dev_cnt - 1`

	echo "Found $dev_cnt WAM device(s)."
}

wam_check_driver()
{
	found=`lsmod | grep -c "$driver_string"`

	if [ $found -eq 0 ]; then
		echo "WAM driver not loaded."
		exit 1
	fi
}

wam_create_log_dir()
{
        ts=`date +%y%m%d_%H%M%S`
	log_dir="./`hostname`_$ts"
	echo "Creating log directory: $log_dir"
	mkdir $log_dir
}

wam_get_dmesg()
{
	dmesg > $log_dir/dmesg.out
	cp /var/log/dmesg* $log_dir/ > /dev/null 2>&1
}

wam_get_var_log_msg()
{
	cp /var/log/messages* $log_dir/ > /dev/null 2>&1
}

wam_collect_info()
{
	sys_info=$log_dir/sys_info.out

	# Dump Linux kernel version
	uname -a >> $sys_info
	uptime >> $sys_info
	echo "-------------------------------------------------------" \
		>> $sys_info

	# Dump detailed device info including pci config space
	lspci -vvv -xxx -d "$dev_string" >> $sys_info
	echo "-------------------------------------------------------" \
		>> $sys_info

	# Dump all device information including system chipset info
	lspci -vvv >> $sys_info
	echo "-------------------------------------------------------" \
		>> $sys_info

	# Dump CPU info
	cat /proc/cpuinfo >> $sys_info
	echo "-------------------------------------------------------" \
		>> $sys_info

	# Dump system memory info
	cat /proc/meminfo >> $sys_info
	echo "-------------------------------------------------------" \
		>> $sys_info

	# List all wam device nodes
	ls -l /dev/mv* >> $sys_info
	echo "-------------------------------------------------------" \
		>> $sys_info
	ls -l /dev/nvdisk* >> $sys_info
	echo "-------------------------------------------------------" \
		>> $sys_info

	# Disabling fdisk as it may cause the script to hang if IOs hang
	# List all disk information
	# fdisk -l >> $sys_info 2>&1
	# echo "-------------------------------------------------------" \
	#	>> $sys_info

	# List filesytem info
	df -kh >> $sys_info
	
	echo "-------------------------------------------------------" \
		>> $sys_info

	# Dump interrupt info
	grep $driver_string /proc/interrupts >> $sys_info
}

wam_collect_dev_info()
{
	for i in `seq 0 $dev_idx`; do
		wam_log_dir=$log_dir/wam$i
		wam_dev_info=$wam_log_dir/info.out
		mkdir $wam_log_dir
		dev="/dev/mvwam$i"
		$WAMCTL $dev info v >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMCTL $dev events clear >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMDIAG $dev d >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMCTL $dev list_errors >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMDIAG $dev t >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMDIAG $dev x >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMDIAG $dev a >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMCTL $dev tlog >> $wam_dev_info 2>&1
		echo "------------------------------------------------------" \
			>> $wam_dev_info
		$WAMCTL $dev regdump > $wam_log_dir/regdump.out 2>&1
		$WAMCTL $dev nvsdump > $wam_log_dir/nvsdump.bin 2>&1
	done
}

wam_zip_log_dir()
{
	echo "Creating zip file: $log_dir.tar.gz"
	tar czf $log_dir.tar.gz $log_dir
	rm -rf $log_dir
}

#wam_check_tool
wam_check_dev
wam_check_driver
wam_create_log_dir
wam_get_dmesg
wam_get_var_log_msg
wam_collect_info
wam_collect_dev_info
wam_zip_log_dir
