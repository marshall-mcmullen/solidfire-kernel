#! /bin/bash

K_VER=`uname -r`
OLD_PATH="/lib/modules/$K_VER/kernel/drivers/block"
NEW_PATH="/lib/modules/$K_VER/kernel/drivers/nvme/host"
ORG_DRV="nvme.ko"
BAK_DRV="nvme.ko.org"
ORG_DRV_CMP="nvme.ko.xz"
BAK_DRV_CMP="nvme.ko.xz.org"

if [ -d "$NEW_PATH" ]; then
# Preserve the original driver
    if [[ -e "$NEW_PATH/$ORG_DRV" &&
	  ! -e "$NEW_PATH/$BAK_DRV" ]]; then
	mv "$NEW_PATH/$ORG_DRV" "$NEW_PATH/$BAK_DRV"
	cp "$ORG_DRV" "$NEW_PATH/$ORG_DRV"
	echo "Original driver was saved as $BAK_DRV"
    else
	if [[ -e "$NEW_PATH/$ORG_DRV_CMP" &&
	      ! -e "$NEW_PATH/$BAK_DRV_CMP" ]]; then
		mv "$NEW_PATH/$ORG_DRV_CMP" "$NEW_PATH/$BAK_DRV_CMP"
		cp "$ORG_DRV" "$NEW_PATH/$ORG_DRV"
		echo "Original driver was saved as $BAK_DRV_CMP"
	else
	    cp "$ORG_DRV" "$NEW_PATH/$ORG_DRV"
	    echo "Driver was installed as $ORG_DRV"
	fi
    fi
else
    cp "$ORG_DRV" "$OLD_PATH/$ORG_DRV"
    echo "Driver was installed as $ORG_DRV"
fi
cd "/lib/modules/$K_VER/"
depmod
echo '*******************************************'
echo 'If you use initrd and include nvme.ko in it,'
echo 'be sure to update the driver there'
echo '*******************************************'
