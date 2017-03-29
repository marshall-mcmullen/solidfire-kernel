/*****************************************************************************
 *
 *  Copyright (C) 2017  NetApp / SolidFire Inc.
 *  
 *  Derived from Copyright (C) 2009-2013  Integrated Device Technology, Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#ifndef __RADIAN_NVME_EXPRESS_H__
#define __RADIAN_NVME_EXPRESS_H__

/**
 * @brief NVM Express Log Page - Error Informaton Log Entry data structure.
 */
struct error_log
{

    union
    {
        struct
        {
            /**
             * @brief 64-bit incrementing error count, indicating a unique
	     *		identifier for this error.
	     *		The error count starts at 1h, is incremented for each
	     *		unique error log entry, and is retained across power
	     *		off conditions. A value of 0h indicates an invalid
	     *		entry; this value may be used when there are lost
	     *		entries or when there a fewer errors than the maximum
	     *		number of entries the controller supports.
             */
	    __u64 errorCount;
            /**
             * @brief The Submission Queue Identifier of the command
	     *		that the error information is associated with.
             */
	    __u16 sqID;
            /**
             * @brief This field indicates the Command Identifier of the
	     *		command that the error is assocated with.
             */
	    __u16 cmdID;
            /**
             * @brief The Status Code that the command completed with.
             */
	    __u16 status;
            /**
             * @brief Byte in command that contained the error.
	     *		Valid values are 0 to 63.
             */
	    __u16 errorByte:8,
            /**
             * @brief Bit in command that contained the error.
	     *		Valid values are 0 to7.
             */
	    	  errorBit:3,
            /**
             * @brief reserved.
             */
	     	  reservedA:5;
            /**
             * @brief First LBA that experienced the error condition, if
	     *		applicable.
             */
	    __u64 lba;
            /**
             * @brief The namespace that the error is associated with, if
	     *		applicable.
             */
	    __u32 nameSpace;
            /**
             * @brief If there is additional vendor specific error information
	     *		available, this field provides the log page identifier
	     *		associated with that page. A value of 00h indicates
	     *		that no additional information is available.
	     *		Valid values are in the range of 80h to FFh.
             */
	    __u8 vendorInfo;
            /**
             * @brief Reserved;
             */
    	    __u8 reservedB[63-29+1];
	};
        /**
         * @brief byte host memory buffer address.
         */
	__u32   asUlong[16];
    };
};

/**
 * @brief NVM Express Log Page - SMART/HEALTH Informaton Log data structure.
 */
struct smart_log
{
    union
    {
        struct __attribute__((__packed__))
        {
#if 0
            /**
             * @brief This field indicates critical warnings for the state
	     * of the controller. Each bit corresponds to a critical warning
	     * type; multiple bits may be set. If a bit is cleared to .0.,
	     * then that critical warning does not apply. Critical warnings
	     * may result in an asynchronous event notification to the host.
             */
	    __u8 criticalError;
            /**
             * @brief Contains the temperature of the overall device
	     * (controller and NVM included) in units of Kelvin. If the
	     * temperature exceeds the temperature threshold, refer to section
	     * 5.12.1.4, then an asynchronous event may be issued to the host.
             */
	    __u16 temperature;
            /**
             * @brief Contains a normalized percentage (0 to 100%) of the
	     * remaining spare capacity available.
             */
	    __u8 availableSpace;
#else
	    /* Jimmy hacking: GCC is ignoring __attribute__(packed), so
             * I put the following three members into a 32-bit bitfield
             * in order to match the size and alignment of the smart log page
             * in the Princeton.
             */
	    __u32 criticalError:8;
	    __u32 temperature:16;
	    __u32 availableSpace:8;
#endif
            /**
             * @brief When the Available Spare falls below the threshold
	     * indicated in this field, an asynchronous event may be issued
	     * to the host. The value is indicated as a normalized percentage
	     * (0 to 100%).
             */
	    __u8 availableSpaceThreshold;
            /**
             * @brief Contains a vendor specific estimate of the percentage
	     * of device life used based on the actual device usage and the
	     * manufacturer.s prediction of device life. A value of 100
	     * indicates that the estimated endurance of the device has been
	     * consumed, but may not indicate a device failure. The value is
	     * allowed to exceed 100. Percentages greater than 254 shall be
	     * represented as 255. This value shall be updated once per
	     * power-on hour (when the controller is not in a sleep state).
             */
	    __u8 precentageUsed;
            /**
             * @brief Reserved.
	     */
    	    __u8 reservedA[31-6+1];
            /**
             * @brief Contains the number of 512 byte data units the host has
	     * read from the controller; this value does not include metadata.
	     * This value is reported in thousands (i.e., a value of 1
	     * corresponds to 1000 units of 512 bytes read) and is rounded up.
	     * When the LBA size is a value other than 512 bytes, the
	     * controller shall convert the amount of data read to 512 byte
	     * units.
             */
	    __u8 dataUnitsRead[47-32+1];
            /**
             * @brief Contains the number of 512 byte data units the host has
	     * written to the controller; this value does not include metadata.
	     * This value is reported in thousands (i.e., a value of 1
	     * corresponds to 1000 units of 512 bytes written) and is rounded
	     * up. When the LBA size is a value other than 512 bytes, the
	     * controller shall convert the amount of data written to 512 byte
	     * units.
             */
	    __u8 dataUnitsWritten[63-48+1];
            /**
             * @brief Contains the number of read commands issued to the
	     * controller.
             */
	    __u8 hostReadCommands[79-64+1];
            /**
             * @brief Contains the number of write commands issued to the
	     * controller.
             */
	    __u8 hostWriteCommands[95-80+1];
            /**
             * @brief Contains the amount of time the controller is busy with
	     * I/O commands. The controller is busy when there is a command
	     * outstanding to an I/O Queue (specifically, a command was issued
	     * via an I/O Submission Queue Tail doorbell write and the
	     * corresponding completion queue entry has not been posted yet to
	     * the associated I/O Completion Queue). This value is reported
	     * in minutes.
             */
    	    __u8 controllerBusyTime[111-96+1];
            /**
             * @brief Contains the number of power cycles.
             */
    	    __u8 powerCycles[127-112+1];
            /**
             * @brief Contains the number of power-on hours. This does not
	     * include time that the controller was powered and in a low
	     * power state condition.
             */
    	    __u8 powerOnHours[143-128+1];
            /**
             * @brief Contains the number of unsafe shutdowns. This count is
	     * incremented when a shutdown notification (CC.SHN) is not
	     * received prior to loss of power.
             */
    	    __u8 unsafeShutdowns[159-144+1];
            /**
             * @brief Contains the number of occurrences where the controller
	     * detected an unrecovered data integrity error. Errors such as
	     * uncorrectable ECC, CRC checksum failure, or LBA tag mismatch
	     * are included in this field.
             */
    	    __u8 mediaErrors[175-160+1];
            /**
             * @brief Contains the number of Error Information log entries
	     * over the life of the controller.
             */
    	    __u8 numberOfErrorInfoLogs[191-176+1];
            /**
             * @brief reserved.
             */
    	    __u8 reservedB[511-192+1];
	};
        /**
         * @brief Byte host memory buffer address.
         */
	__u8   asByte[512];
    };
};

/**
 * @brief NVM Express Log Page - Firmware Slot Information Log data structure.
 */
struct firmware_slot_log
{
    union
    {
        struct
        {
            /**
             * @brief This log page is used to describe the firmware revision
	     * stored in each firmware slot supported. The firmware revision
	     * is indicated as an ASCII string. The log page also indicates
	     * the active slot number.
             */
	    __u8 activeFirmwareInfo;
            /**
             * @brief Reserved.
             */
	    __u8 reservedA[7-1+1];
            /**
             * @brief Contains the revision of the firmware downloaded to
	     * firmware slot 1. If no valid firmware revision is present or
	     * if this slot is unsupported, all zeros shall be returned.
             */
	    __u8 FirmwareRevisionSlot1[15-8+1];
            /**
             * @brief Contains the revision of the firmware downloaded to
	     * firmware slot 2. If no valid firmware revision is present or
	     * if this slot is unsupported, all zeros shall be returned.
             */
	    __u8 FirmwareRevisionSlot2[23-16+1];
            /**
             * @brief Contains the revision of the firmware downloaded to
	     * firmware slot 3. If no valid firmware revision is present or
	     * if this slot is unsupported, all zeros shall be returned.
             */
	    __u8 FirmwareRevisionSlot3[31-24+1];
            /**
             * @brief Contains the revision of the firmware downloaded to
	     * firmware slot 4. If no valid firmware revision is present or
	     * if this slot is unsupported, all zeros shall be returned.
             */
	    __u8 FirmwareRevisionSlot4[39-32+1];
            /**
             * @brief Contains the revision of the firmware downloaded to
	     * firmware slot 5. If no valid firmware revision is present or
	     * if this slot is unsupported, all zeros shall be returned.
             */
	    __u8 FirmwareRevisionSlot5[47-40+1];
            /**
             * @brief Contains the revision of the firmware downloaded to
	     * firmware slot 6. If no valid firmware revision is present or
	     * if this slot is unsupported, all zeros shall be returned.
             */
	    __u8 FirmwareRevisionSlot6[55-48+1];
            /**
             * @brief Contains the revision of the firmware downloaded to
	     * firmware slot 7. If no valid firmware revision is present or
	     * if this slot is unsupported, all zeros shall be returned.
             */
	    __u8 FirmwareRevisionSlot7[63-56+1];
            /**
             * @brief Reserved.
	     */
    	    __u8 reservedB[511-64+1];
	};
        /**
         * @brief byte host memory buffer address.
         */
	__u8   asByte[512];
    };
};

/**
 * @brief NVM Express PRP entry data structure.
 */

struct nvme_prp
{
    union
    {
        struct
        {
            /**
             * @brief Lower 32-bits of the 64-bit PRP address.
             */
            __u32 lower;

            /**
             * @brief Upper 32-bits of the 64-bit PRP address.
             */
            __u32 upper;
        };
        /**
         * @brief 64-bit host memory buffer address.
         */
        __u64 addr;
    };
};


/**
 * @brief NVM Express command header data structure.
 */
struct nvme_cmd_hdr
{
    /**
     * @brief Opcode.
     *
     * This field indicates the opcode of the command to be executed.
     */
    __u32 opCode:8,
    /**
     * @brief Fused Operation.
     *
     * In a fused operation, a complex command is created by fusing
     * together two simpler commands. This field indicates whether
     * this command is part of a fused operation and if so, which
     * command it is in the sequence.
     * - 00b Normal operation.
     * - 01b Fused operation, first command.
     * - 10b Fused operation, second command.
     * - 11b Reserved.
     */
        fusedOp:2,
     /**
      * @brief Reserved.
      */
        :6,
     /**
      * @brief Command Identifier.
      *
      * This field indicates a unique identifier for the command when
      * combined with the Submission Queue identifier.
      */
        cmdID:16;
    /**
     * @brief Napespace Identifier.
     *
     * This field indicates the namespace that this command applies to.
     * If the namespace is not used for the command, then this field
     * shall be cleared to 0h.  If a command shall be applied to all
     * namespaces on the device, then this value shall be set to
     * FFFFFFFFh.
     */
    __u32 namespaceID;
    /**
     * @brief Reserved.
     */
    __u64 reserved;
    /**
     * @brief Metadata Pointer.
     *
     * This field contains the address of a contiguous physical buffer of
     * metadata.  This field is only used if metadata is not interleaved
     * with the LBA data, as specified in the Format NVM command. This
     * field shall be Dword aligned.
     */
    __u64 metadataPtr;
    /**
     * @brief PRP Entry 1 & 2.
     *
     * PRP entry 1 contains the first PRP entry for the command.
     * PRP entry 2 contains the second PRP entry for the command. If the data
     * transfer spans more than two memory pages, then this field is a PRP
     * List pointer.
     */
    struct nvme_prp prp[2];
};

/**
 * @brief NVM IO command protection information data definition.
 */
#define	PROT_PRACT		(1 << 3)
#define PROT_PRCHK_GUARD	(1 << 2)
#define PROT_PRCHK_APREF	(1 << 1)
#define PROT_PRCHK_LBREF	(1 << 0)
#define PROT_PRCHK_TYPE1	(PROT_PRCHK_APREF|PROT_PRCHK_LBREF|\
						  PROT_PRCHK_GUARD)
#define PROT_PRCHK_TYPE2	(PROT_PRCHK_LBREF|PROT_PRCHK_GUARD)
#define PROT_PRCHK_TYPE3	(PROT_PRCHK_GUARD)

/**
 * @brief NVM read command specific data.
 */
struct nvme_cmd_read
{
    /**
     * @brief Starting LBA.
     *
     * This field indicates the 64-bit address of the first LBA to be written
     * as part
     * of the operation.
     */
    union
    {
        __u64 startLBA;
        __u8 sLBA[8];
    };
    /**
     * @brief Number of logical blocks.
     *
     * This field indicates the number of logical blocks to be
     * written.  This is a 0s based value.
     */
    __u32 numLBA:16,
    :10,
    /**
     * @brief Protection information.
     *
     * Specifies the protection information action and check field.
     */
    protInfo:4,
    /**
     * @brief Force unit access.
     *
     * This field indicates that the data shall be written to non-volatile
     * media before indicating command completion. There is no implied ordering
     * with other commands.
     */
    forceUnitAccess:1,
    /**
     * @brief Limited retry.
     *
     * If set to 1, the controller should apply limited retry efforts. If
     * cleared to 0, the controller should apply all available error recovery
     * means to write the data to the NVM.
     */
    limitedRetry:1;
    /**
     * @brief Dataset management.
     *
     * This field indicates attributes for the dataset that the LBA(s) being
     * read are associated with.
     */
    __u32 datasetMgmnt:8,
                :24;
    /**
     * @brief Expected initial logical block reference tag.
     *
     * This field indicates the Initial Logical Block
     * Reference Tag expected value. This field is only used if the namespace
     * is formatted to use end-to-end protection information.
     */
    __u32 expInitLogBlkRefTag;
    /**
     * @brief Expected logical block application tag.
     *
     * This field indicates the Application Tag expected value. This
     * field is only used if the namespace is formatted to use end-to-end
     * protection information.
     */
    __u32 expLogBlkAppTag:16,
    /**
     * @brief Expected logical block application tag mask.
     *
     * This field indicates the Application Tag Mask expected
     * value. This field is only used if the namespace is formatted to use
     * end-to-end protection information.
     */
    expLogBlkAppTagMsk:16;
};


/**
 * @brief NVM write command specific data.
 */
struct nvme_cmd_write
{
    /**
     * @brief Starting LBA.
     *
     * This field indicates the 64-bit address of the first LBA to be written
     * as part of the operation.
     */
    union
    {
        __u64 startLBA;
        __u8 sLBA[8];
    };
    /**
     * @brief Number of logical blocks.
     *
     * This field indicates the number of logical blocks to be
     * written.  This is a 0s based value.
     */
    __u32 numLBA:16,
            :10,
    /**
     * @brief Protection information.
     *
     * Specifies the protection information action and check field.
     */
    protInfo:4,
    /**
     * @brief Force unit access.
     *
     * This field indicates that the data shall be written to non-volatile
     * media before indicating command completion. There is no implied ordering
     *  with other commands.
     */
    forceUnitAccess:1,
    /**
     * @brief Limited retry.
     *
     * If set to 1, the controller should apply limited retry efforts. If
     * cleared to 0,
     * the controller should apply all available error recovery means to write
     * the data to the NVM.
     */
    limitedRetry:1;
    /**
     * @brief Dataset management.
     *
     * This field indicates attributes for the dataset that the LBA(s) being
     * read are associated with.
     */
    __u32 datasetMgmnt:8,
                :24;
    /**
     * @brief Initial logical block reference tag.
     *
     * This field indicates the Initial Logical Block
     * Reference Tag value. This field is only used if the namespace is
     * formatted to use end-to-end protection information.
     */
    __u32 initLogBlkRefTag;
    /**
     * @brief Logical block application tag.
     *
     * This field indicates the Application Tag value. This
     * field is only used if the namespace is formatted to use end-to-end
     * protection information.
     */
    __u32 logBlkAppTag:16,
    /**
     * @brief Logical block application tag mask.
     *
     * This field indicates the Application Tag Mask
     * value. This field is only used if the namespace is formatted to use
     * end-to-end protection information.
     */
    logBlkAppTagMsk:16;

};

/**
 * @brief NVM dataset command specific LBA Range data format.
 */
struct nvme_dataset_mgmt_data
{

    /**
     * @brief Context Attributes
     *
     * The context attributes specified for each range provides information
     * about how the range is intended to be used by host software. The use
     * of this information is optional and the controller is not required to
     * perform any specific action.
     * Note: The controller is required to maintain the integrity of data on
     * the NVM media regardless of whether the attributes provided by host
     * software are accurate.
     */

     /**
      * @brief Command Access Size
      * Number of logical blocks expected to be transferred in a single Read
      * or Write command from this dataset. A value of 0h indicates no Command
      * Access Size is provided.
      */
     __u32	access_size : 8,

     /**
      * @brief Reserved.
      */
     		reservedA : 13,
     /**
      * @brief Write Prepare.
      * If set to 1 then the provided range is expected to be written in
      * the near future.
      */
		writePrepare : 1,
     /**
      * @brief Sequential Write Range.
      * If set to 1 then the dataset should be optimized for sequential write
      * access. The host expects to perform operations on the dataset as a
      * single object for writes.
      */
		sequentialWriteRange : 1,
     /**
      * @brief Sequential Read Range.
      * If set to 1 then the dataset should be optimized for sequential
      * read access. The host expects to perform operations on the dataset
      * as a single object for reads.
      */
		sequentialReadRange : 1,
     /**
      * @brief Reserved.
      */
     		reservedB : 2,
     /**
      * @brief Access Latency.
      * 00b None. No latency information provided.
      * 01b Idle. Longer latency acceptable.
      * 10b Normal. Typical latency.
      * 11b Low. Smallest possible latency.
      */
		accessLatency : 2,
     /**
      * @brief Access Frequency.
      * 0000b No frequency information provided.
      * 0001b Typical number of reads and writes expected for this LBA range.
      * 0010b Infrequent writes and infrequent reads to the LBA range indicated.
      * 0011b Infrequent writes and frequent reads to the LBA range indicated.
      * 0100b Frequent writes and infrequent reads to the LBA range indicated.
      * 0101b Frequent writes and frequent reads to the LBA range indicated.
      * 0110b Reserved
      * ...   Reserved
      * 1111b Reserved
      *
      */
		accessFrequency : 4;

     /**
      * @brief Length in logical blocks.
      */
    __u32	numLBA;

     /**
      * @brief Starting LBA.
      */
    __u64	startLBA;
};


/**
 * @brief NVM dataset command specific data.
 */
struct nvme_cmd_dataset
{

    /**
     * @brief Number of Ranges
     *
     * Indicates the number of 16 byte range sets that are specified in the
     * command. This is a 0.s based value.
     */
    __u32    numRanges;

    /**
     * @brief Attribute
     *
     * bit 2 - Deallocate (AD): If set to 1 then the NVM subsystem may
     * deallocate all provided ranges. If a read occurs to a deallocated
     * range, the NVM Express subsystem shall return all zeros, all ones,
     * or the last data written to the associated LBA.
     *
     * bit 1 - Integral Dataset for Write (IDW): If set to 1 then the dataset
     * should be optimized for write access as an integral unit. The host
     * expects to perform operations on all ranges provided as an integral
     * unit for writes, indicating that if a portion of the dataset is written
     * it is expected that all of the ranges in the dataset are going to be
     * written.
     *
     * bit 0 - Integral Dataset for Read (IDR): If set to 1 then the dataset
     * should be optimized for read access as an integral unit. The host
     * expects to perform operations on all ranges provided as an integral
     * unit for reads, indicating that if a portion of the dataset is read
     * it is expected that all of the ranges in the dataset are going to be
     * read.
     *
     */
    __u32    attribute;

};


/**
 * @brief NVM create completion queue command specific data.
 */
struct nvme_cmd_create_cplq
{
    /**
     * @brief Queue Identifier.
     *
     * This field indicates the identifier to assign to the Completion Queue to
     * be created. This identifier corresponds to the Completion Queue Head
     * Doorbell * used for this command (i.e., the value y). This value shall
     * not exceed the value reported in the Number of Queues feature for I/O
     * Completion Queues.
     */
    __u32     identifier  : 16,
    /**
     * @brief Queue Size.
     *
     * This field indicates the size of the Completion Queue
     * (in completion queue entries) to be created. This is a 0s based value.
     */
            size        : 16;
    /**
     * @brief Physically Contiguous.
     *
     * If set to 1, then the Completion Queue is physically contiguous
     * and PRP Entry 1 (PRP1) is the address of a contiguous physical buffer.
     * If cleared to 0, then the Completion Queue is not physically
     * contiguous and PRP Entry 1 (PRP1) is a PRP List pointer.
     */
    __u32     contiguous  : 1,
    /**
     * @brief Interrupt Enable.
     *
     * If set to 1, then interrupts are enabled for this Completion Queue. If
     * cleared to 0, then interrupts are disabled for this Completion Queue.
     */
            interruptEnable : 1,
    /**
     * @brief Reserved.
     */
            :14,
    /**
     * @brief Interrupt Vector.
     *
     * This field indicates interrupt vector to use for this Completion Queue.
     * This corresponds to the MSI-X or multiple message MSI vector to use.
     * If using single message MSI or pin-based interrupts, then this field
     * shall be cleared to 0h.  In MSI-X, a maximum of 2K vectors are used.
     * This value shall not be set to a value greater than the number of
     * messages the controller supports (refer to MSICAP.MC.MME or
     * MSIXCAP.MXC.TS).
     */
            interruptVector : 16;
};

/**
 * @brief NVM create submission queue command specific data.
 */
struct nvme_cmd_create_subq
{
    /**
     * @brief Queue Identifier.
     *
     * This field indicates the identifier to assign to the Submission Queue to
     * be created. This identifier corresponds to the Submission Queue Tail
     * Doorbell used for this command (i.e., the value y). This value shall
     * not exceed the value reported in the Number of Queues feature for I/O
     * Submission Queues.
     */
    __u32     identifier  : 16,
    /**
     * @brief Queue Size.
     *
     * This field indicates the size of the Submission Queue
     * (in submission queue entries) to be created. This is a 0s based value.
     */
            size        : 16;
    /**
     * @brief Physically Contiguous.
     *
     * If set to 1, then the Completion Queue is physically contiguous
     * and PRP Entry 1 (PRP1) is the address of a contiguous physical buffer.
     * If cleared to 0, then the Completion Queue is not physically
     * contiguous and PRP Entry 1 (PRP1) is a PRP List pointer.
     */
    __u32     contiguous  : 1,
    /**
     * @brief Queue Priority.
     *
     * This field indicates the priority service class to use for commands
     * within this Submission Queue.  This field is only used when the
     * weighted round robin with an urgent priority service class is the
     * arbitration mechanism is selected.
     * - 00b Urgent
     * - 01b High
     * - 10b Medium
     * - 11b Low
     */
            priority : 2,
    /**
     * @brief Reserved.
     */
            :13,
    /**
     * @brief Completion Queue Identifier.
     *
     * This field indicates the identifier of the Completion Queue
     * to utilize for any command completions entries associated
     * with this Submission Queue. The value of 0h
     * (Admin Completion Queue) shall not be specified.
     */
            completionQueueID : 16;
};

/**
 * @brief NVM delete submission queue command specific data.
 */
struct nvme_cmd_delete_subq
{
    /**
     * @brief Queue Identifier.
     *
     * This field indicates the identifier of the Submission Queue to
     * be deleted. This identifier corresponds to the Submission Queue Tail
     * Doorbell used for this command (i.e., the value y). This value shall
     * not exceed the value reported in the Number of Queues feature for I/O
     * Submission Queues.
     */
    __u32     identifier  : 16;
};

/**
 * @brief NVM delete completion queue command specific data.
 */
struct nvme_cmd_delete_cplq
{
    /**
     * @brief Queue Identifier.
     *
     * This field indicates the identifier of the Completion Queue to
     * be deleted. This identifier corresponds to the Completion Queue Tail
     * Doorbell
     * used for this command (i.e., the value y). This value shall not exceed
     * the value reported in the Number of Queues feature for I/O Completion
     * Queues.
     */
    __u32     identifier  : 16;
};

/**
 * @brief NVM identify command specific data.
 */
struct nvme_cmd_identify
{
    /**
     * @brief Identify strcuture.
     *
     * This field indicates the identify structure to retrieve. If
     * controllerStructure is set as 1, it retrieves the controller structure,
     * otherwise, it retrieves the namespace structure associated with the
     * namespaceID indicated in header.
     */
    __u32     controllerStructure : 1,
            reserved : 31;
};

/**
 * @brief NVM set features command specific data.
 */
struct nvme_cmd_set_feature
{
    /**
     * @brief Feature Identifier.
     *
     * This field indicates the identifier of the Feature that attributes are
     * being specified for.
     */
    __u32     featureID   : 8,
            reserved    : 24;
    __u32     numSubQReq:16, numCplQReq:16;
};


/**
 * @brief NVM set features command specific data.
 */
struct nvme_cmd_get_feature
{
    /**
     * @brief Feature Identifier.
     *
     * This field indicates the identifier of the Feature that attributes are
     * being specified for.
     */
    __u32     featureID   : 8,
            reserved    : 24;
    __u32     numSubQReq:16, numCplQReq:16;
};

/**
 * @brief NVM firmware activate command specific data.
 */
struct nvme_cmd_firmware_activate
{
    /**
     * @brief firmware slot and action.
     *
     * This field indicates the slot of firmware to activate and the specific
     * activate action
     */
    __u32     slot: 3,
            action: 2,    : 24;
};

/**
 * @brief NVM firmware download command specific data.
 */
struct nvme_cmd_firmware_download
{
    /**
     * @brief download size and offset.
     *
     * This field indicates the size of download image in Dwords, and the
     * download offset of the image in Dwords.
     */
    __u32     numDW;
    __u32     offset;
};


/**
 * @brief NVM get log page command specific data.
 */
struct nvme_cmd_get_log_page
{
    /**
     * @brief log page identifier and log page size.
     *
     * This field indicates the log page identifier and log page size
     * in Dwords.
     */
    __u16     LogPageID;
    __u16     numDW:12,reserved:4;
};

/**
 * @brief NVM Format Media command specific data.
 */
struct nvme_cmd_format
{
    /**
     * @brief The Format NVM command is used to low level format the NVM media.
     *
     * This field selects various format options.
     *
     * 31-12:	Reserved
     * 11-09:   Secure Erase Settings (SES):
     *		This field specifies whether a secure erase should be performed
     *		as part of the format and the type of the secure erase
     *		operation.
     *
     * 08:	Protection Information Location (PIL):
     *		If set to "1" and protection information is enabled, then
     *		protection information is transferred as the first eight bytes
     *		of metadata. If cleared to "0" and protection information is
     *		enabled, then protection information is transferred as the
     *		last eight bytes of metadata.
     *
     * 07-05:	Protection Information (PI):
     *		This field specifies whether end-to-end data protection is
     *		enabled and the type of protection information. The values
     *		for this field have the following meanings:
     *
     *		000b 	    Protection information is not enabled
     *		001b	    Protection information is enabled, Type 1
     *		010b	    Protection information is enabled, Type 2
     *		011b	    Protection information is enabled, Type 3
     *		100-111b    Reserved
     *
     * 04:	Metadata Settings (MS):
     *		This field is set to "1" if the metadata is transferred as
     *		part of an extended data LBA. This field is cleared to "0" if
     *		the metadata is transferred as part of a separate buffer.
     *		The metadata may include protection information, based on the
     *		Protection Information (PI) field.
     *
     * 03-00:	LBA Format (LBAF):
     *		This field specifies the LBA format to apply to the NVM media.
     *		This corresponds to the LBA formats indicated in the Identify
     *		command, Only supported LBA formats shall be selected.
     */
#define	FORMAT_LBAF_SHIFT	0
#define	FORMAT_META_SHIFT	4
#define FORMAT_PI_SHIFT		5
#define FORMAT_PIL_SHIFT	8
#define FORMAT_SECURITY_SHIFT	9
    __u32     formatOption;
};


/**
 * @brief NVM/Admin Vendor command specific data,
 */
struct nvme_cmd_vendor_specific
{
    /**
     * @brief Pass-through data and meta-date lengths and command specific
     *		parameters
     *
     */
    __u32     buffNumDW;
    __u32     metaNumDW;
    __u32     vndrCDW12;
    __u32     vndrCDW13;
    __u32     vndrCDW14;
    __u32     vndrCDW15;
};


/**
 * @brief NVM Command Structure.
 */
struct nvme_cmd
{
    union
    {
	struct
	{
            /**
             * @brief Command header.
             */
            struct nvme_cmd_hdr header;
            union
            {
                /**
                 * @brief NVM read command specific information.
                 */
                struct nvme_cmd_read read;

                /**
                 * @brief NVM write command specific information.
                 */
                struct nvme_cmd_write write;

                /**
                 * @brief NVM dataset command specific information.
                 */
                struct nvme_cmd_dataset dataset;

                /**
                 * @brief NVM create completion queue command specific info.
                 */
                struct nvme_cmd_create_cplq createCplQ;

                /**
                 * @brief NVM create completion queue command specific info.
                 */
                struct nvme_cmd_create_subq createSubQ;

                /**
                 * @brief NVM create completion queue command specific info.
                 */
                struct nvme_cmd_delete_subq deleteSubQ;

                /**
                 * @brief NVM create completion queue command specific info.
                 */
                struct nvme_cmd_delete_cplq deleteCplQ;

                /**
                 * @brief NVM identify command specific info.
                 */
                struct nvme_cmd_identify identify;
                /**
                 * @brief NVM set features command specific info.
                 */
                struct nvme_cmd_set_feature setFeatures;
                /**
                 * @brief NVM get features command specific info.
                 */
                struct nvme_cmd_get_feature getFeatures;
                /**
                 * @brief NVM firmware activate command specific info.
                 */
                struct nvme_cmd_firmware_activate firmwareActivate;
                /**
                 * @brief NVM firmware download command specific info.
                 */
                struct nvme_cmd_firmware_download firmwareDownload;
		/**
		 * @brief NVM get log page command specific data.
		 */
		struct nvme_cmd_get_log_page getLogPage;
		/**
		 * @brief NVM Format Media command specific data.
		 */
		struct nvme_cmd_format format;
		/**
		 * @brief NVM/Admin command Vendor Specific command data.
		 */
		struct nvme_cmd_vendor_specific vendorSpecific;
		/**
		 * generic command template.
		 */
	    	__u32 	asUlong[6];
	    } cmd;
        };
        __u32 dw[16];
    };
};

/**
 * @brief Completion queue entry structure.
 */
struct cq_entry
{
    /**
     * @brief Command Specific Completion Code.
     */
    union
    {
        __u32 cmdSpecific;
        __u32 numSubQAlloc:16, numCplQAlloc:16;
    }param;
    /**
     * @brief Reserved.
     */
    __u32 reserved;
    __u32 sqHdPtr:16, sqID:16;
    __u32 cmdID:16, phaseTag:1, SC:8, SCT:3,:2, more:1, noRetry:1;
};

/**
 * @brief Identify - Power State Descriptor Data Structure.
 */
struct pwr_state_desc
{
    /**
     * @brief Maximum Power.
     *
     * This field indicates the maximum power consumed by the NVM
     * subsystem in this power state. The power in Watts is equal to the value
     * in this field multiplied by
     * 0.01
     */
    __u16 maxPower;
    /**
     * @brief Reserved.
     */
    __u16 reservedA;
    /**
     * @brief Entry Latency.
     *
     * This field indicates the maximum entry latency in microseconds
     * associated with entering this power state.
     */
    __u32 entryLat;
    /**
     * @brief Exit Latency.
     *
     * This field indicates the maximum exit latency in microseconds
     * associated with entering this power state.
     */
    __u32 exitLat;
    /**
     * @brief Relative Read Throughput.
     *
     * This field indicates the relative read throughput associated
     * with this power state. The value in this field shall be less than the
     * number of supported power states (e.g., if the controller supports
     * 16 power states, then valid values are 0 through 15). A lower value
     * means higher read throughput.
     */
    __u8 relRdThpt;
    /**
     * @brief Relative Read Latency.
     *
     * This field indicates the relative read latency associated with
     * this power state. The value in this field shall be less than the number
     * of supported power states (e.g., if the controller supports 16 power
     * states, then valid values are 0 through 15). A lower value means lower
     * read latency.
     */
    __u8 relRdLat;
    /**
     * @brief Relative Write Throughput.
     *
     * This field indicates the relative write throughput
     * associated with this power state. The value in this field shall be less
     * than the number of supported power states (e.g., if the controller
     * supports 16 power states, then valid values are 0 through 15). A lower
     * value means higher write throughput.
     */
    __u8 relWrThpt;
    /**
     * @brief Relative Write Latency.
     *
     * This field indicates the relative write latency associated with
     * this power state. The value in this field shall be less than the number
     * of supported power states (e.g., if the controller supports 16 power
     * states, then valid values are 0 through 15). A lower value means lower
     * write latency.
     */
    __u8 relWrLat;
    /**
     * @brief Reserved.
     */
    __u8 reserveds[16];
};

/**
 * @brief Identify - LBA Format Data Structure.
 */
struct lba_format
{
    /**
     * @brief Metadata Size.
     *
     * This field indicates the number of metadata bytes provided per LBA
     * based on the LBA Size indicated.  The namespace may support the metadata
     * being transferred as part of an extended data LBA or as part of a
     * separate contiguous buffer.  If end-to-end data protection is enabled,
     * then the first eight bytes or last eight bytes of the metadata is the
     * protection information.
     */
    __u16 metaSize;
    /**
     * @brief LBA Data Size.
     *
     * This field indicates the LBA data size supported.  The value is
     * reported in terms of a power of two (2^n).  A value smaller than 9
     * (i.e. 512 bytes) is not supported.  If the value reported is 0h
     * then the LBA format is not supported / used.
     */
    __u8 dataSize;
    /**
     * @brief Relative Performance.
     *
     * This field indicates the relative performance of the LBA format
     * indicated relative to other LBA formats supported by the controller.
     * Depending on the size of the LBA and associated metadata, there may
     * be performance implications.  The performance analysis is based
     * on better performance on a queue depth 32 with 4KB read workload. The
     * meanings of the values indicated are included in the following table.
     * - 00b     Best performance
     * - 01b     Better performance
     * - 10b     Good performance
     * - 11b     Degraded performance
     */
     __u8 relPerf;
};


/**
 * @brief LbaRange - LbaRange Data Structure.
 */
struct lba_range
{
    /**
     * @brief LBA Range Type.
     *
     * - 00h		Reserved
     * - 01h		Filesystem
     * - 02h		RAID
     * - 03h		Cache
     * - 04h		Page / swap file
     * - 05h - 7Fh	Reserved
     * - 80h - FFh	Vendor Specific
     *
     */
    __u8 type;
    /**
     * @brief Attributes.
     *
     * This field Identifies attributes of the LBA range.  Each bit defines
     * an attribute.
     * 0	If set to 1, the LBA range may be overwritten. If cleared
     *		to 0, the area should not be overwritten.
     * 1	If set to .1., the LBA range should be hidden from the
     *		OS/EFI/BIOS.  If cleared to 0, the area should be visible
     *		to the OS/EFI/BIOS.
     * 2 - 7	Reserved
     *
     */
    __u8 attributes;
    /**
     * @brief Reserved.
     */
    __u8 reserved[14];
    /**
     * @brief start_lba.
     *
     * This field defines the 64-bit address of the first LBA that is part
     * of this LBA range.
     */
    __u64 start;
    /**
     * @brief Starting LBA.
     *
     * This field defines the number of logical blocks that are part of
     * this LBA range.  This is a 0 based value.
     */
    __u64 size;
    /**
     * @brief Unique Identifier (GUID).
     *
     * This field defines a global unique identifier that uniquely identifies
     * the type of this LBA range.  Well known Types may be defined and are
     * published on the NVMHCI website.
     */
    __u8 GUID[16];
    /**
     * @brief padding.
     */
    __u8 padding[63-48+1];
};


/**
 * @brief Identify - Meta Data Capability field definitions.
 */
#define METADATA_MBUF		(1 << 1)
#define METADATA_LBA		(1 << 0)
#define METADATA_SUPPORT(x)	(x & METADATA_LBA|

/**
 * @brief Identify - Data Protection Capability field definitions.
 */
#define END2END_CAP_LAST_8B		(1 << 4)
#define END2END_CAP_FIRST_8B		(1 << 3)
#define END2END_CAP_TYPE3		(1 << 2)
#define END2END_CAP_TYPE2		(1 << 1)
#define END2END_CAP_TYPE1		(1 << 0)
#define END2END_CAP_TYPE(x)		(x & 0x07)

/**
 * @brief Identify - Data Protection Type settings field definitions.
 */
#define END2END_DPS_FIRST	(1 << 3)
#define END2END_DSP_TYPE(x)	(x & 0x07)


/**
 * @brief Identify - Namespace Data Structure.
 */
struct iden_namespace
{
    /**
     * @brief Namespace Size.
     *
     * This field indicates the total size of the namespace
     * in logical blocks. A namespace of size n consists of LBA 0 through
     * (n - 1).
     * The number of logical blocks is based on the formatted LBA size.
     * This field is undefined prior to the namespace being formatted.
     *
     * @note The creation of the namespace(s) and initial format operation
     * are outside the scope of this specification.
     */
    __u64 size;
    /**
     * @brief Namespace Capacity.
     *
     * This field indicates the maximum number of logical
     * blocks that may be allocated in the namespace at any point in time.
     * The number of logical blocks is based on the formatted LBA size. This
     * field is undefined prior to the namespace being formatted. A value
     * of 0h for the Namespace Capacity indicates that the namespace is not
     * available for use. When using the NVM command set: A logical block
     * is allocated when it is written with a Write or Write Uncorrectable
     * command.  A logical block may be deallocated using the Dataset
     *  Management command.
     */
    __u64 capacity;
    /**
     * @brief Namespace Utilization.
     *
     * This field indicates the current number of
     * logical blocks allocated in the namespace.  The number of logical blocks
     * is based on the formatted LBA size. When using the NVM command set: A
     * logical block is allocated when it is written with a Write or Write
     * Uncorrectable command.  A logical block may be deallocated using the
     * Dataset Management command.
     */
    __u64 utilization;
    /**
     * @brief Namespace Features.
     *
     * This field defines features of the namespace.
     * - Bits 7:1 are reserved.
     * - Bit 0 if set to '1' indicates that the namespace supports thin
     * provisioning. Specifically, the Namespace Size reported may be less
     * than the Namespace Capacity.  When this feature is supported and the
     * Dataset Management command is supported then deallocating LBAs shall
     * be reflected in the Namespace Size field.  Bit 0 if cleared to '0'
     * indicates that thin provisioning is not supported and the Namespace
     * Size and Namespace Capacity fields report the same value.
     */
    __u8 feat;
    /**
     * @brief Number of LBA Formats.
     *
     * This field defines the number of supported LBA size and metadata size
     * combinations supported by the namespace.  LBA
     * formats shall be allocated in order (starting with 0) and packed
     * sequentially.
     * This is a 0's based value.  The maximum number of LBA formats that may be
     * indicated as supported is 16.  The supported LBA formats are indicated
     * in bytes 128 - 191 in this data structure. The metadata may be either
     * transferred as part of the LBA (creating an extended LBA which is a
     * larger LBA size that is exposed to the application) or it may be
     * transferred as a separate contiguous buffer of data.  The metadata
     * shall not be split between the LBA and a separate metadata buffer.
     * It is recommended that software and controllers transition to an LBA
     * size that is 4KB or larger for ECC efficiency at the controller.  If
     * providing metadata, it is recommended that at least 8 bytes is provided
     * per logical block to enable use with end-to-end data protection.
     */
    __u8 numLbaFmt;
    /**
     * @brief Formatted LBA Size.
     *
     * This field indicates the LBA size & metadata size combination that the
     * namespace has been formatted with.
     * - Bits 7:5 are reserved.
     * - Bit 4 if set to '1' indicates that the metadata is transferred at
     * the end of the data LBA, creating an extended data LBA.  Bit 4 if
     * cleared to '0' indicates that all of the metadata for a command is
     * transferred as a separate contiguous buffer of data.
     * - Bits 3:0 indicates one of the 16 supported combinations indicated in
     * this data structure.  This is a 0's based value.
     */
    __u8 fmtLbaSize;
    /**
     * @brief Metadata Capabilities
     *
     * This field indicates the capabilities for metadata.
     * - Bits 7:2 are reserved.
     * - Bit 1 if set to '1' indicates the namespace supports the metadata
     *   being transferred as part of a separate buffer that is specified in
     *   the Metadata Pointer.  Bit 1 if cleared to '0' indicates that the
     *   controller does not support the metadata being transferred as part
     *   of a separate buffer.
     * - Bit 0 if set to '1' indicates that the namespace supports the
     *   metadata being transferred as part of an extended data LBA.
     *   Specifically, the metadata is transferred as part of the data PRP List.
     *   Bit 0 if cleared to '0' indicates that the namespace does not support
     *   the metadata being transferred as part of an extended data LBA.
     */
    __u8 metaDataCap;
    /**
     * @brief End-to-end Data Protection Capabilities
     *
     * This field indicates the capabilities for the end-to-end data protection
     *  feature.
     * - Bits 7:5 are reserved.
     * - Bit 4 if set to '1' indicates that the namespace supports protection
     *   information transferred as the last eight bytes of metadata.  Bit 4 if
     *   cleared to '0' indicates that the namespace does not support protection
     *   information transferred as the last eight bytes of metadata.
     * - Bit 3 if set to '1' indicates that the namespace supports protection
     *   information transferred as the first eight bytes of metadata.  Bit 3
     *   if cleared to '0' indicates that the namespace does not support
     *   protection
     *   information transferred as the first eight bytes of metadata.
     * - Bit 2 if set to '1' indicates that the namespace supports Protection
     *   Information Type 3.  Bit 2 if cleared to '0' indicates that the
     *   namespace
     *   does not support Protection Information Type 3.
     * - Bit 1 if set to '1' indicates that the namespace supports Protection
     *   Information Type 2.  Bit 1 if cleared to '0' indicates that the
     *   namespace does not support Protection Information Type 2.
     * - Bit 0 if set to '1' indicates that the namespace supports Protection
     *   Information Type 1.  Bit 0 if cleared to '0' indicates that the
     *   namespace does not support Protection Information Type 1.
     */
    __u8 dataProtCap;
    /**
     * @brief End-to-end Data Protection Type Settings
     *
     * This field indicates the type settings for the end-to-end data
     * protection feature.
     * - Bits 7:4 are reserved.
     * - Bit 3 if set to '1' indicates that the protection information, if
     *   enabled, is transferred as the first eight bytes of metadata.
     *   Bit 3 if cleared to '0' indicates that the protection information,
     *   if enabled, is transferred as the last eight bytes of metadata.
     * - Bits 2:0 indicate whether Protection Information is enabled and the
     *   type of Protection Information enabled.  The values for this field
     *   have the following meanings:
     *   - 000b    Protection information is not enabled
     *   - 001b    Protection information is enabled, Type 1
     *   - 010b    Protection information is enabled, Type 2
     *   - 011b    Protection information is enabled, Type 3
     *   - 100b - 111b    Reserved
     */
    __u8 dataProtSet;
    /**
     * @brief Reserved
     */
    __u8 reservedA[127-30+1];
    /**
     * @brief LBA Format Support
     *
     * This field indicates the LBA formats that are supported by the
     * controller.
     */
    struct lba_format lbaFmtSup[16];
    /**
     * @brief Reserved
     */
    __u8 reservedB[383-192+1];
    /**
     * @brief Vendor Specific
     *
     * This range of bytes is allocated for vendor specific usage.
     */
    __u8 vendor[4095-384+1];
};

/**
 * @brief Identify - Controller Data Structure.
 */
struct iden_controller
{
    /**
     * @brief PCI Vendor ID
     *
     * Contains the company vendor identifier that is assigned by the
     * PCI SIG. This is the same value as reported in the ID register in
     * section 2.1.1.
     */
    __u16 pcieVID;
    /**
     * @brief PCI Subsystem Vendor ID
     *
     * Contains the company vendor identifier that is
     * assigned by the PCI SIG for the subsystem. This is the same value as
     * reported in the SS register.
     */
    __u16 pcieSSVID;
    /**
     * @brief Serial Number
     *
     * Contains the serial number for the NVM subsystem that is
     * assigned by the vendor as an ASCII string.
     */
    __u8 serialNum[20];
    /**
     * @brief Model Number
     *
     * Contains the model number for the NVM subsystem that is
     * assigned by the vendor as an ASCII string.
     */
    __u8 modelNum[40];
    /**
     * @brief Firmware Revision
     *
     * Contains the currently active firmware revision for the
     * NVM subsystem. This is the same revision information that may be
     * retrieved with the Get Log Page command.
     */
    __u8 firmwareRev[8];
    /**
     * @brief Recommended Arbitration Burst
     *
     * This is the recommended Arbitration Burst size.
     */
    __u8 arbBurstSize;
    /**
     * @brief Reserved
     */
    __u8 reservedA[255-73+1];
    /**
     * @brief Optional Admin Command Support
     *
     * This field indicates the optional
     * Admin commands supported by the controller.
     * - Bits 15:3 are reserved.
     * - Bit 2 if set to 1 then the controller supports the Firmware Activate
     *   and Firmware Download commands. If cleared to 0 then the controller
     *   does not support the Firmware Activate and Firmware Download commands.
     * - Bit 1 if set to 1 then the controller supports the Format NVM command.
     *   If cleared to 0 then the controller does not support the Format NVM
     *   command.
     * - Bit 0 if set to 1 then the controller supports the Security Send and
     *   Security Receive commands. If cleared to 0 then the controller does
     *   not support the Security Send and Security Receive commands.
     */
    __u16 adminCmdSup;
    /**
     * @brief Abort Command Limit
     *
     * This field is used to convey the maximum number of
     * concurrently outstanding Abort commands supported by the controller.
     * This is a 0s based value. It is recommended that implementations support
     * a minimum of four Abort commands outstanding simultaneously.
     * abortComLmt;
     */
    __u8 abortCmdLmt;
    /**
     * @brief Asynchronous Event Request Limit
     *
     * This field is used to convey the
     * maximum number of concurrently outstanding Asynchronous Event Request
     * commands supported by the controller. This is a 0s based value.
     * It is recommended that implementations support a minimum of four
     * Asynchronous Event Request Limit commands oustanding simultaneously.
     */
    __u8 asyncReqLmt;
    /**
     * @brief Firmware Updates
     *
     * This field indicates capabilities regarding firmware
     * updates.
     * - Bits 7:4 are reserved.
     * - Bits 3:1 indicate the number of firmware slots that the device
     *   supports. This field shall specify a value between one and seven,
     *   indicating that at least one firmware slot is supported and up to
     *   seven maximum. This corresponds to firmware slots 1 through 7.
     * - Bit 0 if set to 1 indicates that the first firmware slot (slot 1)
     *   is read only. If cleared to 0 then the first firmware slot (slot 1)
     *   is read/write. Implementations may choose to have a baseline
     *   read only firmware image.
     */
    __u8 firmUpdt;
    /**
     * @brief Log Page Attributes
     *
     * This field indicates optional attributes for log pages that
     * are accessed via the Get Log Page command.
     * - Bits 7:1 are reserved.
     * - Bit 0 if set to 1 then the controller supports the SMART / Health
     *   information log page on a per namespace basis. If cleared to 0
     *   then the controller does not support the SMART / Health information
     *   log page on a per namespace basis; the log page returned is global
     *   for all namespaces.
     */
    __u8 logPgAttrib;
    /**
     * @brief Error Log Page Entries
     *
     * This field indicates the number of Error Information
     * log entries that are stored by the controller. This field is a 0s based
     * value.
     */
    __u8 errLogPgEntr;
    /**
     * @brief Number of Power States Support
     *
     * This field indicates the number of
     * NVMHCI power states supported by the controller. This is a 0s based
     * value. Power states are numbered sequentially starting at power state 0.
     * A controller shall support at least one power state (i.e., power
     * state 0) and may support up to 31 additional power states
     * (i.e., up to 32 total).
     */
    __u8 numPowerSt;
    /**
     * @brief Admin Vendor Specific Command Configuration (AVSCC):
     *
     * This field indicates the configuration settings for admin vendor
     * specific command handling.
     *
     * Bits 7:1 are reserved.
     *
     * Bit 0 if set to .1. indicates that all Admin Vendor Specific Commands
     * use the format defined in Figure 8. If cleared to .0. indicates that
     * format of all Admin Vendor Specific Commands is vendor specific.
     */
    __u8 admVendCmdCfg;
    /**
     * @brief Reserved
     */
    __u8 reservedB[511-265+1];
    /**
     * @brief Submission Queue Entry Size
     *
     * This field defines the required and
     * maximum Submission Queue entry size when using the NVM Command Set.
     * - Bits 7:4 define the maximum Submission Queue entry size when using
     *   the NVM Command Set. This value is larger than or equal to the
     *   required SQ entry size. The value is in bytes and is reported as
     *   a power of two (2^n).
     * - Bits 3:0 define the required Submission Queue Entry size when using
     *   the NVM Command Set. This is the minimum entry size that may be used.
     *   The value is in
     *   bytes and is reported as a power of two (2^n). The required value
     *   shall be 6, corresponding to 64.
     */
    __u8 subQSize;
    /**
     * @brief Completion Queue Entry Size
     *
     * This field defines the required and
     * maximum Completion Queue entry size when using the NVM Command Set.
     * - Bits 7:4 define the maximum Completion Queue entry size when using
     *   the NVM Command Set. This value is larger than or equal to the
     *   required CQ entry size. The value is in bytes and is reported as
     *   a power of two (2^n).
     * - Bits 3:0 define the required Completion Queue entry size when using
     *   the NVM Command Set. This is the minimum entry size that may be used.
     *    The value is in bytes and is reported as a power of two (2^n).
     *    The required value shall be 4, corresponding to 16.
     */
    __u8 compQSize;
    /**
     * @brief Reserved
     */
    __u8 reservedC[515-514+1];
    /**
     * @brief Number of Namespaces
     *
     * This field defines the number of valid namespaces
     * present for the controller. Namespaces shall be allocated in order
     * (starting with 0) and packed sequentially. This is a 0s based value.
     */
    __u32 numNmspc;
    /**
     * @brief Optional NVM Command Support
     *
     * This field indicates the optional NVM
     * commands supported by the controller. Refer to section 6.
     * - Bits 15:3 are reserved.
     * - Bit 2 if set to 1 then the controller supports the Dataset Management
     *   command. If cleared to 0 then the controller does not support the
     *   Dataset Management command.
     * - Bit 1 if set to 1 then the controller supports the Write Uncorrectable
     *   command. If cleared to 0 then the controller does not support the
     *   Write Uncorrectable command.
     * - Bit 0 if set to 1 then the controller supports the Compare command.
     *   If cleared to 0 then the controller does not support the Compare
     *   command.
     */
    __u16 cmdSupt;
    /**
     * @brief Fused Operation Support
     *
     * This field indicates the fused operations that
     * the controller supports.
     * - Bits 15:1 are reserved.
     * - Bit 0 if set to 1 then the controller supports the Compare and Write
     *   fused operation. If cleared to 0 then the controller does not support
     *   the Compare and Write fused operation. Compare shall be the first
     *   command in the sequence.
     */
    __u16 fuseSupt;
    /**
     * @brief Format NVM Attributes
     *
     * This field indicates attributes for the Format NVM
     * command.
     * - Bits 7:3 are reserved.
     * - Bit 2 indicates whether cryptographic erase is supported as part of
     *   the secure erase functionality. If set to 1, then cryptographic
     *   erase is supported. If cleared to 0, then cryptographic erase is
     *   not supported.
     * - Bit 1 indicates whether secure erase functionality applies to all
     *   namespaces or is specific to a particular namespace. If set to1,
     *   then a secure erase of a particular namespace as part of a format
     *   results in a secure erase of all namespaces. If cleared to 0, then
     *   a secure erase as part of a format is performed on a per namespace
     *   basis.
     * - Bit 0 indicates whether the format operation applies to all namespaces
     *   or is specific to a particular namespace. If set to 1, then all
     *   namespaces shall be configured with the same attributes and a format
     *   of any namespace results in a format of all namespaces. If cleared
     *   to 0, then the controller supports format on a per namespace basis.
     */
    __u8 cmdAttrib;
    /**
     * @brief Volatile Write Cache
     *
     * This field indicates attributes related to the presence
     * of a volatile write cache in the implementation.
     * - Bits 7:1 are reserved.
     * - Bit 0 if set to 1 indicates that a volatile write cache is present.
     *   If cleared to 0, a volatile write cache is not present. If a volatile
     *   write cache is present, then the host may issue Flush commands and
     *   control whether it is enabled with Set Features specifying the
     *   Volatile Write Cache feature identifier. If a volatile write cache
     *   is not present, the host shall not issue Flush commands nor Set
     *   Features or Get Features with the Volatile Write Cache identifier.
     */
    __u8 volWrCache;
    /**
     * @brief Atomic Write Unit Normal
     *
     * This field indicates the atomic write size for the
     * controller during normal operation. This field is specified in logical
     * blocks and is a 0s based value. If a write is issued of this size or
     * less, the host is guaranteed that the write is atomic to the NVM with
     * respect to other read or write operations. A value of FFh indicates
     * all commands are atomic as this is the largest command size. It is
     * recommended that implementations support a minimum of 128KB
     * (appropriately scaled based on LBA size).
     */
    __u16 atomWrNorm;
    /**
     * @brief Atomic Write Unit Power Fail
     *
     * This field indicates the atomic write size for the controller during a
     * power fail condition. This field is specified in logical blocks and
     * is a 0s based value. If a write is issued of this size or less, the
     * host is guaranteed that the write is atomic to the NVM with respect
     * to other read or write operations.
     */
    __u16 atomWrFail;
    /**
     * @brief NVM Vendor Specific Command Configuration (NVSCC):
     * This field indicates the configuration settings for NVM vendor
     * specific command handling.
     *
     * Bits 7:1 are reserved.
     *
     * Bit 0 if set to .1. indicates that all NVM Vendor Specific Commands
     * use the format defined in Figure 8. If cleared to .0. indicates that
     * format of all NVM Vendor Specific Commands is vendor specific.
     */
    __u8 nvmVendCmdCfg;
    /**
     * @brief Reserved
     */
    __u8 reservedE[2047-531+1];
    /**
     * @brief Power State Descriptors
     *
     * This field indicates the characteristics of the power states.
     */
    struct pwr_state_desc pwrStateDesc[32];

    /**
     * @brief Vendor Specific
     * This range of bytes is allocated for vendor specific usage.
     */
    __u8 resevedF[4095-3072+1];
};

#endif
