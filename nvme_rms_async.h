/*
 * Copyright 2016 Radian Memory Systems LLC
 */
#ifndef NVME_RMS_DEFINED
#define NVME_RMS_DEFINED

#include <linux/syscalls.h>
#include <linux/eventfd.h>
#include <linux/hashtable.h>


/*
 * forward
 */
struct async_cpl;

struct hif_async_cpl_cmd {
	int dw10, dw11;
	int ncpl;
};

void nvme_async_init(struct nvme_dev* dev);
void nvme_async_fini(struct nvme_dev* dev);
int  nvme_async_claim(struct nvme_dev* dev, int ncpl, struct async_cpl* p);
int  nvme_async_set_eventfd(struct nvme_dev* dev, int __user* ip);
void nvme_async_post_cpl(struct nvme_dev* dev);
void nvme_async_compl(struct nvme_dev* dev, struct cmd_info* cmd_info);
int  nvme_async_cmd_rms(struct queue_info *qinfo, struct cmd_info *cmd_info, int timeout);
void nvme_async_disk_open(struct ns_info* ns);
void nvme_async_disk_close(struct ns_info* ns);

int  nvme_send_cmd(struct sub_queue_info *sqinfo, struct cmd_info *cmd_info);
void nvme_unmap_user_pages(struct nvme_dev *dev, struct usr_io *uio, struct cmd_info *cmd_info);
struct queue_info * get_nvmeq(struct nvme_dev *dev);
void nvme_put_cmd(struct queue_info *qinfo, struct cmd_info *cmd_info);
#endif /* NVME_RMS_DEFINED */
