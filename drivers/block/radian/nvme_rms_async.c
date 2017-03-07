#define CONFIG_ASYNC_CMD
#include <linux/bio.h>
#include <linux/scatterlist.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "nvme_express.h"
#include "nvme.h"
#include "nvme_ioctl.h"
#include "nvme_rms_async.h"

#ifdef CONFIG_ASYNC_CMD
const int trace_async = 0;

/*
 * CONFIG_ASYNC_TIMEOUT
 * Enforce a timeout for async commands.
 *   (somewhat experimental)
 */
#define CONFIG_ASYNC_TIMEOUT

/*
 * CONFIG_TRACK_PIDS
 * Tech to keep track of ioctl/pid relationships.
 *   Not used...
 *
#define CONFIG_TRACK_PIDS
 */

/*
 * Implement a scheme whereby arbitrary commands can be "completed"
 *   (to userspace) immediately, and whose completion can be
 *   claimed/waited-for later thru another ioctl.
 */

/*
 * TODO(craigr):
 * .  handle timeouts (these used to be handled inline by sleeping)
 * -  verify status returns in all cases
 * -  cleanup for dead pids
 */

struct async_state {
	struct list_head list;
	struct usr_io2 uio;
	u32		status;
	struct cmd_info *cmd_info;
	struct queue_info* qinfo;
	struct eventfd_ctx* evctx;
};

/* alias of struct hif_async_cpl in userspace... */
struct async_cpl {
	u64		opaque;
	u32		status;
	u32		_rsv;
};

void
nvme_async_init(struct nvme_dev* dev)
{
	struct rms_priv* priv = &dev->rms;

    printk(KERN_INFO "init list head\n");
	INIT_LIST_HEAD(&priv->working);
	INIT_LIST_HEAD(&priv->claimable);
	spin_lock_init(&priv->lock);
    printk(KERN_INFO "init list head - done\n");
};

void
nvme_async_fini(struct nvme_dev* dev)
{
	struct async_state *asp, *next;
	unsigned long flags;

	/*
	 * should clean up our kmalloc's...
	 */
	dev->lock_func(&dev->rms.lock, &flags);
	list_for_each_entry_safe(asp, next, &dev->rms.working, list) {
		list_del(&asp->list);
		printk(KERN_INFO "free working opaque %llx\n", asp->uio.async_opaque);
		kfree(asp);
	}

	list_for_each_entry_safe(asp, next, &dev->rms.claimable, list) {
		list_del(&asp->list);
		printk(KERN_INFO "free claimable opaque %llx\n", asp->uio.async_opaque);
		kfree(asp);
	}
}

#ifdef CONFIG_TRACK_PIDS
/*
 * Keep a small hashtable of active pids (that have
 *   our device open... to allow management of async
 *   replies.
 * TODO: right now, notifications can span namespaces...
 *   Do we want to restrict this?
 */
static DEFINE_HASHTABLE(pids, 8);

static int
hash_pid(int pid)
{
	return hash_32(pid, HASH_BITS(pids));
}

struct pid_entry {
	struct hlist_node hash;
	int count;
	struct eventfd_ctx* evctx;
};

static struct pid_entry*
hash_find_pid(int pid)
{
	struct pid_entry *pe;
	int hashi = hash_pid(pid);

	pe = 0;
	hash_for_each_possible(pids, pe, hash, hashi) {
		if (pe)
			break;
	}

	return pe;
}

struct eventfd_ctx*
nvme_async_get_notify_fd(int pid)
{
	struct pid_entry* pe = hash_find_pid(pid);
	struct eventfd_ctx* ctx = 0;

	if (pe)
		ctx = pe->evctx;

	return ctx;
}

void
nvme_async_cleanup_pid(int pid, struct pid_entry* pe)
{

}
#endif

void
nvme_async_disk_open(struct ns_info* ns)
{
#ifdef CONFIG_TRACK_PIDS
	struct pid_entry *pe;
	int pid = current->pid;
	int hashi = hash_pid(pid);

	/*
	 * We need to keep a list of pid's... and use-count...
	 *   so we know how to clean up any async commands
	 *   that may be extant when the last reference to
	 *   the drive goes away...
	 * Approach: maintain a pid->{ count, eventfd_ctx } hash...
	 */
	pe = hash_find_pid(pid);
	
	if (!pe) {
		pe = kzalloc(sizeof(*pe), GFP_KERNEL);
		hash_add(pids, &pe->hash, hashi);
	}

	pe->count++;
	//printk(KERN_INFO "open %5d %5d ns %d\n", pid, pe->count, ns->id);
#endif
}

void
nvme_async_disk_close(struct ns_info* ns)
{
#ifdef CONFIG_TRACK_PIDS
	struct pid_entry* pe;
	int pid = current->pid;

	/* need to keep ref count... so when it goes to 0, we can
	 * clean up claimable/working async items.
	 */
	pe = hash_find_pid(pid);
	if (pe) {
		pe->count--;

		//printk(KERN_INFO "clos %5d %5d ns %d state %x flags %x %s\n", pid, pe->count, ns->id,
		//	   current->exit_state, current->flags,
		//	   current->flags & PF_EXITING ? "exitting" : "??");

		if (pe->count <= 0) {
			// ... hmm
			nvme_async_cleanup_pid(pid, pe);
			hash_del(&pe->hash);
			kfree(pe);
		}
	}
	else {
		printk(KERN_INFO "pid %d not found\n", pid);
	}
#endif
}

void
nvme_async_dump(struct nvme_dev* dev)
{
	struct async_state* p;
	unsigned long flags;

	dev->lock_func(&dev->rms.lock, &flags);
	list_for_each_entry(p, &dev->rms.working, list) {
		int pid;
		pid = p->cmd_info && p->cmd_info->task ? p->cmd_info->task->pid : 0;
		printk(KERN_INFO "working: %llx status %x pid %d\n",
			   p->uio.async_opaque, p->status, pid);
	}
	list_for_each_entry(p, &dev->rms.claimable, list) {
		int pid;
		pid = p->cmd_info && p->cmd_info->task ? p->cmd_info->task->pid : 0;

		printk(KERN_INFO "claimable: %llx status %x pid %d\n",
			   p->uio.async_opaque, p->status, pid);
	}
	dev->unlock_func(&dev->rms.lock, &flags);
}

/*
 * Cleanup any claimables for which pid has exitted.
 */
int
nvme_async_cleanup(struct nvme_dev* dev)
{
	struct async_state *asp, *next;
	unsigned long flags;
	struct list_head claimable;

	INIT_LIST_HEAD(&claimable);

	dev->lock_func(&dev->rms.lock, &flags);
	list_for_each_entry_safe(asp, next, &dev->rms.claimable, list) {
		struct task_struct* task = asp->cmd_info ? asp->cmd_info->task : 0;

		if (!task)
			continue;

		if (task->exit_state == EXIT_DEAD) {
			list_del(&asp->list);
			list_add_tail(&asp->list, &claimable);
		}
	}
	dev->unlock_func(&dev->rms.lock, &flags);

	list_for_each_entry_safe(asp, next, &claimable, list) {
		struct queue_info* qinfo = asp->qinfo;

		if (trace_async > 2) {
			int pid;
			pid = asp->cmd_info && asp->cmd_info->task ? asp->cmd_info->task->pid : 0;

			printk(KERN_INFO "free %llx for dead task pid %d\n",
				   asp->uio.async_opaque, pid);
		}

		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		nvme_put_cmd(qinfo, asp->cmd_info);
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

		kfree(asp);
		asp = 0;
	}

	return 0;
}


/*
 * @brief Fire off an async command.
 * Even though this will return (to userspace) immediately, a timeout is
 *   allowed/encouraged to create a stronger failure-mechanism
 */
int
nvme_async_cmd_rms(struct queue_info *qinfo, struct cmd_info *cmd_info, int timeout)
{
    int result = 0;
	struct async_state* asp;
	struct usr_io2 *puio;
	struct nvme_dev* dev;
	unsigned long flags;

	dev = qinfo->dev;

#ifdef CONFIG_ASYNC_TIMEOUT
	/* add timeout... */
	cmd_info->timeout_id = dev->timeout_id;
	qinfo->timeout[cmd_info->timeout_id]++;
#endif

	if (result) {
		printk(KERN_INFO "send fails w/ nomem\n");
		return -ENOMEM;
	}
	result = 0;

	asp = kzalloc(sizeof(*asp), GFP_ATOMIC);

	/*
	 * Alas, need a copy of usr_io later, during completion...
	 */
	puio = (struct usr_io2 *) cmd_info->req;
	if (puio)
		asp->uio = *puio;
	asp->cmd_info = cmd_info;
	asp->qinfo = qinfo;

	cmd_info->req = &asp->uio;				/* under lock? */
	cmd_info->task = current;				/* TODO(craigr): BUG? get_task_struct(current)*/

	asp->evctx = eventfd_ctx_fdget(puio->eventfd);
	if (IS_ERR(asp->evctx)) {
		printk(KERN_INFO "async-cpl: pid %d eventfd %d is bad\n", current->pid, puio->eventfd);
		asp->evctx = 0;
	}

	dev->lock_func(&dev->rms.lock, &flags);
	list_add_tail(&asp->list, &dev->rms.working);
	dev->unlock_func(&dev->rms.lock, &flags);

	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	result = nvme_send_cmd(qinfo->sub_queue, cmd_info);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

	if (trace_async) {
		printk(KERN_INFO "add:\n");
		nvme_async_dump(dev);
	}

	return result;
}

/*
 * This called on cmd completion... from nvme_isr
 */
void
nvme_async_compl(struct nvme_dev* dev, struct cmd_info* cmd_info)
{
	struct async_state* asp;
	//struct queue_info *qinfo;
	unsigned long flags;

	asp = container_of(cmd_info->req, struct async_state, uio);

	/* we expect cmd_info->type is ADMIN_ASYNC_CONTEXT */
	if (trace_async)
		printk(KERN_INFO "cmd completion... opaque %llx status: %d\n", 
		   asp->uio.async_opaque, cmd_info->status);

	/* move cmd_info to claimable list */
	asp->status = cmd_info->status;
	
	dev->lock_func(&dev->rms.lock, &flags);
	list_del(&asp->list);
	list_add_tail(&asp->list, &dev->rms.claimable);
	dev->unlock_func(&dev->rms.lock, &flags);

	if (asp->evctx) {
		if (IS_ERR(asp->evctx))
			printk(KERN_INFO "bogus eventfd ptr %p\n", asp->evctx);
		else
			eventfd_signal(asp->evctx, 1);
	}
	else {
		printk(KERN_INFO "nothing to signal\n");
	}

#ifdef CONFIG_ASYNC_TIMEOUT
	/* timeout complete */
	asp->qinfo->timeout[cmd_info->timeout_id]--;
#endif

	if (trace_async)
		printk(KERN_INFO "compl::\n");

	if (trace_async > 5) {
		nvme_async_dump(dev);
	}

	nvme_unmap_user_pages(dev, &asp->uio.uio, cmd_info);
	if (trace_async > 4)
		printk(KERN_INFO "did unmap\n");
}

int
nvme_async_claim(struct nvme_dev* dev, int ncpl, struct async_cpl __user* p)
{
	struct async_state* asp, *next;
	int pid = current->pid;
	int n = 0, work = 0, oclaim = 0;
	int rc = 0;
	unsigned long flags;
	struct list_head claimable;

	INIT_LIST_HEAD(&claimable);

	/* make a local copy of candidates to claim under lock; remove
	 *   these from device list
	 */
	dev->lock_func(&dev->rms.lock, &flags);
	list_for_each_entry_safe(asp, next, &dev->rms.claimable, list) {
		/* claim/grab up to n completions */
		if (n >= ncpl)
			break;

		if (asp && asp->cmd_info && asp->cmd_info->task) {
			if (pid == asp->cmd_info->task->pid) {
				if (trace_async)
					printk(KERN_INFO "claimable: %llx statux %x\n",
						   asp->uio.async_opaque, asp->status);

				list_del(&asp->list);
				list_add_tail(&asp->list, &claimable);

				n++;
			}
			else
				oclaim++;
		}
		else {
			printk(KERN_INFO "bad asp, etc. %p\n", asp);
		}
	}

	list_for_each_entry_safe(asp, next, &dev->rms.working, list) {
		work++;
	}
	dev->unlock_func(&dev->rms.lock, &flags);

	//printk(KERN_INFO "pid 0x%x working %d oclaim %d claiming %d\n", pid, work, oclaim, n);

	/*
	 * special-case: if 'p' is 0, we clean up data structures without
	 *   attempting to copy_to_user; this is used internally for cleanup.
	 */
	n = 0;
	list_for_each_entry_safe(asp, next, &claimable, list) {
		struct queue_info* qinfo;

		//printk(KERN_INFO "claiming: %llx status %x\n",
		//	   asp->uio.async_opaque, asp->status);
		/* bull-in-a-china-shop technique...: */
		if (p) {
			if (copy_to_user((void *)&p[n].opaque, 
							 (const void*) &asp->uio.async_opaque,
							 sizeof(p[n].opaque)))
				rc |= 1;
			if (copy_to_user((void *)&p[n].status, 
							 (const void*) &asp->status, 
							 sizeof(p[n].status)))
				rc |= 1;
		}
		n++;

		qinfo = asp->qinfo;
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		nvme_put_cmd(qinfo, asp->cmd_info);
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

		kfree(asp);
		asp = 0;
	}

	if (n > ncpl)
		printk(KERN_INFO "n %d ncpl %d\n", n, ncpl);

	return rc >= 0 ? n : 0;
}

#ifdef UNUSED
/*
 * EVENTFD:
 *   fs/file.h: get_unused_fd_flags
 * user does ioctl to get cmd-event-fd... which he can then register w/ epoll.
 *   IFF allocated, we signal this fd on async cpl availability.
 *   note: calls to release may not be enough... need to see if others hold
 *     refs to the fd... and how to free when they last let go.  hm.
 */
int
nvme_async_set_eventfd(struct nvme_dev* dev, int __user* ip)
{
	int evfd;
	struct pid_entry* pe = 0;

	if (get_user(evfd, ip))
		return -EFAULT;

	pe = hash_find_pid(current->pid);
	if (pe)
		pe->evctx = eventfd_ctx_fdget(evfd);
	else
		printk(KERN_INFO "can't find pid %d\n", current->pid);

	//printk(KERN_INFO "evfd: %d %s\n", evfd, pe ? "ctx assigned" : "??");

	return 0;
}

void
nvme_async_post_cpl(struct nvme_dev* dev)
{
#if 0
	struct rms_priv* priv = &dev->rms;
	struct eventfd_ctx* ctx = 0;

	if (priv->evfd < 0)
		return;

	/* THIS IS WRONG...  evfd may be different per task */
	ctx = eventfd_ctx_fdget(priv->evfd);
	eventfd_signal(ctx, 1);
#endif
}

#endif
#endif /* CONFIG_ASYNC_CMD */

