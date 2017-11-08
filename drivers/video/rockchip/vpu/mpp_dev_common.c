/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: chenhengming chm@rock-chips.com
 *	   Alpha Lin, alpha.lin@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/rockchip/pmu.h>
#include <linux/rockchip/cru.h>

#include <video/rk_vpu_service.h>

#include "mpp_debug.h"
#include "mpp_dev_common.h"
#include "mpp_iommu_dma.h"
#include "mpp_service.h"

#define MPP_TIMEOUT_DELAY		(2 * HZ)
#define MPP_POWER_OFF_DELAY		(4 * HZ)

#ifdef CONFIG_COMPAT
struct compat_mpp_request {
	compat_uptr_t req;
	u32 size;
};
#endif

int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "bit switch for mpp device debug information");

static void *mpp_fd_to_mem_region(struct mpp_dma_session *dma, int fd)
{
	struct mpp_mem_region *mem_region = NULL;
	dma_addr_t iova;

	if (fd <= 0 || !dma)
		return ERR_PTR(-EINVAL);

	iova = mpp_dma_import_fd(dma, fd);
	if (IS_ERR_VALUE(iova)) {
		mpp_err("can't access dma-buf %d\n", fd);
		return ERR_PTR(-EINVAL);
	}

	mem_region = kzalloc(sizeof(*mem_region), GFP_KERNEL);
	if (!mem_region) {
		mpp_err("allocate memory for iommu memory region failed\n");
		mpp_dma_release_fd(dma, fd);
		return ERR_PTR(-ENOMEM);
	}

	mem_region->hdl = (void *)(long)fd;
	mem_region->iova = iova;

	return mem_region;
}

static void mpp_dev_sched_irq(struct work_struct *work)
{
	struct mpp_task *task = container_of(work, struct mpp_task, work);
	struct rockchip_mpp_dev *mpp_dev = NULL;

	mpp_dev = task->session->mpp;

	if (mpp_dev->ops->finish)
		mpp_dev->ops->finish(mpp_dev, task);

	mpp_srv_lock(mpp_dev->srv);
	/* Wake up the GET thread */
	mpp_srv_done(mpp_dev->srv);
	mpp_srv_unlock(mpp_dev->srv);
}

static void *mpp_dev_alloc_task(struct rockchip_mpp_dev *mpp_dev,
				struct mpp_session *session, void __user *src,
				u32 size)
{
	if (mpp_dev->ops->alloc_task)
		return mpp_dev->ops->alloc_task(session, src, size);
	return NULL;
}

static int mpp_dev_free_task(struct mpp_session *session, struct mpp_task *task)
{
	struct rockchip_mpp_dev *mpp_dev = session->mpp;

	if (mpp_dev->ops->free_task)
		mpp_dev->ops->free_task(session, task);
	return 0;
}

struct mpp_mem_region *mpp_dev_task_attach_fd(struct mpp_task *task, int fd)
{
	struct mpp_mem_region *mem_region = NULL;

	mem_region = mpp_fd_to_mem_region(task->session->dma, fd);
	if (IS_ERR(mem_region))
		return mem_region;

	INIT_LIST_HEAD(&mem_region->reg_lnk);
	list_add_tail(&mem_region->reg_lnk, &task->mem_region_list);

	return mem_region;
}
EXPORT_SYMBOL(mpp_dev_task_attach_fd);

int mpp_reg_address_translate(struct rockchip_mpp_dev *mpp,
			      struct mpp_task *task, int fmt, u32 *reg)
{
	struct mpp_trans_info *trans_info = mpp->variant->trans_info;
	const u8 *tbl = trans_info[fmt].table;
	int size = trans_info[fmt].count;
	int i;

	mpp_debug_enter();
	for (i = 0; i < size; i++) {
		struct mpp_mem_region *mem_region = NULL;
		int usr_fd = reg[tbl[i]] & 0x3FF;
		int offset = 0;

		mpp_debug(DEBUG_IOMMU, "reg[%03d] fd = %d\n", tbl[i], usr_fd);

		offset = reg[tbl[i]] >> 10;

		mem_region = mpp_dev_task_attach_fd(task, usr_fd);
		if (IS_ERR(mem_region))
			return PTR_ERR(mem_region);

		mem_region->reg_idx = tbl[i];
		mpp_debug(DEBUG_IOMMU, "pos: %3d fd: %3d %pad + offset %10d\n",
			  tbl[i], usr_fd, &mem_region->iova, offset);
		reg[tbl[i]] = mem_region->iova + offset;
	}

	mpp_debug_leave();

	return 0;
}
EXPORT_SYMBOL(mpp_reg_address_translate);

void mpp_translate_extra_info(struct mpp_task *task,
			      struct extra_info_for_iommu *ext_inf,
			      u32 *reg)
{
	mpp_debug_enter();
	if (ext_inf) {
		int i;

		for (i = 0; i < ext_inf->cnt; i++) {
			mpp_debug(DEBUG_IOMMU, "reg[%d] + offset %d\n",
				  ext_inf->elem[i].index,
				  ext_inf->elem[i].offset);
			reg[ext_inf->elem[i].index] += ext_inf->elem[i].offset;
		}
	}
	mpp_debug_leave();
}
EXPORT_SYMBOL(mpp_translate_extra_info);

int mpp_dev_task_init(struct mpp_session *session, struct mpp_task *task)
{
	INIT_LIST_HEAD(&task->session_link);
	INIT_LIST_HEAD(&task->status_link);
	INIT_LIST_HEAD(&task->mem_region_list);
	INIT_WORK(&task->work, mpp_dev_sched_irq);

	task->session = session;

	return 0;
}
EXPORT_SYMBOL(mpp_dev_task_init);

void mpp_dev_task_finish(struct mpp_session *session, struct mpp_task *task)
{
	struct rockchip_mpp_dev *mpp_dev = NULL;

	mpp_dev = session->mpp;
	queue_work(mpp_dev->irq_workq, &task->work);
}
EXPORT_SYMBOL(mpp_dev_task_finish);

void mpp_dev_task_finalize(struct mpp_session *session, struct mpp_task *task)
{
	struct mpp_mem_region *mem_region = NULL, *n;

	list_del_init(&task->session_link);
	list_del_init(&task->status_link);

	/* release memory region attach to this registers table. */
	list_for_each_entry_safe(mem_region, n,
				 &task->mem_region_list, reg_lnk) {
		mpp_dma_release_fd(session->dma, (long)mem_region->hdl);
		list_del_init(&mem_region->reg_lnk);
		kfree(mem_region);
	}
}
EXPORT_SYMBOL(mpp_dev_task_finalize);

static void mpp_dev_session_clear(struct rockchip_mpp_dev *mpp,
				  struct mpp_session *session)
{
	struct mpp_task *task, *n;

	/*
	 * FIXME clear all the task of this session, not only those have been
	 * done
	 */
	list_for_each_entry_safe(task, n, &session->done, session_link) {
		mpp_dev_free_task(session, task);
	}
}

static inline void mpp_queue_power_off_work(struct rockchip_mpp_dev *mpp)
{
	queue_delayed_work(system_wq, &mpp->power_off_work,
			   MPP_POWER_OFF_DELAY);
}

static void mpp_power_off_work(struct work_struct *work_s)
{
	struct delayed_work *dlwork = container_of(work_s,
						   struct delayed_work, work);
	struct rockchip_mpp_dev *mpp = container_of(dlwork,
						    struct rockchip_mpp_dev,
						    power_off_work);

	if (mutex_trylock(&mpp->srv->lock)) {
		mpp_dev_power_off(mpp);
		mutex_unlock(&mpp->srv->lock);
	} else {
		/* Come back later if the device is busy... */
		mpp_queue_power_off_work(mpp);
	}
}

static void mpp_dev_reset(struct rockchip_mpp_dev *mpp_dev)
{
	mpp_debug_enter();

	atomic_set(&mpp_dev->reset_request, 0);

	mpp_dev->ops->reset(mpp_dev);

	mpp_iommu_detach(mpp_dev->iommu_info);
	mpp_iommu_attach(mpp_dev->iommu_info);

	mpp_debug_leave();
}

void mpp_dev_power_on(struct rockchip_mpp_dev *mpp)
{
	int ret;
	ktime_t now = ktime_get();

	if (ktime_to_ns(ktime_sub(now, mpp->last)) > NSEC_PER_SEC) {
		cancel_delayed_work_sync(&mpp->power_off_work);
		mpp_queue_power_off_work(mpp);
		mpp->last = now;
	}
	ret = atomic_add_unless(&mpp->enabled, 1, 1);
	if (!ret)
		return;

	mpp->ops->power_on(mpp);
	pm_runtime_get_sync(mpp->dev);
	atomic_add(1, &mpp->power_on_cnt);
	wake_lock(&mpp->wake_lock);
}
EXPORT_SYMBOL(mpp_dev_power_on);

void mpp_dev_power_off(struct rockchip_mpp_dev *mpp)
{
	int total_running;
	int ret = atomic_add_unless(&mpp->enabled, -1, 0);

	if (!ret)
		return;

	total_running = atomic_read(&mpp->total_running);
	if (total_running) {
		pr_alert("alert: power off when %d task running!!\n",
			 total_running);
		mdelay(50);
		pr_alert("alert: delay 50 ms for running task\n");
	}

	mpp->ops->power_off(mpp);

	atomic_add(1, &mpp->power_off_cnt);
	wake_unlock(&mpp->wake_lock);
	pm_runtime_put(mpp->dev);
	pr_info("done\n");
}
EXPORT_SYMBOL(mpp_dev_power_off);

bool mpp_dev_is_power_on(struct rockchip_mpp_dev *mpp)
{
	return !!atomic_read(&mpp->enabled);
}
EXPORT_SYMBOL(mpp_dev_is_power_on);

static void rockchip_mpp_run(struct rockchip_mpp_dev *mpp_dev)
{
	struct mpp_task *task;

	mpp_debug_enter();

	/* Push pending task to running queue */
	mpp_srv_run(mpp_dev->srv);

	task = mpp_srv_get_last_running_ctx(mpp_dev->srv);
	mpp_time_record(task);

	mpp_dev_power_on(mpp_dev);

	mpp_debug(DEBUG_TASK_INFO, "pid %d, start hw %s\n",
		  task->session->pid, dev_name(mpp_dev->dev));

	if (atomic_read(&mpp_dev->reset_request))
		mpp_dev_reset(mpp_dev);

	if (unlikely(debug & DEBUG_REGISTER))
		mpp_debug_dump_reg(mpp_dev->reg_base,
				   mpp_dev->variant->reg_len);

	atomic_add(1, &mpp_dev->total_running);
	if (mpp_dev->ops->run)
		mpp_dev->ops->run(mpp_dev, task);

	mpp_debug_leave();
}

static int rockchip_mpp_result(struct rockchip_mpp_dev *mpp,
			       struct mpp_task *task, u32 __user *dst, u32 size)
{
	mpp_debug_enter();

	if (mpp->ops->result)
		mpp->ops->result(mpp, task, dst, size);

	if (mpp->ops->free_task)
		mpp->ops->free_task(task->session, task);

	mpp_debug_leave();
	return 0;
}

static void rockchip_mpp_try_run(struct rockchip_mpp_dev *mpp)
{
	int ret = 0;
	struct rockchip_mpp_dev *pending;
	struct mpp_task *task;

	mpp_debug_enter();

	if (!mpp_srv_pending_is_empty(mpp->srv)) {
		/*
		 * if prepare func in hw driver define, state will be determined
		 * by hw driver prepare func, or state will be determined by
		 * service. ret = 0, run ready ctx.
		 */
		task = mpp_srv_get_pending_ctx(mpp->srv);
		pending = task->session->mpp;
		if (mpp->ops->prepare)
			ret = mpp->ops->prepare(pending, task);
		else if (mpp_srv_is_running(mpp->srv))
			ret = -EBUSY;

		if (!ret)
			rockchip_mpp_run(pending);
	}

	mpp_debug_leave();
}

static int rockchip_mpp_wait_result(struct mpp_session *session,
				    struct rockchip_mpp_dev *mpp,
				    struct vpu_request req)
{
	struct mpp_task *task;
	int ret;

	ret = wait_event_timeout(session->wait,
				 !list_empty(&session->done),
				 MPP_TIMEOUT_DELAY);

	if (!list_empty(&session->done)) {
		if (ret < 0)
			mpp_err("warning: pid %d wait task error ret %d\n",
				session->pid, ret);
		ret = 0;
	} else {
		if (unlikely(ret < 0)) {
			mpp_err("error: pid %d wait task ret %d\n",
				session->pid, ret);
		} else if (ret == 0) {
			mpp_err("error: pid %d wait %d task done timeout\n",
				session->pid,
				atomic_read(&session->task_running));
			ret = -ETIMEDOUT;

			mpp_debug_dump_reg(mpp->reg_base,
					   mpp->variant->reg_len);
		}
	}

	/* FIXME it looks like block the system */
	if (ret < 0) {
		mpp_srv_lock(mpp->srv);
		atomic_sub(1, &mpp->total_running);

		if (mpp->ops->reset)
			mpp->ops->reset(mpp);
		mpp_srv_unlock(mpp->srv);
		return ret;
	}

	mpp_srv_lock(mpp->srv);
	task = mpp_srv_get_done_ctx(session);
	rockchip_mpp_result(mpp, task, req.req, req.size);
	mpp_srv_unlock(mpp->srv);

	return 0;
}

long mpp_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mpp_session *session = (struct mpp_session *)filp->private_data;
	struct rockchip_mpp_dev *mpp = NULL;

	mpp_debug_enter();
	if (!session)
		return -EINVAL;

	mpp = session->mpp;

	switch (cmd) {
	case VPU_IOC_SET_CLIENT_TYPE:
		break;
	case VPU_IOC_SET_REG: {
		struct vpu_request req;
		struct mpp_task *task;

		mpp_debug(DEBUG_IOCTL, "pid %d set reg\n",
			  session->pid);
		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct vpu_request))) {
			mpp_err("error: set reg copy_from_user failed\n");
			return -EFAULT;
		}
		task = mpp_dev_alloc_task(mpp, session, (void __user *)req.req,
					  req.size);
		if (!task)
			return -EFAULT;
		mpp_srv_pending_locked(mpp->srv, task);

		mpp_srv_lock(mpp->srv);
		rockchip_mpp_try_run(mpp);
		mpp_srv_unlock(mpp->srv);
	} break;
	case VPU_IOC_GET_REG: {
		struct vpu_request req;

		mpp_debug(DEBUG_IOCTL, "pid %d get reg\n",
			  session->pid);
		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct vpu_request))) {
			mpp_err("error: get reg copy_from_user failed\n");
			return -EFAULT;
		}

		return rockchip_mpp_wait_result(session, mpp, req);
	} break;
	case VPU_IOC_PROBE_IOMMU_STATUS: {
		int iommu_enable = 1;

		mpp_debug(DEBUG_IOCTL, "VPU_IOC_PROBE_IOMMU_STATUS\n");

		if (copy_to_user((void __user *)arg,
				 &iommu_enable, sizeof(int))) {
			mpp_err("error: iommu status copy_to_user failed\n");
			return -EFAULT;
		}
		break;
	}
	default: {
		dev_err(mpp->dev, "unknown mpp ioctl cmd %x\n", cmd);
		return -ENOIOCTLCMD;
	} break;
	}

	mpp_debug_leave();
	return 0;
}
EXPORT_SYMBOL(mpp_dev_ioctl);

#ifdef CONFIG_COMPAT

#define VPU_IOC_SET_CLIENT_TYPE32          _IOW(VPU_IOC_MAGIC, 1, u32)
#define VPU_IOC_GET_HW_FUSE_STATUS32       _IOW(VPU_IOC_MAGIC, 2, \
						compat_ulong_t)
#define VPU_IOC_SET_REG32                  _IOW(VPU_IOC_MAGIC, 3, \
						compat_ulong_t)
#define VPU_IOC_GET_REG32                  _IOW(VPU_IOC_MAGIC, 4, \
						compat_ulong_t)
#define VPU_IOC_PROBE_IOMMU_STATUS32       _IOR(VPU_IOC_MAGIC, 5, u32)

static long native_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -ENOIOCTLCMD;

	if (file->f_op->unlocked_ioctl)
		ret = file->f_op->unlocked_ioctl(file, cmd, arg);

	return ret;
}

long mpp_dev_compat_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct vpu_request req;
	void __user *up = compat_ptr(arg);
	int compatible_arg = 1;
	long err = 0;

	mpp_debug_enter();
	mpp_debug(DEBUG_IOCTL, "cmd %x, VPU_IOC_SET_CLIENT_TYPE32 %x\n", cmd,
		  (u32)VPU_IOC_SET_CLIENT_TYPE32);
	/* First, convert the command. */
	switch (cmd) {
	case VPU_IOC_SET_CLIENT_TYPE32:
		cmd = VPU_IOC_SET_CLIENT_TYPE;
		break;
	case VPU_IOC_GET_HW_FUSE_STATUS32:
		cmd = VPU_IOC_GET_HW_FUSE_STATUS;
		break;
	case VPU_IOC_SET_REG32:
		cmd = VPU_IOC_SET_REG;
		break;
	case VPU_IOC_GET_REG32:
		cmd = VPU_IOC_GET_REG;
		break;
	case VPU_IOC_PROBE_IOMMU_STATUS32:
		cmd = VPU_IOC_PROBE_IOMMU_STATUS;
		break;
	}
	switch (cmd) {
	case VPU_IOC_SET_REG:
	case VPU_IOC_GET_REG:
	case VPU_IOC_GET_HW_FUSE_STATUS: {
		compat_uptr_t req_ptr;
		struct compat_mpp_request __user *req32 = NULL;

		req32 = (struct compat_mpp_request __user *)up;
		memset(&req, 0, sizeof(req));

		if (get_user(req_ptr, &req32->req) ||
		    get_user(req.size, &req32->size)) {
			mpp_err("error: compat get hw status copy_from_user failed\n");
			return -EFAULT;
		}
		req.req = compat_ptr(req_ptr);
		compatible_arg = 0;
	} break;
	}

	if (compatible_arg) {
		err = native_ioctl(file, cmd, (unsigned long)up);
	} else {
		mm_segment_t old_fs = get_fs();

		set_fs(KERNEL_DS);
		err = native_ioctl(file, cmd, (unsigned long)&req);
		set_fs(old_fs);
	}

	mpp_debug_leave();
	return err;
}
EXPORT_SYMBOL(mpp_dev_compat_ioctl);
#endif

static int mpp_dev_open(struct inode *inode, struct file *filp)
{
	struct rockchip_mpp_dev *mpp = container_of(inode->i_cdev,
						    struct rockchip_mpp_dev,
						    mpp_cdev);
	struct mpp_session *session = NULL;

	mpp_debug_enter();

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	session->pid = current->pid;
	session->mpp = mpp;
	INIT_LIST_HEAD(&session->done);
	INIT_LIST_HEAD(&session->list_session);
	init_waitqueue_head(&session->wait);
	atomic_set(&session->task_running, 0);
	session->dma = mpp_dma_session_create(mpp->dev);
	mpp_srv_lock(mpp->srv);
	list_add_tail(&session->list_session, &mpp->srv->session);
	mpp_srv_unlock(mpp->srv);
	filp->private_data = (void *)session;

	mpp_debug_leave();

	return nonseekable_open(inode, filp);
}

static int mpp_dev_release(struct inode *inode, struct file *filp)
{
	struct rockchip_mpp_dev *mpp = container_of(inode->i_cdev,
						    struct rockchip_mpp_dev,
						    mpp_cdev);
	int task_running;
	struct mpp_session *session = filp->private_data;

	mpp_debug_enter();
	if (!session)
		return -EINVAL;

	task_running = atomic_read(&session->task_running);
	if (task_running) {
		pr_err("session %d still has %d task running when closing\n",
		       session->pid, task_running);
		msleep(50);
	}
	wake_up(&session->wait);

	mpp_srv_lock(mpp->srv);
	/* remove this filp from the asynchronusly notified filp's */
	list_del_init(&session->list_session);
	mpp_dev_session_clear(mpp, session);
	mpp_srv_unlock(mpp->srv);

	mpp_dma_destroy_session(session->dma);
	filp->private_data = NULL;

	kfree(session);

	dev_dbg(mpp->dev, "closed\n");
	mpp_debug_leave();
	return 0;
}

static const struct file_operations mpp_dev_default_fops = {
	.unlocked_ioctl = mpp_dev_ioctl,
	.open		= mpp_dev_open,
	.release	= mpp_dev_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = mpp_dev_compat_ioctl,
#endif
};

/* The device will do more probing work after this */
int mpp_dev_common_probe(struct rockchip_mpp_dev *mpp_dev,
			 struct platform_device *pdev,
			 struct mpp_dev_ops *ops)
{
	struct device *dev = NULL;
	struct device_node *srv_np = NULL;
	struct platform_device *srv_pdev = NULL;
	struct resource *res = NULL;
	int err;

	dev = &pdev->dev;
	/* Get and register to MPP service */
	srv_np = of_parse_phandle(pdev->dev.of_node, "rockchip,srv", 0);
	srv_pdev = of_find_device_by_node(srv_np);

	mpp_dev->srv = platform_get_drvdata(srv_pdev);
	mpp_srv_attach(mpp_dev->srv, &mpp_dev->lnk_service);

	mpp_dev->dev = dev;
	mpp_dev->state = 0;
	mpp_dev->ops = ops;

	/* TODO: change android power to linux standard */
	wake_lock_init(&mpp_dev->wake_lock, WAKE_LOCK_SUSPEND, "mpp");
	atomic_set(&mpp_dev->enabled, 0);
	atomic_set(&mpp_dev->power_on_cnt, 0);
	atomic_set(&mpp_dev->power_off_cnt, 0);
	atomic_set(&mpp_dev->total_running, 0);
	atomic_set(&mpp_dev->reset_request, 0);

	INIT_DELAYED_WORK(&mpp_dev->power_off_work, mpp_power_off_work);
	mpp_dev->last.tv64 = 0;

	pm_runtime_enable(dev);

	mpp_dev->irq_workq = create_singlethread_workqueue("mpp_irq");
	if (!mpp_dev->irq_workq) {
		dev_err(dev, "failed to create irq workqueue\n");
		err = -EINVAL;
		goto failed_irq_workq;
	}

	mpp_dev->irq = platform_get_irq(pdev, 0);
	if (mpp_dev->irq < 0) {
		dev_err(dev, "No interrupt resource found\n");
		err = -ENODEV;
		goto failed;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		err = -ENODEV;
		goto failed;
	}
	mpp_dev->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(mpp_dev->reg_base)) {
		err = PTR_ERR(mpp_dev->reg_base);
		goto failed;
	}

	/*
	 * TODO: here or at the device itself, some device doesn't
	 * have the iommu, maybe in the device is better.
	 */
	mpp_dev->iommu_info = mpp_iommu_probe(dev);
	if (IS_ERR(mpp_dev->iommu_info)) {
		dev_err(dev, "failed to attach mpp dev ret %ld\n",
			PTR_ERR(mpp_dev->iommu_info));
	}

	return 0;

#if 0
failed_free_irq:
	devm_free_irq(dev, mpp_dev->irq, (void *)mpp_dev);
#endif
failed_irq_workq:
	destroy_workqueue(mpp_dev->irq_workq);
failed:
	pm_runtime_disable(dev);
	return err;
}
EXPORT_SYMBOL(mpp_dev_common_probe);

/* Remember to set the platform data after this */
int mpp_dev_register_node(struct rockchip_mpp_dev *mpp_dev,
			  const char *node_name, const void *fops)
{
	struct device *dev = mpp_dev->dev;
	struct class *service_class = NULL;
	int ret = 0;

	/* create a device node */
	ret = alloc_chrdev_region(&mpp_dev->dev_id, 0, 1, node_name);
	if (ret) {
		dev_err(dev, "alloc dev_t failed\n");
		return ret;
	}

	if (fops)
		cdev_init(&mpp_dev->mpp_cdev, fops);
	else
		cdev_init(&mpp_dev->mpp_cdev, &mpp_dev_default_fops);
	mpp_dev->mpp_cdev.owner = THIS_MODULE;

	ret = cdev_add(&mpp_dev->mpp_cdev, mpp_dev->dev_id, 1);
	if (ret) {
		unregister_chrdev_region(mpp_dev->dev_id, 1);
		dev_err(dev, "add device node failed\n");
		return ret;
	}

	service_class = mpp_srv_get_class();
	device_create(service_class, dev, mpp_dev->dev_id, NULL, "%s",
		      node_name);

	return 0;
}
EXPORT_SYMBOL(mpp_dev_register_node);

/* Disable the Power domain after that */
int mpp_dev_common_remove(struct rockchip_mpp_dev *mpp_dev)
{
	struct class *service_class = NULL;

	destroy_workqueue(mpp_dev->irq_workq);

	mpp_srv_lock(mpp_dev->srv);
	cancel_delayed_work_sync(&mpp_dev->power_off_work);
	mpp_dev_power_off(mpp_dev);
	mpp_srv_detach(mpp_dev->srv, &mpp_dev->lnk_service);
	mpp_srv_unlock(mpp_dev->srv);

	service_class = mpp_srv_get_class();
	device_destroy(service_class, mpp_dev->dev_id);
	cdev_del(&mpp_dev->mpp_cdev);
	unregister_chrdev_region(mpp_dev->dev_id, 1);

#if 0
	pm_runtime_disable(&pdev->dev);
#endif

	return 0;
}
EXPORT_SYMBOL(mpp_dev_common_remove);

void mpp_debug_dump_reg(void __iomem *regs, int count)
{
	int i;

	pr_info("dumping registers: %p\n", regs);

	for (i = 0; i < count; i++)
		pr_info("reg[%02d]: %08x\n", i, readl_relaxed(regs + i * 4));
}
EXPORT_SYMBOL(mpp_debug_dump_reg);

void mpp_debug_dump_reg_mem(u32 *regs, int count)
{
	int i;

	pr_info("Dumping registers: %p\n", regs);

	for (i = 0; i < count; i++)
		pr_info("reg[%03d]: %08x\n", i, regs[i]);
}
EXPORT_SYMBOL(mpp_debug_dump_reg_mem);

MODULE_LICENSE("GPL v2");
