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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_platform.h>

#include "mpp_dev_common.h"
#include "mpp_service.h"

static struct class *mpp_service_class;

void mpp_srv_lock(struct mpp_service *pservice)
{
	mutex_lock(&pservice->lock);
}
EXPORT_SYMBOL(mpp_srv_lock);

void mpp_srv_unlock(struct mpp_service *pservice)
{
	mutex_unlock(&pservice->lock);
}
EXPORT_SYMBOL(mpp_srv_unlock);

/* service queue schedule */
void mpp_srv_pending_locked(struct mpp_service *pservice,
			    struct mpp_task *ctx)
{
	mpp_srv_lock(pservice);

	list_add_tail(&ctx->status_link, &pservice->pending);

	mpp_srv_unlock(pservice);
}
EXPORT_SYMBOL(mpp_srv_pending_locked);

void mpp_srv_run(struct mpp_service *pservice)
{
	struct mpp_task *ctx = mpp_srv_get_pending_ctx(pservice);

	list_del_init(&ctx->status_link);
	list_add_tail(&ctx->status_link, &pservice->running);
}
EXPORT_SYMBOL(mpp_srv_run);

void mpp_srv_done(struct mpp_service *pservice)
{
	struct mpp_task *ctx = list_entry(pservice->running.next,
					 struct mpp_task, status_link);

	list_del_init(&ctx->session_link);
	list_add_tail(&ctx->session_link, &ctx->session->done);

	list_del_init(&ctx->status_link);
	list_add_tail(&ctx->status_link, &pservice->done);

	wake_up(&ctx->session->wait);
}
EXPORT_SYMBOL(mpp_srv_done);

struct mpp_task *mpp_srv_get_pending_ctx(struct mpp_service *pservice)
{
	return list_entry(pservice->pending.next, struct mpp_task, status_link);
}
EXPORT_SYMBOL(mpp_srv_get_pending_ctx);

struct mpp_task *mpp_srv_get_current_ctx(struct mpp_service *pservice)
{
	return list_entry(pservice->running.next, struct mpp_task, status_link);
}
EXPORT_SYMBOL(mpp_srv_get_current_ctx);

struct mpp_task *mpp_srv_get_last_running_ctx(struct mpp_service *pservice)
{
	return list_entry(pservice->running.prev, struct mpp_task, status_link);
}
EXPORT_SYMBOL(mpp_srv_get_last_running_ctx);

struct mpp_session *mpp_srv_get_current_session(struct mpp_service *pservice)
{
	struct mpp_task *ctx = list_entry(pservice->running.next,
					 struct mpp_task, status_link);
	return ctx ? ctx->session : NULL;
}
EXPORT_SYMBOL(mpp_srv_get_current_session);

struct mpp_task *mpp_srv_get_done_ctx(struct mpp_session *session)
{
	return list_entry(session->done.next, struct mpp_task, session_link);
}
EXPORT_SYMBOL(mpp_srv_get_done_ctx);

bool mpp_srv_pending_is_empty(struct mpp_service *pservice)
{
	return !!list_empty(&pservice->pending);
}
EXPORT_SYMBOL(mpp_srv_pending_is_empty);

void mpp_srv_attach(struct mpp_service *pservice, struct list_head *elem)
{
	INIT_LIST_HEAD(elem);
	list_add_tail(elem, &pservice->subdev_list);
	pservice->dev_cnt++;
}
EXPORT_SYMBOL(mpp_srv_attach);

void mpp_srv_detach(struct mpp_service *pservice, struct list_head *elem)
{
	list_del_init(elem);
	pservice->dev_cnt--;
}
EXPORT_SYMBOL(mpp_srv_detach);

bool mpp_srv_is_running(struct mpp_service *pservice)
{
	return !list_empty(&pservice->running);
}
EXPORT_SYMBOL(mpp_srv_is_running);

static void mpp_init_drvdata(struct mpp_service *pservice)
{
	INIT_LIST_HEAD(&pservice->pending);
	mutex_init(&pservice->lock);

	INIT_LIST_HEAD(&pservice->done);
	INIT_LIST_HEAD(&pservice->session);
	INIT_LIST_HEAD(&pservice->subdev_list);
	INIT_LIST_HEAD(&pservice->running);
}

static int mpp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mpp_service *pservice = devm_kzalloc(dev, sizeof(*pservice),
						    GFP_KERNEL);
	if (!pservice)
		return -ENOMEM;

	pservice->dev = dev;

	mpp_init_drvdata(pservice);

	platform_set_drvdata(pdev, pservice);
	dev_info(dev, "init success\n");

	return 0;
}

static int mpp_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mpp_service_dt_ids[] = {
	{ .compatible = "rockchip,mpp_service", },
	{ },
};

void *mpp_srv_get_class(void)
{
	return mpp_service_class;
}
EXPORT_SYMBOL(mpp_srv_get_class);

static struct platform_driver mpp_driver = {
	.probe = mpp_probe,
	.remove = mpp_remove,
	.driver = {
		.name = "mpp",
		.of_match_table = of_match_ptr(mpp_service_dt_ids),
	},
};

static int __init mpp_service_init(void)
{
	int ret = platform_driver_register(&mpp_driver);

	if (ret) {
		pr_err("Platform device register failed (%d).\n", ret);
		return ret;
	}

	mpp_service_class = class_create(THIS_MODULE, "mpp_service");
	if (IS_ERR(mpp_service_class))
		return PTR_ERR(mpp_service_class);

	return ret;
}

static void __exit mpp_service_exit(void)
{
	class_destroy(mpp_service_class);
}

module_init(mpp_service_init);
module_exit(mpp_service_exit)
MODULE_LICENSE("GPL");
