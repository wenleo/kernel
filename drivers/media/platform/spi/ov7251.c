#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define GRF_BASE_ADDR 0xff770000
#define GRF_GPIO2B_IOMUX 0x0014
#define GRF_IO_VSEL 0x0380
#define PREISP_MCLK_RATE (24 * 1000 * 1000ul)
struct ov7251_state {
	struct v4l2_subdev	sd;
	struct platform_device *pdev;
	struct mutex		lock; /*lock resource*/
	struct clk		*mclk;
	struct clk		*pd_cif;
	struct clk		*aclk_cif;
	struct clk		*hclk_cif;
	struct clk		*cif_clk_in;
	struct clk		*cif_clk_out;
	struct clk		*clk_mipi_24m;
	struct clk		*hclk_mipiphy1;
	u32 camera_nums;
	int powerdown1;
	int rst1;
	int powerdown0;
	int rst0;
	void __iomem *grf_gpio2b_iomux;
	void __iomem *grf_io_vsel;

};

static inline struct ov7251_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7251_state, sd);
}

static int ov7251_sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	unsigned int grf_val = 0;
	struct ov7251_state *pdata = to_state(sd);

	mutex_lock(&pdata->lock);
	if (on) {
		clk_prepare_enable(pdata->pd_cif);
		clk_prepare_enable(pdata->aclk_cif);
		clk_prepare_enable(pdata->hclk_cif);
		clk_prepare_enable(pdata->cif_clk_in);
		clk_prepare_enable(pdata->cif_clk_out);
		clk_prepare_enable(pdata->clk_mipi_24m);
		clk_prepare_enable(pdata->hclk_mipiphy1);
		clk_set_rate(pdata->cif_clk_out, PREISP_MCLK_RATE);

		clk_prepare_enable(pdata->mclk);
		clk_set_rate(pdata->mclk, PREISP_MCLK_RATE);

		gpio_direction_output(pdata->rst1, 0);
		mdelay(3);
		gpio_direction_output(pdata->rst1, 1);
		mdelay(5);
		gpio_direction_output(pdata->powerdown1, 1);

		gpio_direction_output(pdata->rst0, 0);
		mdelay(3);
		gpio_direction_output(pdata->rst0, 1);
		mdelay(5);
		gpio_direction_output(pdata->powerdown0, 1);

		pdata->grf_gpio2b_iomux = ioremap((resource_size_t)
						  (GRF_BASE_ADDR +
						   GRF_GPIO2B_IOMUX), 4);
		grf_val = __raw_readl(pdata->grf_gpio2b_iomux);
		__raw_writel(((grf_val) | (1 << 6) | (1 << (6 + 16))),
			     pdata->grf_gpio2b_iomux);

		pdata->grf_io_vsel = ioremap((resource_size_t)
					      (GRF_BASE_ADDR + GRF_IO_VSEL), 4);
		grf_val = __raw_readl(pdata->grf_io_vsel);
		__raw_writel(((grf_val) | (1 << 1) | (1 << (1 + 16))),
			     pdata->grf_io_vsel);
	} else if (!on) {
		grf_val = __raw_readl(pdata->grf_gpio2b_iomux);
		grf_val |= (1 << (6 + 16));
		grf_val &= (~(1 << 6));
		__raw_writel(grf_val, pdata->grf_gpio2b_iomux);
		iounmap(pdata->grf_gpio2b_iomux);

		grf_val = 0;
		grf_val = __raw_readl(pdata->grf_io_vsel);
		grf_val |= (1 << (1 + 16));
		grf_val &= (~(1 << 1));
		__raw_writel(grf_val, pdata->grf_io_vsel);
		iounmap(pdata->grf_io_vsel);

		gpio_direction_input(pdata->rst1);
		mdelay(3);
		gpio_direction_input(pdata->powerdown1);
		gpio_direction_input(pdata->rst0);
		mdelay(5);
		gpio_direction_input(pdata->powerdown0);

		clk_disable_unprepare(pdata->pd_cif);
		clk_disable_unprepare(pdata->aclk_cif);
		clk_disable_unprepare(pdata->hclk_cif);
		clk_disable_unprepare(pdata->cif_clk_in);
		clk_disable_unprepare(pdata->cif_clk_out);
		clk_disable_unprepare(pdata->clk_mipi_24m);
		clk_disable_unprepare(pdata->hclk_mipiphy1);
		clk_disable_unprepare(pdata->mclk);
	}
	mutex_unlock(&pdata->lock);

	return ret;
}

static const struct v4l2_subdev_core_ops ov7251_core_ops = {
	.s_power = ov7251_sensor_power,
};

static const struct v4l2_subdev_ops ov7251_subdev_ops = {
	.core	= &ov7251_core_ops,
};

static int ov7251_parse_dt_property(struct ov7251_state *pdata)
{
	int ret = 0;

	struct device *dev = &pdata->pdev->dev;
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;

	if (!node)
		return 1;

	pdata->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(pdata->mclk)) {
		dev_err(dev, "can not get mclk, error %ld\n",
			PTR_ERR(pdata->mclk));
		pdata->mclk = NULL;
	}
	pdata->pd_cif = devm_clk_get(dev, "pd_cif0");
	if (IS_ERR(pdata->pd_cif)) {
		dev_err(dev, "can not get pd_cif, error %ld\n",
			PTR_ERR(pdata->pd_cif));
		pdata->pd_cif = NULL;
	}
	pdata->aclk_cif = devm_clk_get(dev, "aclk_cif0");
	if (IS_ERR(pdata->aclk_cif)) {
		dev_err(dev, "can not get aclk_cif, error %ld\n",
			PTR_ERR(pdata->aclk_cif));
		pdata->aclk_cif = NULL;
	}
	pdata->hclk_cif = devm_clk_get(dev, "hclk_cif0");
	if (IS_ERR(pdata->hclk_cif)) {
		dev_warn(dev, "can not get hclk_cif, error %ld\n",
			 PTR_ERR(pdata->hclk_cif));
		pdata->hclk_cif = NULL;
	}
	pdata->cif_clk_in = devm_clk_get(dev, "cif0_in");
	if (IS_ERR(pdata->cif_clk_in)) {
		dev_err(dev, "can not get cif_clk_in, error %ld\n",
			PTR_ERR(pdata->cif_clk_in));
		pdata->cif_clk_in = NULL;
	}
	pdata->cif_clk_out = devm_clk_get(dev, "cif0_out");
	if (IS_ERR(pdata->cif_clk_out)) {
		dev_err(dev, "can not get cif_clk_out, error %ld\n",
			PTR_ERR(pdata->cif_clk_out));
		pdata->cif_clk_out = NULL;
	}
	pdata->clk_mipi_24m = devm_clk_get(dev, "clk_mipi_24m");
	if (IS_ERR(pdata->clk_mipi_24m)) {
		dev_err(dev, "can not get clk_mipi_24m, error %ld\n",
			PTR_ERR(pdata->clk_mipi_24m));
			pdata->clk_mipi_24m = NULL;
	}
	pdata->hclk_mipiphy1 = devm_clk_get(dev, "hclk_mipiphy1");
	if (IS_ERR(pdata->hclk_mipiphy1)) {
		dev_err(dev, "can not get hclk_mipiphy1, error %ld\n",
			PTR_ERR(pdata->hclk_mipiphy1));
		pdata->hclk_mipiphy1 = NULL;
	}
	pdata->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(pdata->mclk)) {
		dev_err(dev, "can not get mclk, error %ld\n",
			PTR_ERR(pdata->mclk));
		pdata->mclk = NULL;
	}
	ret = of_get_named_gpio_flags(node, "rockchip,reset1", 0, &flags);
	if (ret <= 0)
		dev_err(dev, "can not find reset1 error %d\n", ret);
	pdata->rst1 = ret;

	ret = devm_gpio_request(dev, pdata->rst1, "rockchip-reset1");
	if (ret) {
		dev_err(dev, "gpio pdata->rst1 %d request error %d\n",
			pdata->rst1, ret);
		return ret;
	}
	gpio_set_value(pdata->rst1, 0);
	ret = gpio_direction_output(pdata->rst1, 0);
	if (ret) {
		dev_err(dev, "gpio %d direction output error %d\n",
			pdata->rst1, ret);
		return ret;
	}

	ret = of_get_named_gpio_flags(node, "rockchip,powerdown1", 0, &flags);
	if (ret <= 0)
		dev_warn(dev, "can not find powerdown1, error %d\n", ret);

	pdata->powerdown1 = ret;
	ret = devm_gpio_request(dev, pdata->powerdown1, "rockchip-powerdown1");
	if (ret) {
		dev_err(dev, "gpio pdata->powerdown1 %d request error %d\n",
			pdata->powerdown1, ret);
		return ret;
	}

	ret = gpio_direction_output(pdata->powerdown1, 0);
	if (ret) {
		dev_err(dev, "gpio %d direction output error %d\n",
			pdata->powerdown1, ret);
		return ret;
	}

	ret = of_get_named_gpio_flags(node, "rockchip,reset0", 0, &flags);
	if (ret <= 0)
		dev_warn(dev,
			 "can not find property rockchip,reset0, error %d\n",
			 ret);
	pdata->rst0 = ret;
	ret = devm_gpio_request(dev, pdata->rst0, "rockchip-reset0");
	if (ret) {
		dev_err(dev, "gpio pdata->rst0 %d request error %d\n",
			pdata->rst0, ret);
		return ret;
	}
	gpio_set_value(pdata->rst0, 0);
	ret = gpio_direction_output(pdata->rst0, 0);
	if (ret) {
		dev_err(dev, "gpio0 %d direction output error %d\n",
			pdata->rst0, ret);
		return ret;
	}

	ret = of_get_named_gpio_flags(node, "rockchip,powerdown0", 0, &flags);
	if (ret <= 0)
		dev_warn(dev, "can not find powerdown0,error %d\n", ret);

	pdata->powerdown0 = ret;

	ret = devm_gpio_request(dev, pdata->powerdown0, "rockchip-powerdown0");
	if (ret) {
		dev_err(dev, "gpio pdata->powerdown0 %d request error %d\n",
			pdata->powerdown0, ret);
		return ret;
	}

	ret = gpio_direction_output(pdata->powerdown0, 0);
	if (ret) {
		dev_err(dev, "gpio0 %d direction output error %d\n",
			pdata->powerdown0, ret);
		return ret;
	}

	return ret;
}

static const struct of_device_id ov7251_match_id[] = {
	{ .compatible = "rockchip,ov7251" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov7251_match_id);

static int ov7251_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ov7251_state *ov7251;
	const struct of_device_id *of_id;
	struct v4l2_subdev *sd;
	int ret;

	ov7251 = devm_kzalloc(&pdev->dev, sizeof(*ov7251), GFP_KERNEL);
	ov7251->pdev = pdev;
	ret = ov7251_parse_dt_property(ov7251);
	if (ret)
		goto parse_err;
	mutex_init(&ov7251->lock);
	sd = &ov7251->sd;
	v4l2_subdev_init(sd, &ov7251_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "Camera-ov7251");
	sd->dev = dev;
	platform_set_drvdata(pdev, sd);
	of_id = of_match_device(ov7251_match_id, &pdev->dev);
	if (!of_id)
		goto drvdata_err;

	return 0;
drvdata_err:
	mutex_unlock(&ov7251->lock);
parse_err:
	kfree(ov7251);
	return -EINVAL;
}

static int ov7251_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd;
	struct ov7251_state *ov7251;

	sd = platform_get_drvdata(pdev);
	ov7251 = to_state(sd);
	mutex_unlock(&ov7251->lock);
	kfree(ov7251);

	return 0;
}

static struct platform_driver ov7251_driver = {
	.probe = ov7251_probe,
	.remove = ov7251_remove,
	.driver = {
			.name = "ov7251",
			.of_match_table = ov7251_match_id,
	},
};

module_platform_driver(ov7251_driver);

MODULE_AUTHOR("Wen Nuan <leo.wen@rockchips.com>");
MODULE_DESCRIPTION("A camera driver for ov7251 sensors");
MODULE_LICENSE("GPL v2");
