// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Camera Image Control Processor remoteproc loader
 *
 * Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
 */

#include <linux/clk.h>
#include <linux/elf.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/soc/qcom/mdt_loader.h>

#define CAMERA_ICP_FIRMWARE	"qcom/sm8650/CAMERA_ICP.mbn"
#define CAMERA_ICP_PAS_ID	33
#define CAMERA_ICP_RATE		240000000UL
#define CAMERA_ICP_NUM_CLOCKS	4
#define CAMERA_ICP_NUM_PATHS	2

struct qcom_camera_icp {
	struct device *dev;
	struct rproc *rproc;
	struct clk_bulk_data clks[CAMERA_ICP_NUM_CLOCKS];
	struct icc_bulk_data paths[CAMERA_ICP_NUM_PATHS];
	u32 pas_id;
	int watchdog_irq;

	void __iomem *mem_region;
	phys_addr_t mem_phys;
	phys_addr_t mem_reloc;
	size_t mem_size;
};

static int qcom_camera_icp_load(struct rproc *rproc,
				const struct firmware *firmware)
{
	struct qcom_camera_icp *icp = rproc->priv;

	return qcom_mdt_load(icp->dev, firmware, rproc->firmware, icp->pas_id,
			     icp->mem_region, icp->mem_phys, icp->mem_size,
			     &icp->mem_reloc);
}

static int qcom_camera_icp_start(struct rproc *rproc)
{
	struct qcom_camera_icp *icp = rproc->priv;
	int ret;

	ret = pm_runtime_resume_and_get(icp->dev);
	if (ret < 0)
		return ret;

	ret = icc_bulk_set_bw(ARRAY_SIZE(icp->paths), icp->paths);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_set_rate(icp->clks[1].clk, CAMERA_ICP_RATE);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(icp->clks), icp->clks);
	if (ret)
		goto clear_interconnect_votes;

	enable_irq(icp->watchdog_irq);

	ret = qcom_scm_pas_auth_and_reset(icp->pas_id);
	if (ret) {
		dev_err(icp->dev, "failed to authenticate and start firmware: %d\n",
			ret);
		goto disable_watchdog;
	}

	return 0;

disable_watchdog:
	disable_irq(icp->watchdog_irq);
	clk_bulk_disable_unprepare(ARRAY_SIZE(icp->clks), icp->clks);
clear_interconnect_votes:
	icc_set_bw(icp->paths[1].path, 0, 0);
	icc_set_bw(icp->paths[0].path, 0, 0);
	pm_runtime_put_sync(icp->dev);
	return ret;
}

static int qcom_camera_icp_stop(struct rproc *rproc)
{
	struct qcom_camera_icp *icp = rproc->priv;
	int ret;

	disable_irq(icp->watchdog_irq);

	ret = qcom_scm_pas_shutdown(icp->pas_id);
	if (ret)
		dev_err(icp->dev, "failed to shut down firmware: %d\n", ret);

	clk_bulk_disable_unprepare(ARRAY_SIZE(icp->clks), icp->clks);
	icc_set_bw(icp->paths[1].path, 0, 0);
	icc_set_bw(icp->paths[0].path, 0, 0);
	pm_runtime_put_sync(icp->dev);

	return ret;
}

static void *qcom_camera_icp_da_to_va(struct rproc *rproc, u64 da,
				      size_t len, bool *is_iomem)
{
	struct qcom_camera_icp *icp = rproc->priv;
	u64 offset;

	if (da < icp->mem_reloc)
		return NULL;

	offset = da - icp->mem_reloc;
	if (offset > icp->mem_size || len > icp->mem_size - offset)
		return NULL;

	if (is_iomem)
		*is_iomem = true;

	return icp->mem_region + offset;
}

static const struct rproc_ops qcom_camera_icp_ops = {
	.start = qcom_camera_icp_start,
	.stop = qcom_camera_icp_stop,
	.da_to_va = qcom_camera_icp_da_to_va,
	.load = qcom_camera_icp_load,
};

static irqreturn_t qcom_camera_icp_watchdog(int irq, void *data)
{
	struct rproc *rproc = data;

	rproc_report_crash(rproc, RPROC_WATCHDOG);

	return IRQ_HANDLED;
}

static int qcom_camera_icp_alloc_memory(struct qcom_camera_icp *icp)
{
	struct resource res;
	int ret;

	ret = of_reserved_mem_region_to_resource(icp->dev->of_node, 0, &res);
	if (ret)
		return dev_err_probe(icp->dev, ret,
				     "failed to resolve memory-region\n");

	icp->mem_phys = res.start;
	icp->mem_reloc = res.start;
	icp->mem_size = resource_size(&res);
	icp->mem_region = devm_ioremap_resource_wc(icp->dev, &res);
	if (IS_ERR(icp->mem_region))
		return dev_err_probe(icp->dev, PTR_ERR(icp->mem_region),
				     "failed to map firmware memory\n");

	return 0;
}

static int qcom_camera_icp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_camera_icp *icp;
	const char *firmware_name = CAMERA_ICP_FIRMWARE;
	struct rproc *rproc;
	int irq;
	int ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	if (!qcom_scm_pas_supported(CAMERA_ICP_PAS_ID))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "PAS %u is not supported\n",
				     CAMERA_ICP_PAS_ID);

	of_property_read_string(dev->of_node, "firmware-name", &firmware_name);

	rproc = devm_rproc_alloc(dev, "camera-icp", &qcom_camera_icp_ops,
				 firmware_name, sizeof(*icp));
	if (!rproc)
		return -ENOMEM;

	rproc->auto_boot = false;
	rproc->recovery_disabled = true;
	rproc_coredump_set_elf_info(rproc, ELFCLASS32, EM_XTENSA);

	icp = rproc->priv;
	icp->dev = dev;
	icp->rproc = rproc;
	icp->pas_id = CAMERA_ICP_PAS_ID;
	platform_set_drvdata(pdev, icp);

	ret = qcom_camera_icp_alloc_memory(icp);
	if (ret)
		return ret;

	icp->clks[0].id = "icp_ahb";
	icp->clks[1].id = "icp_src";
	icp->clks[2].id = "icp";
	icp->clks[3].id = "qdss";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(icp->clks), icp->clks);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get clocks\n");

	icp->paths[0].name = "ahb";
	icp->paths[0].avg_bw = 38400;
	icp->paths[0].peak_bw = 76800;
	icp->paths[1].name = "memory";
	icp->paths[1].avg_bw = MBps_to_icc(40);
	icp->paths[1].peak_bw = MBps_to_icc(40);
	ret = devm_of_icc_bulk_get(dev, ARRAY_SIZE(icp->paths), icp->paths);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get interconnect paths\n");

	irq = platform_get_irq_byname(pdev, "watchdog");
	if (irq < 0)
		return irq;

	icp->watchdog_irq = irq;
	ret = devm_request_irq(dev, irq, qcom_camera_icp_watchdog,
			       IRQF_NO_AUTOEN,
			       dev_name(dev), rproc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request watchdog interrupt\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable runtime PM\n");

	ret = rproc_add(rproc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add remoteproc\n");

	return 0;
}

static void qcom_camera_icp_remove(struct platform_device *pdev)
{
	struct qcom_camera_icp *icp = platform_get_drvdata(pdev);

	rproc_del(icp->rproc);
}

static const struct of_device_id qcom_camera_icp_of_match[] = {
	{ .compatible = "qcom,sm8650-camera-icp" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_camera_icp_of_match);

static struct platform_driver qcom_camera_icp_driver = {
	.probe = qcom_camera_icp_probe,
	.remove = qcom_camera_icp_remove,
	.driver = {
		.name = "qcom-camera-icp",
		.of_match_table = qcom_camera_icp_of_match,
	},
};
module_platform_driver(qcom_camera_icp_driver);

MODULE_DESCRIPTION("Qualcomm Camera Image Control Processor remoteproc loader");
MODULE_LICENSE("GPL");
