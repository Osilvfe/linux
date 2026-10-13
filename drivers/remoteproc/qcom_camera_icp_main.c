// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Camera Image Control Processor remoteproc loader
 *
 * Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "qcom_camera_icp_hfi.h"

#define CAMERA_ICP_FIRMWARE	"qcom/sm8650/CAMERA_ICP.mbn"
#define CAMERA_ICP_PAS_ID	33
#define CAMERA_ICP_RATE		400000000UL
#define CAMERA_ICP_SLOW_AHB_RATE	80000000UL
#define CAMERA_ICP_FAST_AHB_RATE	300000000UL
#define CAMERA_ICP_CAMNOC_RATE	300000000UL
#define CAMERA_ICP_BPS_RATE		200000000UL
#define CAMERA_ICP_IPE_RATE		455000000UL
#define CAMERA_ICP_NUM_PATHS	2

enum qcom_camera_icp_clock {
	CAMERA_ICP_CLK_AHB,
	CAMERA_ICP_CLK_SRC,
	CAMERA_ICP_CLK_CORE,
	CAMERA_ICP_CLK_QDSS,
	CAMERA_ICP_CLK_GCC_AHB,
	CAMERA_ICP_CLK_GCC_AXI_HF,
	CAMERA_ICP_CLK_GCC_AXI_SF,
	CAMERA_ICP_CLK_SLOW_AHB_SRC,
	CAMERA_ICP_CLK_CPAS_AHB,
	CAMERA_ICP_CLK_CPAS_CORE_AHB,
	CAMERA_ICP_CLK_DRV_AHB,
	CAMERA_ICP_CLK_FAST_AHB_SRC,
	CAMERA_ICP_CLK_CPAS_FAST_AHB,
	CAMERA_ICP_CLK_CAMNOC_AXI_RT_SRC,
	CAMERA_ICP_CLK_CAMNOC_AXI_RT,
	CAMERA_ICP_CLK_CAMNOC_AXI_NRT,
	CAMERA_ICP_CLK_DRV_XO,
	CAMERA_ICP_CLK_PLL0,
	CAMERA_ICP_CLK_BPS_AHB,
	CAMERA_ICP_CLK_BPS_FAST_AHB,
	CAMERA_ICP_CLK_BPS_SRC,
	CAMERA_ICP_CLK_BPS,
	CAMERA_ICP_CLK_CPAS_BPS,
	CAMERA_ICP_CLK_IPE_NPS_AHB,
	CAMERA_ICP_CLK_IPE_NPS_FAST_AHB,
	CAMERA_ICP_CLK_IPE_PPS_FAST_AHB,
	CAMERA_ICP_CLK_IPE_NPS_SRC,
	CAMERA_ICP_CLK_IPE_NPS,
	CAMERA_ICP_CLK_IPE_PPS,
	CAMERA_ICP_CLK_CPAS_IPE_NPS,
	CAMERA_ICP_CLK_COUNT,
};

#define CAMERA_ICP_IRQ_MASK	0x200
#define CAMERA_ICP_IRQ_CLEAR	0x204
#define CAMERA_ICP_IRQ_STATUS	0x20c
#define CAMERA_ICP_IRQ_CMD	0x210
#define CAMERA_ICP_HOST_IRQ	BIT(2)
#define CAMERA_ICP_WDOG_BITE	BIT(4)
#define CAMERA_ICP_IRQ_CLEAR_CMD	BIT(1)

#define CAMERA_ICP_WDOG_CTRL	0x8
#define CAMERA_ICP_WDOG_CLEAR	0xc

#define CAMERA_ICP_HFI_INTERFACE		0x20
#define CAMERA_ICP_HFI_FW_VERSION	0x04
#define CAMERA_ICP_HFI_INIT_REQUEST	0x08
#define CAMERA_ICP_HFI_INIT_RESPONSE	0x0c
#define CAMERA_ICP_HFI_SHARED_PTR	0x10
#define CAMERA_ICP_HFI_SHARED_SIZE	0x14
#define CAMERA_ICP_HFI_QUEUE_TABLE_PTR	0x18
#define CAMERA_ICP_HFI_HEAP_PTR		0x1c
#define CAMERA_ICP_HFI_HEAP_SIZE		0x20
#define CAMERA_ICP_HFI_SFR_PTR		0x28
#define CAMERA_ICP_HFI_QDSS_PTR		0x2c
#define CAMERA_ICP_HFI_QDSS_SIZE		0x30
#define CAMERA_ICP_HFI_IO_PTR		0x34
#define CAMERA_ICP_HFI_IO_SIZE		0x38
#define CAMERA_ICP_HFI_IO2_PTR		0x3c
#define CAMERA_ICP_HFI_IO2_SIZE		0x40
#define CAMERA_ICP_HFI_UNCACHED_PTR	0x44
#define CAMERA_ICP_HFI_UNCACHED_SIZE	0x48
#define CAMERA_ICP_HFI_DEVICE_PTR	0x4c
#define CAMERA_ICP_HFI_DEVICE_SIZE	0x50
#define CAMERA_ICP_HFI_HOST_TO_ICP	0x300

#define CAMERA_ICP_HFI_INIT_SUCCESS	1
#define CAMERA_ICP_HFI_INIT_TIMEOUT_US	1500000
#define CAMERA_ICP_HFI_SYS_INIT_TIMEOUT	msecs_to_jiffies(5000)
#define CAMERA_ICP_HFI_PING_TIMEOUT	msecs_to_jiffies(5000)
#define CAMERA_ICP_HFI_CMD_SYS_INIT		0x00010001
#define CAMERA_ICP_HFI_CMD_SYS_SET_PROPERTY	0x00010003
#define CAMERA_ICP_HFI_CMD_SYS_PING		0x00010005
#define CAMERA_ICP_HFI_MSG_SYS_INIT_DONE		0x00020001
#define CAMERA_ICP_HFI_MSG_SYS_PING_ACK		0x00020006
#define CAMERA_ICP_HFI_PROP_SYS_MEM_REGIONS	0x0000000e
#define CAMERA_ICP_HFI_PROP_SYS_SUPPORTED	0x00000004
#define CAMERA_ICP_HFI_MAX_MSG_WORDS		128
#define CAMERA_ICP_HFI_MAX_HW_VERSIONS		6
#define CAMERA_ICP_HFI_PING_TOKEN		0x434149484f4e4701ULL

#define CAMERA_ICP_HFI_MEM_REGION_GLOBAL_SYNC	0
#define CAMERA_ICP_HFI_MEM_REGION_SYNX_MUTEX	1
#define CAMERA_ICP_HFI_MEM_REGION_IPC_MUTEX	2
#define CAMERA_ICP_HFI_MEM_REGION_GLOBAL_COUNTER	3
#define CAMERA_ICP_HFI_MEM_REGION_UNCACHED	1
#define CAMERA_ICP_HFI_MEM_REGION_DEVICE		2
#define CAMERA_ICP_HFI_MEM_REGION_COUNT		4

#define CAMERA_ICP_QDSS_IOVA		0xc0000000
#define CAMERA_ICP_QDSS_PHYS		0x16790000
#define CAMERA_ICP_QDSS_SIZE		SZ_1M
#define CAMERA_ICP_DEVICE_IOVA		0xc0100000
#define CAMERA_ICP_DEVICE_SIZE		SZ_1M
#define CAMERA_ICP_SYNX_MUTEX_PHYS	0x01f4a000
#define CAMERA_ICP_IPC_MUTEX_PHYS	0x01f4b000
#define CAMERA_ICP_GLOBAL_COUNTER_PHYS	0x0c220000
#define CAMERA_ICP_GLOBAL_SYNC_IOVA	0xc0200000
#define CAMERA_ICP_SHARED_IOVA		0xc0700000
#define CAMERA_ICP_SHARED_SIZE		0x38e00000
#define CAMERA_ICP_IO_IOVA		0xf9500000
#define CAMERA_ICP_IO_SIZE		0x06a00000
#define CAMERA_ICP_MAX_MAPPINGS		6

struct qcom_camera_icp_mapping {
	dma_addr_t iova;
	size_t size;
};

struct qcom_camera_icp_hfi_mem_region {
	__le32 id;
	__le32 type;
	__le32 start_low;
	__le32 start_high;
	__le32 size;
} __packed;

struct qcom_camera_icp_hfi_mem_regions {
	__le32 size;
	__le32 packet_type;
	__le32 num_properties;
	__le32 property;
	__le32 num_regions;
	struct qcom_camera_icp_hfi_mem_region
		region[CAMERA_ICP_HFI_MEM_REGION_COUNT];
} __packed;

struct qcom_camera_icp_hfi_ping {
	__le32 size;
	__le32 packet_type;
	__le32 user_data_low;
	__le32 user_data_high;
} __packed;

struct qcom_camera_icp {
	struct device *dev;
	struct rproc *rproc;
	struct clk_bulk_data clks[CAMERA_ICP_CLK_COUNT];
	struct icc_bulk_data paths[CAMERA_ICP_NUM_PATHS];
	struct dev_pm_domain_list *pds;
	u32 pas_id;
	int watchdog_irq;
	void __iomem *csr;
	void __iomem *watchdog;
	struct iommu_domain *domain;
	struct qcom_camera_icp_hfi hfi;
	struct qcom_camera_icp_hfi_region hfi_region;
	struct qcom_camera_icp_mapping mappings[CAMERA_ICP_MAX_MAPPINGS];
	unsigned int mapping_count;
	struct completion hfi_sys_init_done;
	struct completion hfi_ping_done;
	__le32 hfi_msg[CAMERA_ICP_HFI_MAX_MSG_WORDS];
	u32 hfi_sys_init_error;
	u64 hfi_ping_token;
	bool hfi_ready;

	void __iomem *mem_region;
	phys_addr_t mem_phys;
	phys_addr_t mem_reloc;
	size_t mem_size;
};

static int qcom_camera_icp_map(struct qcom_camera_icp *icp, dma_addr_t iova,
			       phys_addr_t phys, size_t size, int prot)
{
	int ret;

	if (icp->mapping_count >= ARRAY_SIZE(icp->mappings))
		return -E2BIG;

	ret = iommu_map(icp->domain, iova, phys, size, prot, GFP_KERNEL);
	if (ret)
		return ret;

	icp->mappings[icp->mapping_count].iova = iova;
	icp->mappings[icp->mapping_count].size = size;
	icp->mapping_count++;
	return 0;
}

static void qcom_camera_icp_unprepare_hfi(struct qcom_camera_icp *icp)
{
	while (icp->mapping_count) {
		struct qcom_camera_icp_mapping *mapping;
		size_t unmapped;

		mapping = &icp->mappings[--icp->mapping_count];
		unmapped = iommu_unmap(icp->domain, mapping->iova, mapping->size);
		if (unmapped != mapping->size)
			dev_warn(icp->dev, "only unmapped %zu of %zu bytes at %pad\n",
				 unmapped, mapping->size, &mapping->iova);
	}

	if (icp->hfi_region.base) {
		dma_free_coherent(icp->dev, icp->hfi_region.size,
				  icp->hfi_region.base, icp->hfi_region.iova);
		memset(&icp->hfi_region, 0, sizeof(icp->hfi_region));
	}
	memset(&icp->hfi, 0, sizeof(icp->hfi));
}

static int qcom_camera_icp_prepare_hfi(struct qcom_camera_icp *icp)
{
	struct resource global_sync;
	size_t hfi_size = QCOM_CAMERA_ICP_HFI_REGION_SIZE;
	int rw_prot = IOMMU_READ | IOMMU_WRITE;
	int device_prot = rw_prot | IOMMU_MMIO;
	int ret;

	icp->domain = iommu_get_domain_for_dev(icp->dev);
	if (!icp->domain)
		return dev_err_probe(icp->dev, -ENODEV,
				     "Camera ICP IOMMU domain is unavailable\n");

	ret = of_reserved_mem_region_to_resource(icp->dev->of_node, 4,
						 &global_sync);
	if (ret)
		return dev_err_probe(icp->dev, ret,
				     "failed to resolve global sync memory\n");

	icp->hfi_region.base = dma_alloc_coherent(icp->dev, hfi_size,
						  &icp->hfi_region.iova,
						  GFP_KERNEL | GFP_DMA32);
	if (!icp->hfi_region.base)
		return -ENOMEM;
	icp->hfi_region.size = QCOM_CAMERA_ICP_HFI_REGION_SIZE;
	if (icp->hfi_region.iova > U32_MAX - (hfi_size - 1)) {
		dev_err(icp->dev, "HFI DMA address %pad exceeds 32-bit range\n",
			&icp->hfi_region.iova);
		ret = -ERANGE;
		goto unprepare;
	}

	ret = qcom_camera_icp_map(icp, CAMERA_ICP_QDSS_IOVA,
				  CAMERA_ICP_QDSS_PHYS, CAMERA_ICP_QDSS_SIZE,
				  rw_prot);
	if (ret)
		goto unprepare;
	ret = qcom_camera_icp_map(icp, CAMERA_ICP_DEVICE_IOVA,
				  CAMERA_ICP_SYNX_MUTEX_PHYS, PAGE_SIZE,
				  device_prot);
	if (ret)
		goto unprepare;
	ret = qcom_camera_icp_map(icp, CAMERA_ICP_DEVICE_IOVA + PAGE_SIZE,
				  CAMERA_ICP_IPC_MUTEX_PHYS, PAGE_SIZE,
				  device_prot);
	if (ret)
		goto unprepare;
	ret = qcom_camera_icp_map(icp, CAMERA_ICP_DEVICE_IOVA + 2 * PAGE_SIZE,
				  CAMERA_ICP_GLOBAL_COUNTER_PHYS, PAGE_SIZE,
				  device_prot);
	if (ret)
		goto unprepare;
	ret = qcom_camera_icp_map(icp, CAMERA_ICP_GLOBAL_SYNC_IOVA,
				  global_sync.start, resource_size(&global_sync),
				  rw_prot);
	if (ret)
		goto unprepare;
	ret = qcom_camera_icp_hfi_region_init(&icp->hfi, &icp->hfi_region);
	if (ret)
		goto unprepare;

	return 0;

unprepare:
	qcom_camera_icp_unprepare_hfi(icp);
	return dev_err_probe(icp->dev, ret, "failed to prepare HFI memory\n");
}

static void qcom_camera_icp_write_hfi_regions(struct qcom_camera_icp *icp)
{
	void __iomem *hfi = icp->csr + CAMERA_ICP_HFI_INTERFACE;

	writel(icp->hfi_region.memory.table_iova,
	       hfi + CAMERA_ICP_HFI_QUEUE_TABLE_PTR);
	writel(icp->hfi_region.sfr_iova, hfi + CAMERA_ICP_HFI_SFR_PTR);
	writel(CAMERA_ICP_SHARED_IOVA, hfi + CAMERA_ICP_HFI_SHARED_PTR);
	writel(CAMERA_ICP_SHARED_SIZE, hfi + CAMERA_ICP_HFI_SHARED_SIZE);
	writel(icp->hfi_region.secondary_heap_iova,
	       hfi + CAMERA_ICP_HFI_HEAP_PTR);
	writel(icp->hfi_region.secondary_heap_size,
	       hfi + CAMERA_ICP_HFI_HEAP_SIZE);
	writel(CAMERA_ICP_QDSS_IOVA, hfi + CAMERA_ICP_HFI_QDSS_PTR);
	writel(CAMERA_ICP_QDSS_SIZE, hfi + CAMERA_ICP_HFI_QDSS_SIZE);
	writel(CAMERA_ICP_IO_IOVA, hfi + CAMERA_ICP_HFI_IO_PTR);
	writel(CAMERA_ICP_IO_SIZE, hfi + CAMERA_ICP_HFI_IO_SIZE);
	writel(0, hfi + CAMERA_ICP_HFI_IO2_PTR);
	writel(0, hfi + CAMERA_ICP_HFI_IO2_SIZE);
	writel(icp->hfi_region.iova, hfi + CAMERA_ICP_HFI_UNCACHED_PTR);
	writel(icp->hfi_region.size, hfi + CAMERA_ICP_HFI_UNCACHED_SIZE);
	writel(CAMERA_ICP_DEVICE_IOVA, hfi + CAMERA_ICP_HFI_DEVICE_PTR);
	writel(CAMERA_ICP_DEVICE_SIZE, hfi + CAMERA_ICP_HFI_DEVICE_SIZE);
}

static void qcom_camera_icp_set_hfi_mem_region(struct qcom_camera_icp_hfi_mem_region *region,
					       u32 id, u32 type, dma_addr_t start,
					       size_t size)
{
	region->id = cpu_to_le32(id);
	region->type = cpu_to_le32(type);
	region->start_low = cpu_to_le32(lower_32_bits(start));
	region->start_high = cpu_to_le32(upper_32_bits(start));
	region->size = cpu_to_le32(size);
}

static int qcom_camera_icp_send_memory_regions(struct qcom_camera_icp *icp)
{
	struct qcom_camera_icp_hfi_mem_regions command = {};
	int ret;

	command.size = cpu_to_le32(sizeof(command));
	command.packet_type =
		cpu_to_le32(CAMERA_ICP_HFI_CMD_SYS_SET_PROPERTY);
	command.num_properties = cpu_to_le32(1);
	command.property = cpu_to_le32(CAMERA_ICP_HFI_PROP_SYS_MEM_REGIONS);
	command.num_regions = cpu_to_le32(CAMERA_ICP_HFI_MEM_REGION_COUNT);

	qcom_camera_icp_set_hfi_mem_region(&command.region[0],
					   CAMERA_ICP_HFI_MEM_REGION_GLOBAL_SYNC,
					   CAMERA_ICP_HFI_MEM_REGION_UNCACHED,
					   CAMERA_ICP_GLOBAL_SYNC_IOVA, SZ_1M);
	qcom_camera_icp_set_hfi_mem_region(&command.region[1],
					   CAMERA_ICP_HFI_MEM_REGION_SYNX_MUTEX,
					   CAMERA_ICP_HFI_MEM_REGION_DEVICE,
					   CAMERA_ICP_DEVICE_IOVA, PAGE_SIZE);
	qcom_camera_icp_set_hfi_mem_region(&command.region[2],
					   CAMERA_ICP_HFI_MEM_REGION_IPC_MUTEX,
					   CAMERA_ICP_HFI_MEM_REGION_DEVICE,
					   CAMERA_ICP_DEVICE_IOVA + PAGE_SIZE,
					   PAGE_SIZE);
	qcom_camera_icp_set_hfi_mem_region(&command.region[3],
					   CAMERA_ICP_HFI_MEM_REGION_GLOBAL_COUNTER,
					   CAMERA_ICP_HFI_MEM_REGION_DEVICE,
					   CAMERA_ICP_DEVICE_IOVA + 2 * PAGE_SIZE,
					   PAGE_SIZE);

	ret = qcom_camera_icp_hfi_write(&icp->hfi, &command, sizeof(command));
	if (ret)
		return ret;

	writel(1, icp->csr + CAMERA_ICP_HFI_HOST_TO_ICP);
	return 0;
}

static int qcom_camera_icp_ping(struct qcom_camera_icp *icp)
{
	struct qcom_camera_icp_hfi_ping command = {};
	int ret;

	command.size = cpu_to_le32(sizeof(command));
	command.packet_type = cpu_to_le32(CAMERA_ICP_HFI_CMD_SYS_PING);
	command.user_data_low = cpu_to_le32(lower_32_bits(CAMERA_ICP_HFI_PING_TOKEN));
	command.user_data_high = cpu_to_le32(upper_32_bits(CAMERA_ICP_HFI_PING_TOKEN));

	reinit_completion(&icp->hfi_ping_done);
	WRITE_ONCE(icp->hfi_ping_token, CAMERA_ICP_HFI_PING_TOKEN);
	ret = qcom_camera_icp_hfi_write(&icp->hfi, &command, sizeof(command));
	if (ret)
		return ret;

	writel(1, icp->csr + CAMERA_ICP_HFI_HOST_TO_ICP);
	if (!wait_for_completion_timeout(&icp->hfi_ping_done,
					 CAMERA_ICP_HFI_PING_TIMEOUT)) {
		struct qcom_camera_icp_hfi_queue_header *cmd_header;
		struct qcom_camera_icp_hfi_queue_header *msg_header;
		struct qcom_camera_icp_hfi_queue_header *dbg_header;
		__le32 debug_packet[CAMERA_ICP_HFI_MAX_MSG_WORDS];
		size_t debug_size;
		const char *sfr;
		size_t sfr_len;
		int debug_ret;
		int debug_count;

		cmd_header = icp->hfi.queues[QCOM_CAMERA_ICP_HFI_CMD_QUEUE].header;
		msg_header = icp->hfi.queues[QCOM_CAMERA_ICP_HFI_MSG_QUEUE].header;
		dbg_header = icp->hfi.queues[QCOM_CAMERA_ICP_HFI_DBG_QUEUE].header;
		dev_err(icp->dev,
			"HFI ping timeout: cmd %u/%u msg %u/%u irq %#x\n",
			le32_to_cpu(READ_ONCE(cmd_header->read_index)),
			le32_to_cpu(READ_ONCE(cmd_header->write_index)),
			le32_to_cpu(READ_ONCE(msg_header->read_index)),
			le32_to_cpu(READ_ONCE(msg_header->write_index)),
			readl(icp->csr + CAMERA_ICP_IRQ_STATUS));
		dev_err(icp->dev,
			"HFI queues: cmd rx_req %u tx_req %u rx_irq %u tx_irq %u drop %u; debug %u/%u drop %u\n",
			le32_to_cpu(READ_ONCE(cmd_header->rx_request)),
			le32_to_cpu(READ_ONCE(cmd_header->tx_request)),
			le32_to_cpu(READ_ONCE(cmd_header->rx_irq_status)),
			le32_to_cpu(READ_ONCE(cmd_header->tx_irq_status)),
			le32_to_cpu(READ_ONCE(cmd_header->packet_drop_count)),
			le32_to_cpu(READ_ONCE(dbg_header->read_index)),
			le32_to_cpu(READ_ONCE(dbg_header->write_index)),
			le32_to_cpu(READ_ONCE(dbg_header->packet_drop_count)));
		sfr = (const char *)icp->hfi_region.sfr + sizeof(u32);
		sfr_len = strnlen(sfr, QCOM_CAMERA_ICP_HFI_SFR_SIZE - sizeof(u32));
		if (sfr_len)
			dev_err(icp->dev, "Camera ICP SFR: %.*s\n",
				(int)min(sfr_len, (size_t)256), sfr);
		for (debug_count = 0; debug_count < 16; debug_count++) {
			debug_ret = qcom_camera_icp_hfi_read(&icp->hfi,
					QCOM_CAMERA_ICP_HFI_DBG_QUEUE,
					debug_packet, sizeof(debug_packet),
					&debug_size);
			if (debug_ret)
				break;
			if (debug_size >= 6 * sizeof(u32))
				dev_err(icp->dev, "Camera ICP debug: type %#x: %.*s\n",
					le32_to_cpu(debug_packet[1]),
					(int)min(debug_size - 6 * sizeof(u32),
						 (size_t)256),
					(const char *)&debug_packet[6]);
		}
		if (debug_ret && debug_ret != -ENODATA)
			dev_err(icp->dev, "failed to drain HFI debug queue: %d\n",
				debug_ret);
		return -ETIMEDOUT;
	}

	return 0;
}

static int qcom_camera_icp_start_hfi(struct qcom_camera_icp *icp)
{
	void __iomem *hfi = icp->csr + CAMERA_ICP_HFI_INTERFACE;
	__le32 command[2];
	u32 response;
	u32 version;
	int ret;

	command[0] = cpu_to_le32(sizeof(command));
	command[1] = cpu_to_le32(CAMERA_ICP_HFI_CMD_SYS_INIT);
	qcom_camera_icp_write_hfi_regions(icp);
	writel(CAMERA_ICP_HFI_INIT_SUCCESS,
	       hfi + CAMERA_ICP_HFI_INIT_REQUEST);
	ret = readl_poll_timeout(hfi + CAMERA_ICP_HFI_INIT_RESPONSE, response,
				 response == CAMERA_ICP_HFI_INIT_SUCCESS,
				 1000, CAMERA_ICP_HFI_INIT_TIMEOUT_US);
	if (ret)
		return dev_err_probe(icp->dev, ret,
				     "HFI host initialization timed out: %#x\n",
				     response);

	version = readl(hfi + CAMERA_ICP_HFI_FW_VERSION);
	dev_info(icp->dev, "Camera ICP firmware version %#x\n", version);

	reinit_completion(&icp->hfi_sys_init_done);
	icp->hfi_sys_init_error = 0;
	WRITE_ONCE(icp->hfi_ready, true);
	writel(CAMERA_ICP_WDOG_BITE | CAMERA_ICP_HOST_IRQ,
	       icp->csr + CAMERA_ICP_IRQ_MASK);
	ret = qcom_camera_icp_hfi_write(&icp->hfi, command, sizeof(command));
	if (ret)
		goto disable_hfi;

	writel(1, icp->csr + CAMERA_ICP_HFI_HOST_TO_ICP);
	if (!wait_for_completion_timeout(&icp->hfi_sys_init_done,
					 CAMERA_ICP_HFI_SYS_INIT_TIMEOUT)) {
		ret = -ETIMEDOUT;
		dev_err(icp->dev, "HFI system initialization timed out\n");
		goto disable_hfi;
	}
	if (icp->hfi_sys_init_error) {
		ret = -EREMOTEIO;
		dev_err(icp->dev, "HFI system initialization failed: %#x\n",
			icp->hfi_sys_init_error);
		goto disable_hfi;
	}

	ret = qcom_camera_icp_send_memory_regions(icp);
	if (ret) {
		dev_err(icp->dev, "failed to send HFI memory regions: %d\n", ret);
		goto disable_hfi;
	}

	ret = qcom_camera_icp_ping(icp);
	if (ret) {
		dev_err(icp->dev, "HFI ping failed: %d\n", ret);
		goto disable_hfi;
	}
	dev_info(icp->dev, "Camera ICP HFI ping succeeded\n");

	return 0;

disable_hfi:
	WRITE_ONCE(icp->hfi_ready, false);
	writel(CAMERA_ICP_WDOG_BITE, icp->csr + CAMERA_ICP_IRQ_MASK);
	return ret;
}

static void qcom_camera_icp_handle_hfi_irq(struct qcom_camera_icp *icp)
{
	size_t packet_size;
	u32 packet_type;
	int ret;

	if (!READ_ONCE(icp->hfi_ready))
		return;

	for (;;) {
		ret = qcom_camera_icp_hfi_read(&icp->hfi,
					       QCOM_CAMERA_ICP_HFI_MSG_QUEUE,
					       icp->hfi_msg,
					       sizeof(icp->hfi_msg),
					       &packet_size);
		if (ret == -ENODATA)
			break;
		if (ret) {
			dev_warn_ratelimited(icp->dev,
					     "failed to read HFI message: %d\n", ret);
			break;
		}
		if (packet_size < 2 * sizeof(u32))
			continue;

		packet_type = le32_to_cpu(icp->hfi_msg[1]);
		if (packet_type == CAMERA_ICP_HFI_MSG_SYS_INIT_DONE) {
			u32 num_properties = packet_size >= 4 * sizeof(u32) ?
				le32_to_cpu(icp->hfi_msg[3]) : 0;

			icp->hfi_sys_init_error = packet_size >= 3 * sizeof(u32) ?
				le32_to_cpu(icp->hfi_msg[2]) : U32_MAX;
			if (!icp->hfi_sys_init_error && num_properties &&
			    packet_size >= 8 * sizeof(u32) &&
			    le32_to_cpu(icp->hfi_msg[4]) ==
				    CAMERA_ICP_HFI_PROP_SYS_SUPPORTED) {
				u32 num_hw = min(le32_to_cpu(icp->hfi_msg[7]),
						 CAMERA_ICP_HFI_MAX_HW_VERSIONS);

				if (packet_size >= (8 + num_hw) * sizeof(u32))
					dev_info(icp->dev,
						 "Camera ICP HFI API %#x device %#x, %u hardware blocks\n",
						 le32_to_cpu(icp->hfi_msg[5]),
						 le32_to_cpu(icp->hfi_msg[6]), num_hw);
			}
			complete(&icp->hfi_sys_init_done);
		} else if (packet_type == CAMERA_ICP_HFI_MSG_SYS_PING_ACK) {
			u64 token;

			if (packet_size < sizeof(struct qcom_camera_icp_hfi_ping)) {
				dev_warn(icp->dev, "short HFI ping response: %zu bytes\n",
					 packet_size);
				continue;
			}
			token = le32_to_cpu(icp->hfi_msg[2]);
			token |= (u64)le32_to_cpu(icp->hfi_msg[3]) << 32;
			if (token != READ_ONCE(icp->hfi_ping_token)) {
				dev_warn(icp->dev,
					 "unexpected HFI ping token: %#llx\n",
					 (unsigned long long)token);
				continue;
			}
			complete(&icp->hfi_ping_done);
		} else {
			dev_dbg_ratelimited(icp->dev,
					    "received HFI message %#x (%zu bytes)\n",
					    packet_type, packet_size);
		}
	}
}

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
	u32 status;
	int ret;

	ret = pm_runtime_resume_and_get(icp->dev);
	if (ret < 0)
		return ret;

	ret = icc_bulk_set_bw(ARRAY_SIZE(icp->paths), icp->paths);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_set_rate(icp->clks[CAMERA_ICP_CLK_SRC].clk, CAMERA_ICP_RATE);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_set_rate(icp->clks[CAMERA_ICP_CLK_SLOW_AHB_SRC].clk,
			   CAMERA_ICP_SLOW_AHB_RATE);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_set_rate(icp->clks[CAMERA_ICP_CLK_FAST_AHB_SRC].clk,
			   CAMERA_ICP_FAST_AHB_RATE);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_set_rate(icp->clks[CAMERA_ICP_CLK_CAMNOC_AXI_RT_SRC].clk,
			   CAMERA_ICP_CAMNOC_RATE);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_set_rate(icp->clks[CAMERA_ICP_CLK_BPS_SRC].clk,
			   CAMERA_ICP_BPS_RATE);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_set_rate(icp->clks[CAMERA_ICP_CLK_IPE_NPS_SRC].clk,
			   CAMERA_ICP_IPE_RATE);
	if (ret)
		goto clear_interconnect_votes;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(icp->clks), icp->clks);
	if (ret)
		goto clear_interconnect_votes;

	ret = qcom_camera_icp_prepare_hfi(icp);
	if (ret)
		goto disable_clocks;

	status = readl(icp->csr + CAMERA_ICP_IRQ_STATUS);
	if (status) {
		writel(status, icp->csr + CAMERA_ICP_IRQ_CLEAR);
		writel(CAMERA_ICP_IRQ_CLEAR_CMD, icp->csr + CAMERA_ICP_IRQ_CMD);
	}

	writel(CAMERA_ICP_WDOG_BITE, icp->csr + CAMERA_ICP_IRQ_MASK);
	enable_irq(icp->watchdog_irq);

	ret = qcom_scm_pas_auth_and_reset(icp->pas_id);
	if (ret) {
		dev_err(icp->dev, "failed to authenticate and start firmware: %d\n",
			ret);
		goto disable_watchdog;
	}

	ret = qcom_camera_icp_start_hfi(icp);
	if (ret)
		goto shutdown_firmware;

	return 0;

shutdown_firmware:
	qcom_scm_pas_shutdown(icp->pas_id);
disable_watchdog:
	writel(0, icp->csr + CAMERA_ICP_IRQ_MASK);
	disable_irq(icp->watchdog_irq);
	qcom_camera_icp_unprepare_hfi(icp);
disable_clocks:
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

	WRITE_ONCE(icp->hfi_ready, false);
	writel(0, icp->csr + CAMERA_ICP_IRQ_MASK);
	disable_irq(icp->watchdog_irq);

	ret = qcom_scm_pas_shutdown(icp->pas_id);
	if (ret)
		dev_err(icp->dev, "failed to shut down firmware: %d\n", ret);

	qcom_camera_icp_unprepare_hfi(icp);
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

static irqreturn_t qcom_camera_icp_irq(int irq, void *data)
{
	struct rproc *rproc = data;
	struct qcom_camera_icp *icp = rproc->priv;
	u32 status;

	status = readl(icp->csr + CAMERA_ICP_IRQ_STATUS);
	if (!status)
		return IRQ_NONE;

	writel(status, icp->csr + CAMERA_ICP_IRQ_CLEAR);
	writel(CAMERA_ICP_IRQ_CLEAR_CMD, icp->csr + CAMERA_ICP_IRQ_CMD);

	if (status & CAMERA_ICP_WDOG_BITE) {
		writel(0, icp->watchdog + CAMERA_ICP_WDOG_CTRL);
		writel(1, icp->watchdog + CAMERA_ICP_WDOG_CLEAR);
		rproc_report_crash(rproc, RPROC_WATCHDOG);
	}
	if (status & CAMERA_ICP_HOST_IRQ)
		qcom_camera_icp_handle_hfi_irq(icp);

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
	static const char * const pd_names[] = { "top", "bps", "ipe" };
	const struct dev_pm_domain_attach_data pd_data = {
		.pd_names = pd_names,
		.num_pd_names = ARRAY_SIZE(pd_names),
	};
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

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set 32-bit DMA mask\n");

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
	init_completion(&icp->hfi_sys_init_done);
	init_completion(&icp->hfi_ping_done);
	platform_set_drvdata(pdev, icp);

	ret = qcom_camera_icp_alloc_memory(icp);
	if (ret)
		return ret;

	icp->csr = devm_platform_ioremap_resource_byname(pdev, "csr");
	if (IS_ERR(icp->csr))
		return dev_err_probe(dev, PTR_ERR(icp->csr),
				     "failed to map CSR registers\n");

	icp->watchdog = devm_platform_ioremap_resource_byname(pdev, "watchdog");
	if (IS_ERR(icp->watchdog))
		return dev_err_probe(dev, PTR_ERR(icp->watchdog),
				     "failed to map watchdog registers\n");

	icp->clks[CAMERA_ICP_CLK_AHB].id = "icp_ahb";
	icp->clks[CAMERA_ICP_CLK_SRC].id = "icp_src";
	icp->clks[CAMERA_ICP_CLK_CORE].id = "icp";
	icp->clks[CAMERA_ICP_CLK_QDSS].id = "qdss";
	icp->clks[CAMERA_ICP_CLK_GCC_AHB].id = "gcc_ahb";
	icp->clks[CAMERA_ICP_CLK_GCC_AXI_HF].id = "gcc_axi_hf";
	icp->clks[CAMERA_ICP_CLK_GCC_AXI_SF].id = "gcc_axi_sf";
	icp->clks[CAMERA_ICP_CLK_SLOW_AHB_SRC].id = "slow_ahb_src";
	icp->clks[CAMERA_ICP_CLK_CPAS_AHB].id = "cpas_ahb";
	icp->clks[CAMERA_ICP_CLK_CPAS_CORE_AHB].id = "cpas_core_ahb";
	icp->clks[CAMERA_ICP_CLK_DRV_AHB].id = "drv_ahb";
	icp->clks[CAMERA_ICP_CLK_FAST_AHB_SRC].id = "fast_ahb_src";
	icp->clks[CAMERA_ICP_CLK_CPAS_FAST_AHB].id = "cpas_fast_ahb";
	icp->clks[CAMERA_ICP_CLK_CAMNOC_AXI_RT_SRC].id = "camnoc_axi_rt_src";
	icp->clks[CAMERA_ICP_CLK_CAMNOC_AXI_RT].id = "camnoc_axi_rt";
	icp->clks[CAMERA_ICP_CLK_CAMNOC_AXI_NRT].id = "camnoc_axi_nrt";
	icp->clks[CAMERA_ICP_CLK_DRV_XO].id = "drv_xo";
	icp->clks[CAMERA_ICP_CLK_PLL0].id = "pll0";
	icp->clks[CAMERA_ICP_CLK_BPS_AHB].id = "bps_ahb";
	icp->clks[CAMERA_ICP_CLK_BPS_FAST_AHB].id = "bps_fast_ahb";
	icp->clks[CAMERA_ICP_CLK_BPS_SRC].id = "bps_src";
	icp->clks[CAMERA_ICP_CLK_BPS].id = "bps";
	icp->clks[CAMERA_ICP_CLK_CPAS_BPS].id = "cpas_bps";
	icp->clks[CAMERA_ICP_CLK_IPE_NPS_AHB].id = "ipe_nps_ahb";
	icp->clks[CAMERA_ICP_CLK_IPE_NPS_FAST_AHB].id = "ipe_nps_fast_ahb";
	icp->clks[CAMERA_ICP_CLK_IPE_PPS_FAST_AHB].id = "ipe_pps_fast_ahb";
	icp->clks[CAMERA_ICP_CLK_IPE_NPS_SRC].id = "ipe_nps_src";
	icp->clks[CAMERA_ICP_CLK_IPE_NPS].id = "ipe_nps";
	icp->clks[CAMERA_ICP_CLK_IPE_PPS].id = "ipe_pps";
	icp->clks[CAMERA_ICP_CLK_CPAS_IPE_NPS].id = "cpas_ipe_nps";
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

	ret = devm_pm_domain_attach_list(dev, &pd_data, &icp->pds);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to attach camera power domains\n");

	irq = platform_get_irq_byname(pdev, "watchdog");
	if (irq < 0)
		return irq;

	icp->watchdog_irq = irq;
	ret = devm_request_irq(dev, irq, qcom_camera_icp_irq,
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
