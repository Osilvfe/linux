// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Camera Image Control Processor remoteproc loader
 *
 * Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/qcom_camera_icp.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "qcom_camera_icp_hfi.h"
#include "qcom_camera_icp_bps.h"

#define CAMERA_ICP_FIRMWARE	"qcom/sm8650/CAMERA_ICP.mbn"
#define CAMERA_ICP_BPS_STRIPING_FIRMWARE_FMT \
	"qcom/sm8650/bps-striping-bggr10-nv12-%ux%u.bin"
#define CAMERA_ICP_BPS_TEST_STRIPING_FIRMWARE \
	"qcom/sm8650/bps-striping-bggr10-nv12-640x480.bin"
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
#define CAMERA_ICP_HFI_CMD_IPEBPS_CREATE_HANDLE	0x01010008
#define CAMERA_ICP_HFI_CMD_IPEBPS_ASYNC_DIRECT	0x0101000a
#define CAMERA_ICP_HFI_CMD_IPEBPS_ASYNC_INDIRECT	0x0101000e
#define CAMERA_ICP_HFI_MSG_IPEBPS_CREATE_HANDLE_ACK	0x01020008
#define CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_DIRECT_ACK	0x0102000a
#define CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_INDIRECT_ACK	0x0102000e
#define CAMERA_ICP_HFI_PROP_SYS_MEM_REGIONS	0x0000000e
#define CAMERA_ICP_HFI_PROP_SYS_SUPPORTED	0x00000004
#define CAMERA_ICP_HFI_PROP_SYS_UBWC_CONFIG_EX	0x00000009
#define CAMERA_ICP_HFI_MAX_MSG_WORDS		128
#define CAMERA_ICP_HFI_MAX_HW_VERSIONS		6
#define CAMERA_ICP_HFI_PING_TOKEN		0x434149484f4e4701ULL
#define CAMERA_ICP_HFI_TEST_TOKEN_BASE		0x434149484f4e4710ULL

static bool test_bps_config;
module_param(test_bps_config, bool, 0400);
MODULE_PARM_DESC(test_bps_config,
		 "Run a bounded BPS CONFIG_IO protocol probe during ICP startup");

static unsigned int test_bps_width = 640;
module_param(test_bps_width, uint, 0400);
MODULE_PARM_DESC(test_bps_width, "BPS CONFIG_IO probe image width");

static unsigned int test_bps_height = 480;
module_param(test_bps_height, uint, 0400);
MODULE_PARM_DESC(test_bps_height, "BPS CONFIG_IO probe image height");

static bool test_bps_frame;
module_param(test_bps_frame, bool, 0400);
MODULE_PARM_DESC(test_bps_frame,
		 "Run bounded 640x480 BPS frames using an offline striping blob");

static unsigned int test_bps_frames = 1;
module_param(test_bps_frames, uint, 0400);
MODULE_PARM_DESC(test_bps_frames,
		 "Number of bounded BPS frames to process (1-16)");

static unsigned int test_bps_batch = 1;
module_param(test_bps_batch, uint, 0400);
MODULE_PARM_DESC(test_bps_batch, "BPS frame sets per request (1-2)");

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
#define CAMERA_ICP_BPS_MAX_MAPPINGS	4

struct qcom_camera_icp_mapping {
	dma_addr_t iova;
	size_t size;
};

struct qcom_camera_icp_bps_session {
	struct qcom_camera_icp *icp;
	struct qcom_camera_icp_mapping mapping[CAMERA_ICP_BPS_MAX_MAPPINGS];
	unsigned int mapping_count;
	u32 handle;
	dma_addr_t config_iova;
	bool configured;
	void *work_cpu;
	dma_addr_t work_iova;
	size_t work_size;
};

struct qcom_camera_icp_bps_buffer {
	void *cpu;
	dma_addr_t iova;
	size_t size;
	dma_addr_t bps_iova;
	size_t bps_size;
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

#define CAMERA_ICP_BPS_FRAME_CONFIG_OFFSET	0
#define CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET	SZ_4K
#define CAMERA_ICP_BPS_FRAME_GENERIC_CDM_OFFSET	0x2800
#define CAMERA_ICP_BPS_FRAME_GENERIC_CDM_SIZE	196
#define CAMERA_ICP_BPS_FRAME_IQ_OFFSET		0x3000
#define CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET	0x4000
#define CAMERA_ICP_BPS_FRAME_CDM_OFFSET		0x6000
#define CAMERA_ICP_BPS_FRAME_CDM_SIZE		SZ_512K
#define CAMERA_ICP_BPS_FRAME_INPUT_OFFSET	0x86000
#define CAMERA_ICP_BPS_FRAME_OUTPUT_Y_OFFSET	0xe8000
#define CAMERA_ICP_BPS_FRAME_OUTPUT_C_OFFSET	0x151000
#define CAMERA_ICP_BPS_FRAME_INPUT_2_OFFSET	0x186000
#define CAMERA_ICP_BPS_FRAME_OUTPUT_Y_2_OFFSET	0x1e8000
#define CAMERA_ICP_BPS_FRAME_OUTPUT_C_2_OFFSET	0x251000
#define CAMERA_ICP_BPS_FRAME_GAMMA_OFFSET	0x280000
/* Keep host DMI commands outside the firmware-owned CDM scratch region. */
#define CAMERA_ICP_BPS_FRAME_DMI_OFFSET	0x2900
#define CAMERA_ICP_BPS_FRAME_BUFFER_SIZE	0x286000
#define CAMERA_ICP_BPS_FRAME_IOVA		0x08000000
#define CAMERA_ICP_BPS_FRAME_IQ_SIZE		1276
#define CAMERA_ICP_BPS_FRAME_STRIPING_SIZE	6432

#define CAMERA_ICP_CDM_CMD_DMI			1
#define CAMERA_ICP_CDM_CMD_REG_CONT		3
#define CAMERA_ICP_BPS_WHITE_BALANCE_REG	0x3668
#define CAMERA_ICP_BPS_DEMOSAIC_REG		0x4768
#define CAMERA_ICP_BPS_COLOR_CORRECTION_REG	0x4968
#define CAMERA_ICP_BPS_GAMMA_DMI_REG		0x5008
#define CAMERA_ICP_BPS_COLOR_XFORM_REG		0x5268
#define CAMERA_ICP_BPS_GAMMA_LUT_ENTRIES	256
#define CAMERA_ICP_BPS_GAMMA_LUT_BANK_SIZE	SZ_1K
#define CAMERA_ICP_BPS_GAMMA_LUT_SIZE		(3 * SZ_1K)
#define CAMERA_ICP_BPS_GAMMA_BASE_MASK		GENMASK(11, 0)
#define CAMERA_ICP_BPS_GAMMA_DELTA_MASK		GENMASK(11, 0)
#define CAMERA_ICP_BPS_GAMMA_DELTA_SHIFT	12

#define CAMERA_ICP_BPS_GAMMA_ENABLE		0

static const u32 qcom_camera_icp_bps_demosaic_config[] = {
	0x00000000, 0x00000080, 0x01000000, 0x00000066,
};

/* Titan880 BPS WB14: green, blue and red gains followed by zero offsets. */
static const u32 qcom_camera_icp_bps_white_balance_config[] = {
	QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10,
	QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10,
	QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10,
	0, 0, 0, 0, 0, 0,
};

/* Default 257-point Qualcomm gamma15 curve from gamma_1_5_0.xsd. */
static const u16 qcom_camera_icp_bps_gamma_curve[] = {
	0, 18, 30, 42, 54, 66, 78, 92, 106, 118, 133, 147, 159, 171, 183,
	195, 207, 217, 227, 239, 249, 259, 269, 277, 287, 297, 305, 313,
	321, 329, 337, 345, 351, 357, 365, 373, 381, 388, 394, 400, 406,
	414, 420, 426, 434, 440, 444, 450, 458, 464, 468, 474, 480, 486,
	492, 496, 502, 508, 512, 518, 524, 528, 534, 538, 542, 548, 552,
	558, 564, 568, 572, 576, 580, 584, 588, 592, 598, 602, 606, 610,
	614, 618, 622, 626, 630, 632, 636, 640, 645, 649, 653, 657, 661,
	665, 669, 672, 675, 679, 681, 685, 688, 691, 695, 699, 701, 704,
	707, 711, 713, 716, 719, 723, 725, 728, 731, 735, 737, 741, 744,
	747, 749, 752, 755, 757, 760, 763, 765, 768, 771, 773, 776, 779,
	781, 784, 787, 789, 792, 795, 797, 800, 803, 805, 808, 811, 813,
	816, 819, 821, 823, 825, 827, 829, 831, 833, 836, 839, 841, 844,
	846, 848, 851, 853, 855, 857, 859, 861, 864, 866, 868, 870, 872,
	874, 876, 878, 880, 882, 884, 886, 888, 890, 892, 894, 896, 898,
	900, 903, 905, 907, 909, 911, 913, 915, 916, 918, 920, 922, 923,
	925, 927, 929, 931, 932, 934, 935, 937, 939, 941, 943, 945, 947,
	948, 950, 952, 954, 955, 957, 959, 961, 963, 964, 966, 967, 969,
	971, 972, 974, 975, 976, 978, 980, 982, 983, 985, 987, 988, 990,
	992, 994, 995, 997, 999, 1001, 1003, 1004, 1005, 1007, 1008,
	1010, 1011, 1013, 1015, 1016, 1017, 1019, 1020, 1022, 1023,
};

/* SM8650 Titan880 RGB-to-YUV matrix with full-range 12-bit clamps. */
static const u32 qcom_camera_icp_bps_color_xform_config[] = {
	0x00750259, 0x00000132, 0x00000000, 0x00000000,
	0x00000000, 0xfff00000, 0x01fe1eae, 0x00001f54,
	0x40000000, 0x00000000, 0x00000000, 0xfff00000,
	0x1fad1e55, 0x000001fe, 0x40000000, 0x00000000,
	0x00000000, 0xfff00000,
};

struct qcom_camera_icp_bps_color_profile {
	int red_gain_q10;
	int blue_gain_q10;
	int matrix_q8[3][3];
};

/* SC1320CS vendor CCM at 5100 K. */
static const struct qcom_camera_icp_bps_color_profile
qcom_camera_icp_bps_sc1320cs_color = {
	.red_gain_q10 = 2099,
	.blue_gain_q10 = 1536,
	.matrix_q8 = {
		{ 426, -122, -48 },
		{ -26, 372, -90 },
		{ 36, -181, 401 },
	},
};

/* SC820CS vendor CCM at 5200 K. */
static const struct qcom_camera_icp_bps_color_profile
qcom_camera_icp_bps_sc820cs_color = {
	.red_gain_q10 = 1587,
	.blue_gain_q10 = 1382,
	.matrix_q8 = {
		{ 500, -221, -22 },
		{ -33, 344, -54 },
		{ 34, -232, 453 },
	},
};

static u32 qcom_camera_icp_bps_pack_coeff(int high, int low)
{
	return ((u32)high & 0xfff) << 16 | ((u32)low & 0xfff);
}

static const struct qcom_camera_icp_bps_color_profile *
qcom_camera_icp_bps_color_profile_for_size(u32 width)
{
	return width >= 4000 ? &qcom_camera_icp_bps_sc1320cs_color :
			       &qcom_camera_icp_bps_sc820cs_color;
}

/* Pack a row-major CCM using the Titan880 CC131 register ordering. */
static void qcom_camera_icp_bps_color_correction_for_size(u32 width,
						   u32 *config)
{
	const struct qcom_camera_icp_bps_color_profile *profile;
	const int (*matrix)[3];

	profile = qcom_camera_icp_bps_color_profile_for_size(width);
	matrix = profile->matrix_q8;
	config[0] = qcom_camera_icp_bps_pack_coeff(matrix[2][1], matrix[1][1]);
	config[1] = (u32)matrix[0][1] & 0xfff;
	config[2] = qcom_camera_icp_bps_pack_coeff(matrix[2][2], matrix[1][2]);
	config[3] = (u32)matrix[0][2] & 0xfff;
	config[4] = qcom_camera_icp_bps_pack_coeff(matrix[2][0], matrix[1][0]);
	config[5] = (u32)matrix[0][0] & 0xfff;
	config[6] = 0;
	config[7] = 0;
	config[8] = 0;
	/* q_factor 1 selects Q8 coefficients (Q7 + q_factor). */
	config[9] = 1;
}

static __le32 *qcom_camera_icp_bps_write_reg_cont(__le32 *command, u32 reg,
						  const u32 *values,
						  size_t count)
{
	size_t i;

	*command++ = cpu_to_le32(CAMERA_ICP_CDM_CMD_REG_CONT << 24 | count);
	*command++ = cpu_to_le32(reg);
	for (i = 0; i < count; i++)
		*command++ = cpu_to_le32(values[i]);

	return command;
}

static __le32 *qcom_camera_icp_bps_write_dmi(__le32 *command, u32 reg,
					     u8 bank, dma_addr_t lut_iova)
{
	*command++ = cpu_to_le32(CAMERA_ICP_CDM_CMD_DMI << 24 |
				 CAMERA_ICP_BPS_GAMMA_LUT_BANK_SIZE - 1);
	*command++ = cpu_to_le32(lower_32_bits(lut_iova));
	*command++ = cpu_to_le32((u32)bank << 24 | reg);

	return command;
}

static void qcom_camera_icp_bps_init_gamma_lut(void *cpu)
{
	__le32 *lut = cpu;
	unsigned int channel;
	unsigned int i;

	BUILD_BUG_ON(ARRAY_SIZE(qcom_camera_icp_bps_gamma_curve) !=
		     CAMERA_ICP_BPS_GAMMA_LUT_ENTRIES + 1);
	for (channel = 0; channel < 3; channel++) {
		for (i = 0; i < CAMERA_ICP_BPS_GAMMA_LUT_ENTRIES; i++) {
			u16 base = qcom_camera_icp_bps_gamma_curve[i] << 2;
			int delta =
				(qcom_camera_icp_bps_gamma_curve[i + 1] << 2) - base;

			lut[channel * CAMERA_ICP_BPS_GAMMA_LUT_ENTRIES + i] =
				cpu_to_le32((base & CAMERA_ICP_BPS_GAMMA_BASE_MASK) |
					    ((delta & CAMERA_ICP_BPS_GAMMA_DELTA_MASK) <<
					     CAMERA_ICP_BPS_GAMMA_DELTA_SHIFT));
		}
	}
}

static __le32 *qcom_camera_icp_bps_write_white_balance(void *cpu,
						       __le32 *command,
						       u32 red_balance_q10,
						       u32 blue_balance_q10)
{
	struct qcom_camera_icp_hfi_bps_config *bps_config;
	struct qcom_camera_icp_hfi_image_desc *input;
	const struct qcom_camera_icp_bps_color_profile *profile;
	u32 config[ARRAY_SIZE(qcom_camera_icp_bps_white_balance_config)];
	u32 width;

	bps_config = cpu + CAMERA_ICP_BPS_FRAME_CONFIG_OFFSET;
	input = &bps_config->image[QCOM_CAMERA_ICP_HFI_BPS_INPUT_IMAGE];
	width = le32_to_cpu(input->info.width);
	profile = qcom_camera_icp_bps_color_profile_for_size(width);
	memcpy(config, qcom_camera_icp_bps_white_balance_config, sizeof(config));
	config[1] = DIV_ROUND_CLOSEST(profile->blue_gain_q10 * blue_balance_q10,
				      QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10);
	config[2] = DIV_ROUND_CLOSEST(profile->red_gain_q10 * red_balance_q10,
				      QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10);

	return qcom_camera_icp_bps_write_reg_cont(command,
		CAMERA_ICP_BPS_WHITE_BALANCE_REG, config, ARRAY_SIZE(config));
}

static __le32 *qcom_camera_icp_bps_write_color_correction(void *cpu,
						   __le32 *command)
{
	struct qcom_camera_icp_hfi_bps_config *bps_config;
	struct qcom_camera_icp_hfi_image_desc *input;
	u32 config[10];
	u32 width;

	bps_config = cpu + CAMERA_ICP_BPS_FRAME_CONFIG_OFFSET;
	input = &bps_config->image[QCOM_CAMERA_ICP_HFI_BPS_INPUT_IMAGE];
	width = le32_to_cpu(input->info.width);
	qcom_camera_icp_bps_color_correction_for_size(width, config);

	return qcom_camera_icp_bps_write_reg_cont(command,
		CAMERA_ICP_BPS_COLOR_CORRECTION_REG, config, ARRAY_SIZE(config));
}

static void qcom_camera_icp_init_bps_frame_settings(void *cpu,
						    dma_addr_t iova,
						    size_t gamma_offset)
{
	struct qcom_camera_icp_hfi_bps_frame_payload *payload;
	struct qcom_camera_icp_hfi_bps_cdm_program *program;
	struct qcom_camera_icp_hfi_bps_iq_settings *iq;
	__le32 *command;
	__le32 *end;
	__le32 *dmi_command;
	__le32 *dmi_end;
	u32 unit = QCOM_CAMERA_ICP_BPS_BALANCE_UNIT_Q10;
	size_t generic_size;
	size_t dmi_size;

	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_iq_settings) !=
		     CAMERA_ICP_BPS_FRAME_IQ_SIZE);
	BUILD_BUG_ON(offsetof(struct qcom_camera_icp_hfi_bps_iq_settings,
			      demosaic_enable) != 0x1ec);
	BUILD_BUG_ON(offsetof(struct qcom_camera_icp_hfi_bps_iq_settings,
			      color_correction_enable) != 0x28c);
	BUILD_BUG_ON(offsetof(struct qcom_camera_icp_hfi_bps_iq_settings,
			      gamma_enable) != 0x294);
	BUILD_BUG_ON(offsetof(struct qcom_camera_icp_hfi_bps_iq_settings,
			      color_xform_enable) != 0x298);
	BUILD_BUG_ON(offsetof(struct qcom_camera_icp_hfi_bps_iq_settings,
			      chroma_horizontal_rounding) != 0x29c);
	BUILD_BUG_ON(offsetof(struct qcom_camera_icp_hfi_bps_iq_settings,
			      full_luma_clamp_min) != 0x308);
	BUILD_BUG_ON(offsetof(struct qcom_camera_icp_hfi_bps_iq_settings,
			      white_balance_enable) != 0x3dc);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET +
		     sizeof(struct qcom_camera_icp_hfi_bps_frame_payload) >
		     CAMERA_ICP_BPS_FRAME_GENERIC_CDM_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_GENERIC_CDM_OFFSET +
		     CAMERA_ICP_BPS_FRAME_GENERIC_CDM_SIZE >
		     CAMERA_ICP_BPS_FRAME_DMI_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_DMI_OFFSET + 3 * 3 * sizeof(*command) >
		     CAMERA_ICP_BPS_FRAME_IQ_OFFSET);
	BUILD_BUG_ON((2 + ARRAY_SIZE(qcom_camera_icp_bps_white_balance_config) +
		      2 + ARRAY_SIZE(qcom_camera_icp_bps_demosaic_config) +
		      2 + 10 +
		      2 + ARRAY_SIZE(qcom_camera_icp_bps_color_xform_config)) *
		     sizeof(*command) != CAMERA_ICP_BPS_FRAME_GENERIC_CDM_SIZE);

	payload = cpu + CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET;
	iq = cpu + CAMERA_ICP_BPS_FRAME_IQ_OFFSET;
	command = cpu + CAMERA_ICP_BPS_FRAME_GENERIC_CDM_OFFSET;

	iq->white_balance_enable = cpu_to_le32(1);
	iq->demosaic_enable = cpu_to_le32(1);
	iq->color_correction_enable = cpu_to_le32(1);
	iq->gamma_enable = cpu_to_le32(CAMERA_ICP_BPS_GAMMA_ENABLE);
	iq->color_xform_enable = cpu_to_le32(1);
	iq->chroma_horizontal_rounding = cpu_to_le32(1);
	iq->chroma_vertical_rounding = cpu_to_le32(2);
	/* NV12 is an 8-bit output, so both full-path channels clamp to 255. */
	iq->full_luma_clamp_max = cpu_to_le32(U8_MAX);
	iq->full_chroma_clamp_max = cpu_to_le32(U8_MAX);

	end = qcom_camera_icp_bps_write_white_balance(cpu, command, unit, unit);
	end = qcom_camera_icp_bps_write_reg_cont(end, CAMERA_ICP_BPS_DEMOSAIC_REG,
						 qcom_camera_icp_bps_demosaic_config,
						 ARRAY_SIZE(qcom_camera_icp_bps_demosaic_config));
	end = qcom_camera_icp_bps_write_color_correction(cpu, end);
	end = qcom_camera_icp_bps_write_reg_cont(end,
						 CAMERA_ICP_BPS_COLOR_XFORM_REG,
						 qcom_camera_icp_bps_color_xform_config,
						 ARRAY_SIZE(qcom_camera_icp_bps_color_xform_config));
	qcom_camera_icp_bps_init_gamma_lut(cpu + gamma_offset);
	generic_size = (u8 *)end - (u8 *)command;
	dmi_command = cpu + CAMERA_ICP_BPS_FRAME_DMI_OFFSET;
	dmi_end = qcom_camera_icp_bps_write_dmi(dmi_command,
						CAMERA_ICP_BPS_GAMMA_DMI_REG, 1,
						iova + gamma_offset);
	dmi_end = qcom_camera_icp_bps_write_dmi(dmi_end,
						CAMERA_ICP_BPS_GAMMA_DMI_REG, 2,
						iova + gamma_offset +
						CAMERA_ICP_BPS_GAMMA_LUT_BANK_SIZE);
	dmi_end = qcom_camera_icp_bps_write_dmi(dmi_end,
						CAMERA_ICP_BPS_GAMMA_DMI_REG, 3,
						iova + gamma_offset +
						2 * CAMERA_ICP_BPS_GAMMA_LUT_BANK_SIZE);
	dmi_size = (u8 *)dmi_end - (u8 *)dmi_command;

	payload->cdm_allocator = 0;
	payload->cdm_num_programs = cpu_to_le32(CAMERA_ICP_BPS_GAMMA_ENABLE ? 2 : 1);
	program = &payload->program[0];
	program->base = cpu_to_le32(lower_32_bits(iova +
				 CAMERA_ICP_BPS_FRAME_GENERIC_CDM_OFFSET));
	program->length = cpu_to_le32((generic_size - 1) <<
				      QCOM_CAMERA_ICP_HFI_BPS_CDM_LENGTH_SHIFT);
	program->type = cpu_to_le32(QCOM_CAMERA_ICP_HFI_BPS_PROGRAM_GENERIC);
	program->uid = 0;
	program->single_register = 0;
	program->internally_allocated = 0;
	program = &payload->program[1];
	program->base = cpu_to_le32(lower_32_bits(iova +
				 CAMERA_ICP_BPS_FRAME_DMI_OFFSET));
	program->length = cpu_to_le32((dmi_size - 1) <<
				      QCOM_CAMERA_ICP_HFI_BPS_CDM_LENGTH_SHIFT);
	program->type = cpu_to_le32(QCOM_CAMERA_ICP_HFI_BPS_PROGRAM_DMI);
	program->uid = 0;
	program->single_register = 0;
	program->internally_allocated = 0;
}

struct qcom_camera_icp {
	struct device *dev;
	struct rproc *rproc;
	struct auxiliary_device *bps_auxdev;
	struct qcom_camera_icp_bps bps;
	/* BPS recovery stops ICP, so only one client session may own it. */
	struct mutex bps_client_lock;
	bool bps_client_active;
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
	struct completion hfi_command_done;
	/* Serializes the single firmware command/response transaction slot. */
	struct mutex hfi_command_lock;
	__le32 hfi_msg[CAMERA_ICP_HFI_MAX_MSG_WORDS];
	u32 hfi_sys_init_error;
	u64 hfi_ping_token;
	u64 hfi_command_token;
	u32 hfi_command_ack;
	u32 hfi_command_opcode;
	u32 hfi_command_error;
	u32 hfi_command_result;
	u32 hfi_command_handle;
	struct qcom_camera_icp_bps_buffer bps_test_buffer;
	bool hfi_ready;

	void __iomem *mem_region;
	phys_addr_t mem_phys;
	phys_addr_t mem_reloc;
	size_t mem_size;
};

static void qcom_camera_icp_init_bps_config(
	struct qcom_camera_icp *icp,
	const struct qcom_camera_icp_bps_config *client_config,
	struct qcom_camera_icp_hfi_bps_config *config);

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

static void qcom_camera_icp_bps_buffer_unmap(struct qcom_camera_icp *icp,
				     struct qcom_camera_icp_bps_buffer *buffer)
{
	size_t unmapped;

	if (!buffer->bps_size)
		return;

	unmapped = iommu_unmap(icp->domain, buffer->bps_iova, buffer->bps_size);
	if (unmapped != buffer->bps_size)
		dev_warn(icp->dev, "only unmapped %zu of %zu BPS bytes at %pad\n",
			 unmapped, buffer->bps_size, &buffer->bps_iova);

	buffer->bps_iova = 0;
	buffer->bps_size = 0;
}

static int qcom_camera_icp_bps_buffer_map(struct qcom_camera_icp *icp,
				   struct qcom_camera_icp_bps_buffer *buffer,
				   dma_addr_t bps_iova)
{
	dma_addr_t source_iova = buffer->iova;
	size_t mapped = 0;
	phys_addr_t phys;
	int ret;

	if (!IS_ALIGNED(source_iova, PAGE_SIZE) ||
	    !IS_ALIGNED(bps_iova, PAGE_SIZE) ||
	    !IS_ALIGNED(buffer->size, PAGE_SIZE) || buffer->bps_size ||
	    bps_iova > U32_MAX - (buffer->size - 1))
		return -EINVAL;

	while (mapped < buffer->size) {
		phys = iommu_iova_to_phys(icp->domain, source_iova + mapped);
		if (!phys) {
			ret = -EFAULT;
			goto unmap;
		}
		if (!IS_ALIGNED(phys, PAGE_SIZE)) {
			ret = -EINVAL;
			goto unmap;
		}

		ret = iommu_map(icp->domain, bps_iova + mapped,
				phys, PAGE_SIZE,
				IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE,
				GFP_KERNEL);
		if (ret)
			goto unmap;
		mapped += PAGE_SIZE;
	}

	buffer->bps_iova = bps_iova;
	buffer->bps_size = mapped;
	return 0;

unmap:
	if (mapped)
		iommu_unmap(icp->domain, bps_iova, mapped);
	return ret;
}

static int qcom_camera_icp_bps_buffer_alloc(struct qcom_camera_icp *icp,
				     struct qcom_camera_icp_bps_buffer *buffer,
				     size_t size)
{
	if (!size || !IS_ALIGNED(size, PAGE_SIZE) || buffer->cpu)
		return -EINVAL;

	buffer->cpu = alloc_pages_exact(size, GFP_KERNEL | GFP_DMA32 | __GFP_ZERO);
	if (!buffer->cpu)
		return -ENOMEM;
	buffer->size = size;
	buffer->iova = dma_map_single(icp->dev, buffer->cpu, size,
				      DMA_BIDIRECTIONAL);
	if (dma_mapping_error(icp->dev, buffer->iova)) {
		free_pages_exact(buffer->cpu, size);
		memset(buffer, 0, sizeof(*buffer));
		return -EIO;
	}

	if (buffer->iova > U32_MAX - (size - 1)) {
		dma_unmap_single(icp->dev, buffer->iova, size,
				 DMA_BIDIRECTIONAL);
		free_pages_exact(buffer->cpu, size);
		memset(buffer, 0, sizeof(*buffer));
		return -ERANGE;
	}

	return 0;
}

static void qcom_camera_icp_bps_buffer_free(struct qcom_camera_icp *icp,
				      struct qcom_camera_icp_bps_buffer *buffer)
{
	qcom_camera_icp_bps_buffer_unmap(icp, buffer);
	if (buffer->cpu) {
		dma_unmap_single(icp->dev, buffer->iova, buffer->size,
				 DMA_BIDIRECTIONAL);
		free_pages_exact(buffer->cpu, buffer->size);
	}
	memset(buffer, 0, sizeof(*buffer));
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
	qcom_camera_icp_bps_buffer_free(icp, &icp->bps_test_buffer);

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

static int qcom_camera_icp_configure_ubwc(struct qcom_camera_icp *icp)
{
	u32 config[4];
	__le32 command[10] = {
		cpu_to_le32(sizeof(command)),
		cpu_to_le32(CAMERA_ICP_HFI_CMD_SYS_SET_PROPERTY),
		cpu_to_le32(1),
		cpu_to_le32(CAMERA_ICP_HFI_PROP_SYS_UBWC_CONFIG_EX),
	};
	int ret;

	if (!of_property_present(icp->dev->of_node, "qcom,ubwc-configuration"))
		return 0;

	ret = of_property_read_u32_array(icp->dev->of_node,
					 "qcom,ubwc-configuration", config,
					 ARRAY_SIZE(config));
	if (ret)
		return ret;

	command[4] = cpu_to_le32(config[0]);
	command[5] = cpu_to_le32(config[1]);
	command[6] = cpu_to_le32(config[2]);
	command[7] = cpu_to_le32(config[3]);
	ret = qcom_camera_icp_hfi_write(&icp->hfi, command, sizeof(command));
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
		struct qcom_camera_icp_hfi *hfi = &icp->hfi;
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
			debug_ret = qcom_camera_icp_hfi_read(hfi,
							     QCOM_CAMERA_ICP_HFI_DBG_QUEUE,
							     debug_packet,
							     sizeof(debug_packet),
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

static int qcom_camera_icp_send_sync(struct qcom_camera_icp *icp,
				     const void *command, size_t command_size,
				     u32 ack, u32 opcode, u64 token)
{
	int ret;

	mutex_lock(&icp->hfi_command_lock);
	reinit_completion(&icp->hfi_command_done);
	WRITE_ONCE(icp->hfi_command_ack, ack);
	WRITE_ONCE(icp->hfi_command_opcode, opcode);
	WRITE_ONCE(icp->hfi_command_token, token);
	WRITE_ONCE(icp->hfi_command_error, U32_MAX);
	WRITE_ONCE(icp->hfi_command_result, U32_MAX);
	ret = qcom_camera_icp_hfi_write(&icp->hfi, command, command_size);
	if (ret)
		goto unlock;

	writel(1, icp->csr + CAMERA_ICP_HFI_HOST_TO_ICP);
	if (!wait_for_completion_timeout(&icp->hfi_command_done,
					 CAMERA_ICP_HFI_PING_TIMEOUT)) {
		dev_err(icp->dev, "HFI command %#x/%#x timed out\n",
			ack, opcode);
		ret = -ETIMEDOUT;
		goto unlock;
	}

	if (READ_ONCE(icp->hfi_command_error) ||
	    READ_ONCE(icp->hfi_command_result)) {
		dev_err(icp->dev,
			"HFI command %#x/%#x failed: transport %#x result %#x\n",
			ack, opcode, READ_ONCE(icp->hfi_command_error),
			READ_ONCE(icp->hfi_command_result));
		ret = -EREMOTEIO;
		goto unlock;
	}

unlock:
	mutex_unlock(&icp->hfi_command_lock);
	return ret;
}

static int
qcom_camera_icp_bps_session_create(struct qcom_camera_icp_bps_session *session)
{
	struct qcom_camera_icp *icp = session->icp;
	const u64 token = CAMERA_ICP_HFI_TEST_TOKEN_BASE;
	struct qcom_camera_icp_hfi_create_handle command = {
		.size = cpu_to_le32(sizeof(command)),
		.packet_type =
			cpu_to_le32(CAMERA_ICP_HFI_CMD_IPEBPS_CREATE_HANDLE),
		.handle_type = cpu_to_le32(QCOM_CAMERA_ICP_HFI_BPS_NON_RT),
		.user_data1 = cpu_to_le64(token),
	};
	int ret;

	WRITE_ONCE(icp->hfi_command_handle, 0);
	ret = qcom_camera_icp_send_sync(icp, &command, sizeof(command),
					CAMERA_ICP_HFI_MSG_IPEBPS_CREATE_HANDLE_ACK,
					0, token);
	if (ret)
		return ret;
	session->handle = READ_ONCE(icp->hfi_command_handle);
	if (!session->handle)
		return -EREMOTEIO;
	session->configured = false;

	dev_info(icp->dev, "Camera ICP BPS handle %#x created\n",
		 session->handle);
	return 0;
}

static int
qcom_camera_icp_bps_session_open(struct qcom_camera_icp *icp,
				 struct qcom_camera_icp_bps_session *session)
{
	memset(session, 0, sizeof(*session));
	session->icp = icp;

	return qcom_camera_icp_bps_session_create(session);
}

static int
qcom_camera_icp_bps_session_map(struct qcom_camera_icp_bps_session *session,
				dma_addr_t iova, size_t size, bool map)
{
	struct qcom_camera_icp *icp = session->icp;
	struct qcom_camera_icp_mapping *mapping;
	const u32 opcode = map ? QCOM_CAMERA_ICP_HFI_MEM_MAP :
				 QCOM_CAMERA_ICP_HFI_MEM_UNMAP;
	const u64 token = CAMERA_ICP_HFI_TEST_TOKEN_BASE + opcode;
	struct qcom_camera_icp_hfi_async_map command = {
		.size = cpu_to_le32(sizeof(command)),
		.packet_type =
			cpu_to_le32(CAMERA_ICP_HFI_CMD_IPEBPS_ASYNC_DIRECT),
		.opcode = cpu_to_le32(opcode),
		.user_data1 = cpu_to_le64(token),
		.num_fw_handles = cpu_to_le32(1),
		.fw_handle = cpu_to_le32(session->handle),
		.map_user_data = cpu_to_le64(token),
		.num_regions = cpu_to_le32(1),
		.region_iova = cpu_to_le32(iova),
		.region_size = cpu_to_le32(size),
	};
	unsigned int i;
	int ret;

	if (!session->handle || !size)
		return -EINVAL;
	ret = qcom_camera_icp_bps_validate_region(iova, size);
	if (ret)
		return ret;

	for (i = 0; i < session->mapping_count; i++) {
		mapping = &session->mapping[i];
		if (mapping->iova == iova && mapping->size == size)
			break;
	}
	if (map && i != session->mapping_count)
		return -EEXIST;
	if (map && session->mapping_count == ARRAY_SIZE(session->mapping))
		return -ENOSPC;
	if (!map && i == session->mapping_count)
		return -ENOENT;

	ret = qcom_camera_icp_send_sync(icp, &command, sizeof(command),
					CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_DIRECT_ACK,
					opcode, token);
	if (ret)
		return ret;

	if (map) {
		mapping = &session->mapping[session->mapping_count++];
		mapping->iova = iova;
		mapping->size = size;
	} else {
		if (session->configured &&
		    qcom_camera_icp_bps_region_contains(iova, size,
						 session->config_iova,
						 sizeof(struct qcom_camera_icp_hfi_bps_config))) {
			session->configured = false;
			session->config_iova = 0;
		}
		session->mapping_count--;
		if (i != session->mapping_count)
			session->mapping[i] =
				session->mapping[session->mapping_count];
	}

	return 0;
}

static int
qcom_camera_icp_bps_session_unmap_all(struct qcom_camera_icp_bps_session *session)
{
	struct qcom_camera_icp_mapping *mapping;
	int ret;

	while (session->mapping_count) {
		mapping = &session->mapping[session->mapping_count - 1];
		ret = qcom_camera_icp_bps_session_map(session, mapping->iova,
					      mapping->size, false);
		if (ret)
			return ret;
	}

	return 0;
}

static int
qcom_camera_icp_bps_session_map_buffer(struct qcom_camera_icp_bps_session *session,
				       struct qcom_camera_icp_bps_buffer *buffer)
{
	int cleanup_ret;
	int ret;

	if (!buffer->cpu || !buffer->size)
		return -EINVAL;
	if (buffer->bps_size && buffer->bps_size != buffer->size)
		return -EINVAL;

	ret = qcom_camera_icp_bps_session_map(session, buffer->iova,
					      buffer->size, true);
	if (ret || !buffer->bps_size || buffer->bps_iova == buffer->iova)
		return ret;

	ret = qcom_camera_icp_bps_session_map(session, buffer->bps_iova,
					      buffer->bps_size, true);
	if (!ret)
		return 0;

	cleanup_ret = qcom_camera_icp_bps_session_map(session, buffer->iova,
						      buffer->size, false);
	return cleanup_ret ?: ret;
}

static bool
qcom_camera_icp_bps_session_contains(struct qcom_camera_icp_bps_session *session,
				     dma_addr_t iova, size_t size)
{
	struct qcom_camera_icp_mapping *mapping;
	unsigned int i;

	for (i = 0; i < session->mapping_count; i++) {
		mapping = &session->mapping[i];
		if (qcom_camera_icp_bps_region_contains(mapping->iova,
						mapping->size, iova, size))
			return true;
	}

	return false;
}

static int
qcom_camera_icp_bps_session_config(struct qcom_camera_icp_bps_session *session,
				   dma_addr_t payload_iova)
{
	struct qcom_camera_icp *icp = session->icp;
	const u64 token = CAMERA_ICP_HFI_TEST_TOKEN_BASE +
			  QCOM_CAMERA_ICP_HFI_BPS_CONFIG_IO;
	struct qcom_camera_icp_hfi_async_indirect command = {
		.size = cpu_to_le32(sizeof(command)),
		.packet_type =
			cpu_to_le32(CAMERA_ICP_HFI_CMD_IPEBPS_ASYNC_INDIRECT),
		.opcode = cpu_to_le32(QCOM_CAMERA_ICP_HFI_BPS_CONFIG_IO),
		.user_data1 = cpu_to_le64(token),
		.num_fw_handles = cpu_to_le32(1),
		.fw_handle = cpu_to_le32(session->handle),
		.payload_iova = cpu_to_le32(payload_iova),
	};

	int ret;

	if (!session->handle)
		return -EINVAL;
	if (payload_iova > U32_MAX)
		return -ERANGE;
	if (!qcom_camera_icp_bps_session_contains(session, payload_iova,
						   sizeof(struct qcom_camera_icp_hfi_bps_config)))
		return -ENOENT;

	ret = qcom_camera_icp_send_sync(icp, &command, sizeof(command),
			CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_INDIRECT_ACK,
			QCOM_CAMERA_ICP_HFI_BPS_CONFIG_IO, token);
	if (!ret) {
		session->configured = true;
		session->config_iova = payload_iova;
	}

	return ret;
}

static int
qcom_camera_icp_bps_session_submit(struct qcom_camera_icp_bps_session *session,
				   dma_addr_t payload_iova, u64 request_id)
{
	struct qcom_camera_icp_hfi_async_indirect command = {
		.size = cpu_to_le32(sizeof(command)),
		.packet_type =
			cpu_to_le32(CAMERA_ICP_HFI_CMD_IPEBPS_ASYNC_INDIRECT),
		.opcode = cpu_to_le32(QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS),
		.user_data1 = cpu_to_le64(CAMERA_ICP_HFI_TEST_TOKEN_BASE +
					   QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS),
		.user_data2 = cpu_to_le64(request_id),
		.num_fw_handles = cpu_to_le32(1),
		.fw_handle = cpu_to_le32(session->handle),
		.payload_iova = cpu_to_le32(payload_iova),
	};

	if (!session->handle || !session->configured)
		return -EINVAL;
	if (payload_iova > U32_MAX)
		return -ERANGE;
	if (!qcom_camera_icp_bps_session_contains(session, payload_iova,
						   sizeof(struct qcom_camera_icp_hfi_bps_frame_payload)))
		return -ENOENT;

	return qcom_camera_icp_send_sync(session->icp, &command, sizeof(command),
		CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_INDIRECT_ACK,
		QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS,
		CAMERA_ICP_HFI_TEST_TOKEN_BASE +
		QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS);
}

static int
qcom_camera_icp_bps_session_destroy(struct qcom_camera_icp_bps_session *session)
{
	struct qcom_camera_icp *icp = session->icp;
	const u64 token = CAMERA_ICP_HFI_TEST_TOKEN_BASE +
			  QCOM_CAMERA_ICP_HFI_BPS_DESTROY;
	struct qcom_camera_icp_hfi_async_destroy command = {
		.size = cpu_to_le32(sizeof(command)),
		.packet_type =
			cpu_to_le32(CAMERA_ICP_HFI_CMD_IPEBPS_ASYNC_DIRECT),
		.opcode = cpu_to_le32(QCOM_CAMERA_ICP_HFI_BPS_DESTROY),
		.user_data1 = cpu_to_le64(token),
		.num_fw_handles = cpu_to_le32(1),
		.fw_handle = cpu_to_le32(session->handle),
		.destroy_user_data = cpu_to_le64(token),
	};

	int ret;

	if (!session->handle)
		return 0;
	if (session->mapping_count)
		return -EBUSY;

	ret = qcom_camera_icp_send_sync(icp, &command, sizeof(command),
			CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_DIRECT_ACK,
			QCOM_CAMERA_ICP_HFI_BPS_DESTROY, token);
	if (!ret) {
		session->handle = 0;
		session->config_iova = 0;
		session->configured = false;
	}

	return ret;
}

static int
qcom_camera_icp_bps_session_close(struct qcom_camera_icp_bps_session *session)
{
	int ret;

	ret = qcom_camera_icp_bps_session_unmap_all(session);
	if (ret)
		return ret;

	return qcom_camera_icp_bps_session_destroy(session);
}

static int qcom_camera_icp_bps_power_on(struct qcom_camera_icp_bps *bps)
{
	struct qcom_camera_icp *icp = container_of(bps, struct qcom_camera_icp, bps);
	int ret;

	ret = rproc_boot(icp->rproc);
	if (ret)
		return ret;
	if (!READ_ONCE(icp->hfi_ready)) {
		rproc_shutdown(icp->rproc);
		return -EIO;
	}

	return 0;
}

static void qcom_camera_icp_bps_power_off(struct qcom_camera_icp_bps *bps)
{
	struct qcom_camera_icp *icp = container_of(bps, struct qcom_camera_icp, bps);

	rproc_shutdown(icp->rproc);
}

static struct qcom_camera_icp_bps_session *
qcom_camera_icp_bps_client_session_open(struct qcom_camera_icp_bps *bps)
{
	struct qcom_camera_icp *icp = container_of(bps, struct qcom_camera_icp, bps);
	struct qcom_camera_icp_bps_session *session;
	int ret;

	if (!READ_ONCE(icp->hfi_ready))
		return ERR_PTR(-EHOSTDOWN);
	mutex_lock(&icp->bps_client_lock);
	if (icp->bps_client_active) {
		mutex_unlock(&icp->bps_client_lock);
		return ERR_PTR(-EBUSY);
	}
	icp->bps_client_active = true;
	mutex_unlock(&icp->bps_client_lock);

	session = kzalloc_obj(*session);
	if (!session) {
		ret = -ENOMEM;
		goto release_client;
	}
	ret = qcom_camera_icp_bps_session_open(icp, session);
	if (ret) {
		kfree(session);
		goto release_client;
	}

	return session;

release_client:
	mutex_lock(&icp->bps_client_lock);
	icp->bps_client_active = false;
	mutex_unlock(&icp->bps_client_lock);
	return ERR_PTR(ret);
}

static int qcom_camera_icp_bps_client_session_close(
	struct qcom_camera_icp_bps_session *session)
{
	struct qcom_camera_icp *icp = session->icp;
	int ret;

	ret = qcom_camera_icp_bps_session_close(session);
	if (!ret) {
		if (session->work_cpu)
			dma_free_coherent(icp->dev, session->work_size,
					  session->work_cpu, session->work_iova);
		kfree(session);
		mutex_lock(&icp->bps_client_lock);
		icp->bps_client_active = false;
		mutex_unlock(&icp->bps_client_lock);
	}

	return ret;
}

static void qcom_camera_icp_bps_client_session_abort(
	struct qcom_camera_icp_bps_session *session)
{
	struct qcom_camera_icp *icp = session->icp;

	if (session->work_cpu)
		dma_free_coherent(icp->dev, session->work_size,
				  session->work_cpu, session->work_iova);
	kfree(session);
	mutex_lock(&icp->bps_client_lock);
	icp->bps_client_active = false;
	mutex_unlock(&icp->bps_client_lock);
}

static int qcom_camera_icp_bps_client_session_map(
	struct qcom_camera_icp_bps_session *session, dma_addr_t iova, size_t size)
{
	return qcom_camera_icp_bps_session_map(session, iova, size, true);
}

static int qcom_camera_icp_bps_client_session_unmap(
	struct qcom_camera_icp_bps_session *session, dma_addr_t iova, size_t size)
{
	return qcom_camera_icp_bps_session_map(session, iova, size, false);
}

static int qcom_camera_icp_bps_client_alias_map(
	struct qcom_camera_icp_bps *bps, dma_addr_t source_iova,
	dma_addr_t bps_iova, size_t size)
{
	struct qcom_camera_icp *icp = container_of(bps, struct qcom_camera_icp, bps);
	struct qcom_camera_icp_bps_buffer buffer = {
		.iova = source_iova,
		.size = size,
	};

	return qcom_camera_icp_bps_buffer_map(icp, &buffer, bps_iova);
}

static int qcom_camera_icp_bps_client_alias_map_sg(
	struct qcom_camera_icp_bps *bps, struct sg_table *sgt,
	dma_addr_t bps_iova, size_t size)
{
	struct qcom_camera_icp *icp = container_of(bps, struct qcom_camera_icp, bps);
	struct scatterlist *sg;
	size_t mapped = 0;
	unsigned int i;
	int ret;

	if (!sgt || !size || !IS_ALIGNED(bps_iova, PAGE_SIZE) ||
	    !IS_ALIGNED(size, PAGE_SIZE) || bps_iova > U32_MAX - (size - 1))
		return -EINVAL;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		dma_addr_t source_iova = sg_dma_address(sg);
		size_t segment_size = min_t(size_t, sg_dma_len(sg), size - mapped);
		size_t offset;

		if (!IS_ALIGNED(source_iova, PAGE_SIZE) ||
		    !IS_ALIGNED(segment_size, PAGE_SIZE)) {
			ret = -EINVAL;
			goto unmap;
		}
		for (offset = 0; offset < segment_size; offset += PAGE_SIZE) {
			phys_addr_t phys;

			phys = iommu_iova_to_phys(icp->domain,
						  source_iova + offset);
			if (!phys || !IS_ALIGNED(phys, PAGE_SIZE)) {
				ret = -EFAULT;
				goto unmap;
			}
			ret = iommu_map(icp->domain, bps_iova + mapped,
					phys, PAGE_SIZE,
					IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE,
					GFP_KERNEL);
			if (ret)
				goto unmap;
			mapped += PAGE_SIZE;
		}
		if (mapped == size)
			return 0;
	}

	ret = -EINVAL;
unmap:
	if (mapped)
		iommu_unmap(icp->domain, bps_iova, mapped);
	return ret;
}

static void qcom_camera_icp_bps_client_alias_unmap(
	struct qcom_camera_icp_bps *bps, dma_addr_t bps_iova, size_t size)
{
	struct qcom_camera_icp *icp = container_of(bps, struct qcom_camera_icp, bps);
	struct qcom_camera_icp_bps_buffer buffer = {
		.bps_iova = bps_iova,
		.bps_size = size,
	};

	qcom_camera_icp_bps_buffer_unmap(icp, &buffer);
}

static int qcom_camera_icp_bps_client_session_prepare(
	struct qcom_camera_icp_bps_session *session,
	const struct qcom_camera_icp_bps_config *client_config)
{
	const struct firmware *striping_firmware;
	struct qcom_camera_icp_hfi_bps_frame_payload *payload;
	struct qcom_camera_icp_hfi_bps_config *config;
	struct qcom_camera_icp *icp = session->icp;
	char striping_name[64];
	size_t cdm_offset;
	size_t gamma_offset;
	int ret;

	if (session->work_cpu || !client_config || !client_config->width ||
	    !client_config->height || !client_config->input_stride ||
	    !client_config->output_stride)
		return -EINVAL;
	snprintf(striping_name, sizeof(striping_name),
		 CAMERA_ICP_BPS_STRIPING_FIRMWARE_FMT,
		 client_config->width, client_config->height);
	ret = request_firmware(&striping_firmware, striping_name, icp->dev);
	if (ret)
		return ret;
	if (!striping_firmware->size ||
	    striping_firmware->size > CAMERA_ICP_BPS_FRAME_CDM_SIZE) {
		ret = -EINVAL;
		goto release_firmware;
	}

	cdm_offset = PAGE_ALIGN(CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET +
				striping_firmware->size);
	gamma_offset = cdm_offset + CAMERA_ICP_BPS_FRAME_CDM_SIZE;
	session->work_size = PAGE_ALIGN(gamma_offset +
					CAMERA_ICP_BPS_GAMMA_LUT_SIZE);
	session->work_cpu = dma_alloc_coherent(icp->dev, session->work_size,
					       &session->work_iova,
					       GFP_KERNEL | GFP_DMA32);
	if (!session->work_cpu) {
		ret = -ENOMEM;
		goto release_firmware;
	}
	if (session->work_iova > U32_MAX - (session->work_size - 1)) {
		ret = -ERANGE;
		goto free_work;
	}

	memset(session->work_cpu, 0, session->work_size);
	config = session->work_cpu + CAMERA_ICP_BPS_FRAME_CONFIG_OFFSET;
	payload = session->work_cpu + CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET;
	qcom_camera_icp_init_bps_config(icp, client_config, config);
	memcpy(session->work_cpu + CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET,
	       striping_firmware->data, striping_firmware->size);
	qcom_camera_icp_init_bps_frame_settings(session->work_cpu,
						 session->work_iova,
						 gamma_offset);
	payload->frame.data.max_cores = cpu_to_le32(1);
	payload->frame.data.cdm_iova =
		cpu_to_le32(session->work_iova + cdm_offset);
	payload->frame.data.cdm_size = cpu_to_le32(CAMERA_ICP_BPS_FRAME_CDM_SIZE);
	payload->frame.data.iq_iova = cpu_to_le32(session->work_iova +
						   CAMERA_ICP_BPS_FRAME_IQ_OFFSET);
	payload->frame.data.striping_iova = cpu_to_le32(session->work_iova +
					      CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET);
	payload->frame.data.cdm_program_array_iova = cpu_to_le32(
		session->work_iova + CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET +
		offsetof(struct qcom_camera_icp_hfi_bps_frame_payload,
			 cdm_allocator));
	payload->frame.data.num_frame_sets = cpu_to_le32(1);
	payload->frame.user_data = cpu_to_le64(CAMERA_ICP_HFI_TEST_TOKEN_BASE +
					      QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS);
	dma_wmb();

	ret = qcom_camera_icp_bps_session_map(session, session->work_iova,
					      session->work_size, true);
	if (ret)
		goto free_work;
	ret = qcom_camera_icp_bps_session_config(session, session->work_iova);
	if (ret)
		goto release_firmware;

	release_firmware(striping_firmware);
	return 0;

free_work:
	dma_free_coherent(icp->dev, session->work_size, session->work_cpu,
			  session->work_iova);
	session->work_cpu = NULL;
	session->work_iova = 0;
	session->work_size = 0;
release_firmware:
	release_firmware(striping_firmware);
	return ret;
}

static int qcom_camera_icp_bps_client_session_process(
	struct qcom_camera_icp_bps_session *session,
	const struct qcom_camera_icp_bps_job *job)
{
	struct qcom_camera_icp_hfi_bps_frame_payload *payload;
	__le32 *white_balance_command;
	int ret;

	if (!session->work_cpu || !session->configured || !job ||
	    job->red_balance_q10 < QCOM_CAMERA_ICP_BPS_BALANCE_MIN_Q10 ||
	    job->red_balance_q10 > QCOM_CAMERA_ICP_BPS_BALANCE_MAX_Q10 ||
	    job->blue_balance_q10 < QCOM_CAMERA_ICP_BPS_BALANCE_MIN_Q10 ||
	    job->blue_balance_q10 > QCOM_CAMERA_ICP_BPS_BALANCE_MAX_Q10)
		return -EINVAL;
	payload = session->work_cpu + CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET;
	payload->frame.data.request_id = cpu_to_le32(job->request_id);
	ret = qcom_camera_icp_bps_encode_buffer(job->input_iova,
		&payload->frame.data.frame_set[0].buffer[0].buffer[0]);
	ret |= qcom_camera_icp_bps_encode_buffer(job->output_y_iova,
		 &payload->frame.data.frame_set[0].buffer[1].buffer[0]);
	ret |= qcom_camera_icp_bps_encode_buffer(job->output_c_iova,
		 &payload->frame.data.frame_set[0].buffer[1].buffer[1]);
	if (ret)
		return ret;
	white_balance_command = session->work_cpu +
		CAMERA_ICP_BPS_FRAME_GENERIC_CDM_OFFSET;
	qcom_camera_icp_bps_write_white_balance(session->work_cpu,
						white_balance_command,
						job->red_balance_q10,
						job->blue_balance_q10);
	dma_wmb();

	return qcom_camera_icp_bps_session_submit(session, session->work_iova +
		CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET, job->request_id);
}

static const struct qcom_camera_icp_bps_ops qcom_camera_icp_bps_ops = {
	.power_on = qcom_camera_icp_bps_power_on,
	.power_off = qcom_camera_icp_bps_power_off,
	.session_open = qcom_camera_icp_bps_client_session_open,
	.session_close = qcom_camera_icp_bps_client_session_close,
	.session_abort = qcom_camera_icp_bps_client_session_abort,
	.session_map = qcom_camera_icp_bps_client_session_map,
	.session_unmap = qcom_camera_icp_bps_client_session_unmap,
	.session_config = qcom_camera_icp_bps_session_config,
	.session_submit = qcom_camera_icp_bps_session_submit,
	.session_prepare = qcom_camera_icp_bps_client_session_prepare,
	.session_process = qcom_camera_icp_bps_client_session_process,
	.alias_map = qcom_camera_icp_bps_client_alias_map,
	.alias_map_sg = qcom_camera_icp_bps_client_alias_map_sg,
	.alias_unmap = qcom_camera_icp_bps_client_alias_unmap,
};

static void
qcom_camera_icp_init_bps_config(struct qcom_camera_icp *icp,
				const struct qcom_camera_icp_bps_config *client_config,
				struct qcom_camera_icp_hfi_bps_config *config)
{
	struct qcom_camera_icp_hfi_image_desc *input;
	struct qcom_camera_icp_hfi_image_desc *output;
	u32 input_stride;
	u32 output_stride;

	input = &config->image[QCOM_CAMERA_ICP_HFI_BPS_INPUT_IMAGE];
	output = &config->image[QCOM_CAMERA_ICP_HFI_BPS_OUTPUT_IMAGE_FULL];
	input_stride = client_config->input_stride;
	output_stride = client_config->output_stride;

	input->info.format =
		cpu_to_le32(QCOM_CAMERA_ICP_HFI_IMAGE_FORMAT_MIPI_10);
	input->info.width = cpu_to_le32(client_config->width);
	input->info.height = cpu_to_le32(client_config->height);
	input->info.bayer_order =
		cpu_to_le32(QCOM_CAMERA_ICP_HFI_BAYER_FIRST_PIXEL_B);
	input->plane[0].stride = cpu_to_le32(input_stride);
	input->plane[0].height = cpu_to_le32(client_config->height);

	output->info.format =
		cpu_to_le32(QCOM_CAMERA_ICP_HFI_IMAGE_FORMAT_LINEAR_NV12);
	output->info.width = cpu_to_le32(client_config->width);
	output->info.height = cpu_to_le32(client_config->height);
	output->plane[0].stride = cpu_to_le32(output_stride);
	output->plane[0].height = cpu_to_le32(client_config->height);
	output->plane[1].stride = cpu_to_le32(output_stride);
	output->plane[1].height = cpu_to_le32(client_config->height / 2);
	config->user_data =
		cpu_to_le64(CAMERA_ICP_HFI_TEST_TOKEN_BASE +
			    QCOM_CAMERA_ICP_HFI_BPS_CONFIG_IO);

	dev_info(icp->dev,
		 "BPS CONFIG_IO: %ux%u MIPI10 stride %u to linear NV12 stride %u\n",
		 client_config->width, client_config->height,
		 input_stride, output_stride);
}

static int qcom_camera_icp_test_bps_config(struct qcom_camera_icp *icp)
{
	struct qcom_camera_icp_bps_config client_config = {
		.width = test_bps_width,
		.height = test_bps_height,
		.input_stride = ALIGN(DIV_ROUND_UP(test_bps_width * 10, 8), 64),
		.output_stride = ALIGN(test_bps_width, 128),
	};
	struct qcom_camera_icp_bps_session session;
	struct qcom_camera_icp_hfi_bps_config *config;
	int cleanup_ret;
	int ret;

	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_image_info) != 64);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_image_desc) != 96);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_config) != 1248);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_create_handle) != 28);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_async_indirect) != 40);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_async_map) != 56);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_async_destroy) != 44);

	if (test_bps_width < 64 || test_bps_width > 16384 ||
	    test_bps_height < 64 || test_bps_height > 16384 ||
	    !IS_ALIGNED(test_bps_width, 2) ||
	    !IS_ALIGNED(test_bps_height, 2))
		return -EINVAL;

	ret = qcom_camera_icp_bps_buffer_alloc(icp, &icp->bps_test_buffer,
					       PAGE_SIZE);
	if (ret)
		return ret;

	config = icp->bps_test_buffer.cpu;
	memset(config, 0, icp->bps_test_buffer.size);
	qcom_camera_icp_init_bps_config(icp, &client_config, config);
	dma_sync_single_for_device(icp->dev, icp->bps_test_buffer.iova,
				   icp->bps_test_buffer.size,
				   DMA_BIDIRECTIONAL);

	ret = qcom_camera_icp_bps_session_open(icp, &session);
	if (ret)
		goto free_buffer;

	ret = qcom_camera_icp_bps_session_map_buffer(&session,
					     &icp->bps_test_buffer);
	if (ret)
		goto close_session;

	ret = qcom_camera_icp_bps_session_config(&session,
					icp->bps_test_buffer.iova);
	if (ret == -ETIMEDOUT)
		return ret;

close_session:
	cleanup_ret = qcom_camera_icp_bps_session_close(&session);
	if (cleanup_ret)
		return cleanup_ret;

free_buffer:
	qcom_camera_icp_bps_buffer_free(icp, &icp->bps_test_buffer);

	if (ret)
		return ret;

	dev_info(icp->dev, "Camera ICP BPS CONFIG_IO probe succeeded\n");
	return 0;
}

static bool qcom_camera_icp_bps_log_nv12_output(struct qcom_camera_icp *icp,
						const u8 *luma,
						const u8 *chroma,
						u32 request_id,
						u32 frame_set,
						u8 initial_value)
{
	size_t luma_initial = 0;
	size_t luma_zero = 0;
	size_t luma_other = 0;
	size_t chroma_initial = 0;
	size_t chroma_zero = 0;
	size_t chroma_other = 0;
	u8 luma_min = U8_MAX;
	u8 luma_max = 0;
	u8 cb_min = U8_MAX;
	u8 cb_max = 0;
	u8 cr_min = U8_MAX;
	u8 cr_max = 0;
	size_t i;

	for (i = 0; i < 640 * 480; i++) {
		luma_min = min(luma_min, luma[i]);
		luma_max = max(luma_max, luma[i]);
		if (luma[i] == initial_value)
			luma_initial++;
		else if (!luma[i])
			luma_zero++;
		else
			luma_other++;
	}

	for (i = 0; i < 640 * 240; i += 2) {
		cb_min = min(cb_min, chroma[i]);
		cb_max = max(cb_max, chroma[i]);
		cr_min = min(cr_min, chroma[i + 1]);
		cr_max = max(cr_max, chroma[i + 1]);
		if (chroma[i] == initial_value)
			chroma_initial++;
		else if (!chroma[i])
			chroma_zero++;
		else
			chroma_other++;
		if (chroma[i + 1] == initial_value)
			chroma_initial++;
		else if (!chroma[i + 1])
			chroma_zero++;
		else
			chroma_other++;
	}

	dev_info(icp->dev,
		 "BPS request %u frame set %u NV12 ranges: Y=%u..%u Cb=%u..%u Cr=%u..%u; Y initial=%zu zero=%zu other=%zu; UV initial=%zu zero=%zu other=%zu\n",
		 request_id, frame_set, luma_min, luma_max, cb_min, cb_max,
		 cr_min, cr_max, luma_initial, luma_zero, luma_other,
		 chroma_initial, chroma_zero, chroma_other);

	return luma_other && chroma_other;
}

static int qcom_camera_icp_test_frame_bps(struct qcom_camera_icp *icp)
{
	struct qcom_camera_icp_bps_config client_config = {
		.width = test_bps_width,
		.height = test_bps_height,
		.input_stride = ALIGN(DIV_ROUND_UP(test_bps_width * 10, 8), 64),
		.output_stride = ALIGN(test_bps_width, 128),
	};
	const struct firmware *striping_firmware;
	struct qcom_camera_icp_bps_session session;
	struct qcom_camera_icp_hfi_bps_frame_payload *payload;
	struct qcom_camera_icp_hfi_bps_config *config;
	dma_addr_t bps_base;
	dma_addr_t fw_base;
	u8 *input;
	u8 *input_2;
	u8 *output_y;
	u8 *output_y_2;
	u8 *output_c;
	u8 *output_c_2;
	u32 frame;
	u32 x;
	u32 y;
	int cleanup_ret;
	int ret;

	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_frame_buffers) != 20);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_frame_set) != 240);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_frame_data) != 5700);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_frame) != 5712);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_cdm_program) != 24);
	BUILD_BUG_ON(sizeof(struct qcom_camera_icp_hfi_bps_frame_payload) != 6128);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_CONFIG_OFFSET +
		     sizeof(struct qcom_camera_icp_hfi_bps_config) >
		     CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET +
		     sizeof(struct qcom_camera_icp_hfi_bps_frame_payload) >
		     CAMERA_ICP_BPS_FRAME_IQ_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_IQ_OFFSET +
		     CAMERA_ICP_BPS_FRAME_IQ_SIZE >
		     CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET +
		     CAMERA_ICP_BPS_FRAME_STRIPING_SIZE >
		     CAMERA_ICP_BPS_FRAME_CDM_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_CDM_OFFSET +
		     CAMERA_ICP_BPS_FRAME_CDM_SIZE >
		     CAMERA_ICP_BPS_FRAME_INPUT_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_INPUT_OFFSET + 832 * 480 >
		     CAMERA_ICP_BPS_FRAME_OUTPUT_Y_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_OUTPUT_Y_OFFSET + 640 * 480 >
		     CAMERA_ICP_BPS_FRAME_OUTPUT_C_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_OUTPUT_C_OFFSET + 640 * 240 >
		     CAMERA_ICP_BPS_FRAME_INPUT_2_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_INPUT_2_OFFSET + 832 * 480 >
		     CAMERA_ICP_BPS_FRAME_OUTPUT_Y_2_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_OUTPUT_Y_2_OFFSET + 640 * 480 >
		     CAMERA_ICP_BPS_FRAME_OUTPUT_C_2_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_OUTPUT_C_2_OFFSET + 640 * 240 >
		     CAMERA_ICP_BPS_FRAME_GAMMA_OFFSET);
	BUILD_BUG_ON(CAMERA_ICP_BPS_FRAME_GAMMA_OFFSET +
		     CAMERA_ICP_BPS_GAMMA_LUT_SIZE >
		     CAMERA_ICP_BPS_FRAME_BUFFER_SIZE);
	BUILD_BUG_ON(U32_MAX - (CAMERA_ICP_BPS_FRAME_BUFFER_SIZE - 1) <
		     CAMERA_ICP_BPS_FRAME_IOVA);

	if (test_bps_width != 640 || test_bps_height != 480 ||
	    !test_bps_frames || test_bps_frames > 16 ||
	    !test_bps_batch || test_bps_batch > 2)
		return -EINVAL;

	ret = request_firmware(&striping_firmware,
			       CAMERA_ICP_BPS_TEST_STRIPING_FIRMWARE, icp->dev);
	if (ret)
		return ret;
	if (striping_firmware->size != CAMERA_ICP_BPS_FRAME_STRIPING_SIZE) {
		ret = -EINVAL;
		goto release_firmware;
	}

	ret = qcom_camera_icp_bps_buffer_alloc(icp, &icp->bps_test_buffer,
				 CAMERA_ICP_BPS_FRAME_BUFFER_SIZE);
	if (ret)
		goto free_buffer;
	ret = qcom_camera_icp_bps_buffer_map(icp, &icp->bps_test_buffer,
					     CAMERA_ICP_BPS_FRAME_IOVA);
	if (ret)
		goto free_buffer;

	memset(icp->bps_test_buffer.cpu, 0, icp->bps_test_buffer.size);
	bps_base = icp->bps_test_buffer.bps_iova;
	fw_base = icp->bps_test_buffer.iova;
	config = icp->bps_test_buffer.cpu + CAMERA_ICP_BPS_FRAME_CONFIG_OFFSET;
	payload = icp->bps_test_buffer.cpu + CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET;
	input = icp->bps_test_buffer.cpu + CAMERA_ICP_BPS_FRAME_INPUT_OFFSET;
	input_2 = icp->bps_test_buffer.cpu + CAMERA_ICP_BPS_FRAME_INPUT_2_OFFSET;
	output_y = icp->bps_test_buffer.cpu + CAMERA_ICP_BPS_FRAME_OUTPUT_Y_OFFSET;
	output_y_2 = icp->bps_test_buffer.cpu +
		CAMERA_ICP_BPS_FRAME_OUTPUT_Y_2_OFFSET;
	output_c = icp->bps_test_buffer.cpu + CAMERA_ICP_BPS_FRAME_OUTPUT_C_OFFSET;
	output_c_2 = icp->bps_test_buffer.cpu +
		CAMERA_ICP_BPS_FRAME_OUTPUT_C_2_OFFSET;

	qcom_camera_icp_init_bps_config(icp, &client_config, config);
	memcpy(icp->bps_test_buffer.cpu + CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET,
	       striping_firmware->data, striping_firmware->size);
	qcom_camera_icp_init_bps_frame_settings(icp->bps_test_buffer.cpu,
						 fw_base,
						 CAMERA_ICP_BPS_FRAME_GAMMA_OFFSET);
	payload->frame.data.max_cores = cpu_to_le32(1);
	payload->frame.data.cdm_iova =
		cpu_to_le32(fw_base + CAMERA_ICP_BPS_FRAME_CDM_OFFSET);
	payload->frame.data.cdm_size =
		cpu_to_le32(CAMERA_ICP_BPS_FRAME_CDM_SIZE);
	payload->frame.data.iq_iova =
		cpu_to_le32(fw_base + CAMERA_ICP_BPS_FRAME_IQ_OFFSET);
	payload->frame.data.striping_iova =
		cpu_to_le32(fw_base + CAMERA_ICP_BPS_FRAME_STRIPING_OFFSET);
	payload->frame.data.cdm_program_array_iova = cpu_to_le32(fw_base +
		CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET +
		offsetof(struct qcom_camera_icp_hfi_bps_frame_payload,
			 cdm_allocator));
	payload->frame.data.num_frame_sets = cpu_to_le32(test_bps_batch);
	ret = qcom_camera_icp_bps_encode_buffer(bps_base +
			CAMERA_ICP_BPS_FRAME_INPUT_OFFSET,
			&payload->frame.data.frame_set[0].buffer[0].buffer[0]);
	ret |= qcom_camera_icp_bps_encode_buffer(bps_base +
			 CAMERA_ICP_BPS_FRAME_OUTPUT_Y_OFFSET,
			 &payload->frame.data.frame_set[0].buffer[1].buffer[0]);
	ret |= qcom_camera_icp_bps_encode_buffer(bps_base +
			 CAMERA_ICP_BPS_FRAME_OUTPUT_C_OFFSET,
			 &payload->frame.data.frame_set[0].buffer[1].buffer[1]);
	ret |= qcom_camera_icp_bps_encode_buffer(bps_base +
			 CAMERA_ICP_BPS_FRAME_INPUT_2_OFFSET,
			 &payload->frame.data.frame_set[1].buffer[0].buffer[0]);
	ret |= qcom_camera_icp_bps_encode_buffer(bps_base +
			 CAMERA_ICP_BPS_FRAME_OUTPUT_Y_2_OFFSET,
			 &payload->frame.data.frame_set[1].buffer[1].buffer[0]);
	ret |= qcom_camera_icp_bps_encode_buffer(bps_base +
			 CAMERA_ICP_BPS_FRAME_OUTPUT_C_2_OFFSET,
			 &payload->frame.data.frame_set[1].buffer[1].buffer[1]);
	if (ret)
		goto free_buffer;
	payload->frame.user_data = cpu_to_le64(CAMERA_ICP_HFI_TEST_TOKEN_BASE +
					      QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS);
	dma_sync_single_for_device(icp->dev, icp->bps_test_buffer.iova,
				   icp->bps_test_buffer.size,
				   DMA_BIDIRECTIONAL);
	ret = qcom_camera_icp_bps_session_open(icp, &session);
	if (ret)
		goto free_buffer;
	ret = qcom_camera_icp_bps_session_map_buffer(&session,
					     &icp->bps_test_buffer);
	if (ret)
		goto close_session;
	ret = qcom_camera_icp_bps_session_config(&session,
					icp->bps_test_buffer.iova);
	if (ret == -ETIMEDOUT)
		goto release_firmware;
	if (ret)
		goto close_session;

	for (frame = 0; frame < test_bps_frames; frame++) {
		for (y = 0; y < 480; y++) {
			u8 *row = input + y * 832;
			u8 *row_2 = input_2 + y * 832;

			for (x = 0; x < 640; x += 4) {
				u16 p0 = (x + y * 3 + frame * 17) & 0x3ff;
				u16 p1 = (x + 1 + y * 3 + frame * 17) & 0x3ff;
				u16 p2 = (x + 2 + y * 3 + frame * 17) & 0x3ff;
				u16 p3 = (x + 3 + y * 3 + frame * 17) & 0x3ff;
				u8 *group = row + x / 4 * 5;
				u8 *group_2 = row_2 + x / 4 * 5;

				group[0] = p0 >> 2;
				group[1] = p1 >> 2;
				group[2] = p2 >> 2;
				group[3] = p3 >> 2;
				group[4] = (p0 & 3) | ((p1 & 3) << 2) |
					   ((p2 & 3) << 4) |
					   ((p3 & 3) << 6);
				memcpy(group_2, group, 5);
			}
		}
		memset(output_y, 0xa5, 640 * 480);
		memset(output_c, 0xa5, 640 * 240);
		memset(output_y_2, 0xa5, 640 * 480);
		memset(output_c_2, 0xa5, 640 * 240);
		payload->frame.data.request_id = cpu_to_le32(frame + 1);
		dma_sync_single_for_device(icp->dev,
					   icp->bps_test_buffer.iova,
					   icp->bps_test_buffer.size,
					   DMA_BIDIRECTIONAL);

		ret = qcom_camera_icp_bps_session_submit(&session,
			icp->bps_test_buffer.iova +
			CAMERA_ICP_BPS_FRAME_PAYLOAD_OFFSET, frame + 1);
		if (ret)
			break;

		dma_sync_single_for_cpu(icp->dev, icp->bps_test_buffer.iova,
					icp->bps_test_buffer.size,
					DMA_BIDIRECTIONAL);
		if (!qcom_camera_icp_bps_log_nv12_output(icp, output_y,
							 output_c, frame + 1, 0,
							 0xa5)) {
			ret = -ENODATA;
			break;
		}
		if (test_bps_batch == 2 &&
		    !qcom_camera_icp_bps_log_nv12_output(icp, output_y_2,
							  output_c_2, frame + 1,
							  1, 0xa5)) {
			ret = -ENODATA;
			break;
		}
	}
	/*
	 * A timeout or cleanup failure leaves ownership uncertain.  Do not free
	 * memory the firmware may still touch; the remoteproc shutdown path resets
	 * ICP first and qcom_camera_icp_unprepare_hfi() releases the retained DMA
	 * buffer.
	 */
	if (ret == -ETIMEDOUT)
		goto release_firmware;
close_session:
	cleanup_ret = qcom_camera_icp_bps_session_close(&session);
	if (cleanup_ret) {
		ret = cleanup_ret;
		goto release_firmware;
	}
free_buffer:
	qcom_camera_icp_bps_buffer_free(icp, &icp->bps_test_buffer);
release_firmware:
	release_firmware(striping_firmware);
	if (!ret)
		dev_info(icp->dev,
			 "Camera ICP BPS %u-request/%u-frame-set probe succeeded\n",
			 test_bps_frames, test_bps_batch);
	return ret;
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
	ret = qcom_camera_icp_configure_ubwc(icp);
	if (ret) {
		dev_err(icp->dev, "failed to configure BPS/IPE UBWC: %d\n", ret);
		goto disable_hfi;
	}

	ret = qcom_camera_icp_ping(icp);
	if (ret) {
		dev_err(icp->dev, "HFI ping failed: %d\n", ret);
		goto disable_hfi;
	}
	dev_info(icp->dev, "Camera ICP HFI ping succeeded\n");
	if (test_bps_config || test_bps_frame) {
		ret = test_bps_frame ? qcom_camera_icp_test_frame_bps(icp) :
				       qcom_camera_icp_test_bps_config(icp);
		if (ret) {
			dev_err(icp->dev, "BPS probe failed: %d\n", ret);
			goto disable_hfi;
		}
	}

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
		} else if (packet_type ==
				   CAMERA_ICP_HFI_MSG_IPEBPS_CREATE_HANDLE_ACK &&
			   packet_type == READ_ONCE(icp->hfi_command_ack)) {
			u64 token;

			if (packet_size < 6 * sizeof(u32))
				continue;
			token = le32_to_cpu(icp->hfi_msg[4]);
			token |= (u64)le32_to_cpu(icp->hfi_msg[5]) << 32;
			if (token != READ_ONCE(icp->hfi_command_token))
				continue;
			WRITE_ONCE(icp->hfi_command_error,
				   le32_to_cpu(icp->hfi_msg[2]));
			WRITE_ONCE(icp->hfi_command_result, 0);
			WRITE_ONCE(icp->hfi_command_handle,
				   le32_to_cpu(icp->hfi_msg[3]));
			complete(&icp->hfi_command_done);
		} else if ((packet_type ==
				    CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_DIRECT_ACK ||
			    packet_type ==
				    CAMERA_ICP_HFI_MSG_IPEBPS_ASYNC_INDIRECT_ACK) &&
			   packet_type == READ_ONCE(icp->hfi_command_ack)) {
			u64 token;
			u32 opcode;

			if (packet_size < 8 * sizeof(u32))
				continue;
			opcode = le32_to_cpu(icp->hfi_msg[2]);
			if (opcode != READ_ONCE(icp->hfi_command_opcode))
				continue;
			if ((opcode == QCOM_CAMERA_ICP_HFI_BPS_CONFIG_IO ||
			     opcode == QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS) &&
			    packet_size < 9 * sizeof(u32))
				continue;
			token = le32_to_cpu(icp->hfi_msg[3]);
			token |= (u64)le32_to_cpu(icp->hfi_msg[4]) << 32;
			if (token != READ_ONCE(icp->hfi_command_token))
				continue;
			WRITE_ONCE(icp->hfi_command_error,
				   le32_to_cpu(icp->hfi_msg[7]));
			WRITE_ONCE(icp->hfi_command_result,
				   (opcode == QCOM_CAMERA_ICP_HFI_BPS_CONFIG_IO ||
				    opcode == QCOM_CAMERA_ICP_HFI_BPS_FRAME_PROCESS) ?
				   le32_to_cpu(icp->hfi_msg[8]) : 0);
			complete(&icp->hfi_command_done);
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
	icp->bps.ops = &qcom_camera_icp_bps_ops;
	init_completion(&icp->hfi_sys_init_done);
	init_completion(&icp->hfi_ping_done);
	init_completion(&icp->hfi_command_done);
	mutex_init(&icp->bps_client_lock);
	mutex_init(&icp->hfi_command_lock);
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

	icp->bps_auxdev = devm_auxiliary_device_create(dev, "bps", &icp->bps);
	if (!icp->bps_auxdev) {
		ret = -ENOMEM;
		goto del_rproc;
	}

	return 0;

del_rproc:
	rproc_del(rproc);
	return dev_err_probe(dev, ret, "failed to add BPS auxiliary device\n");
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
