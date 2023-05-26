// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/io.h>

#include "leap.h"

#define LEAP_CORE_REG_STRIDE		0x100000
#define REG_LEAP_CORE_ROUTINE_CTL	0x80008
#define REG_LEAP_CORE_ENABLE		0x8000c

#define REG_LEAP_IO_PUT_TAKE(port)		(0xf01000 + (port) * 8)
#define REG_LEAP_IO_UPDATE_PEEK(port)		(0xf00800 + (port) * 8)

#define REG_LEAP_IDLE_COUNT(core)		((0x100000 * (core)) + 0x80044)
#define REG_LEAP_BUSY_COUNT(core)		((0x100000 * (core)) + 0x80048)
#define REG_LEAP_BUSY_ROUTINE_COUNT(core, idx)	((0x100000 * (core)) + 0x8004c + (4 * (idx)))

#define REG_LEAP_STATE_BASE(core, idx)		((0x100000 * (core)) + 0x00000 + (0x10000 * (idx)))
#define REG_LEAP_INST_BASE(core, idx)		((0x100000 * (core)) + 0x40000 + (0x10000 * (idx)))
#define REG_LEAP_ROUTINE_FLAGS(core, idx)	((0x100000 * (core)) + 0x8006c + (4 * (idx)))
#define REG_LEAP_WE_SIEVE_BASE(core, idx)	(0xe00000 + (0x400 * (core)) + (4 * (idx)))
#define REG_LEAP_WF_SIEVE_BASE(core, idx)	(0xe00020 + (0x400 * (core)) + (4 * (idx)))
#define REG_LEAP_PDM_SPECIAL_BASE		0xf000f4

#define LEAP_MAX_SECTION_SIZE	4096
#define LEAP_MAX_ROUTINES	8

#define LEAP_NCORES		4

struct leap_core {
	struct device *dev;
	struct leap_cluster *cluster;
	unsigned int no;
	__iomem void *base;
};

struct leap_cluster {
	struct device *dev;
	__iomem void *base;
	struct reset_control *rstc;
	struct dentry *debugfs_root;

	int ncores;
	struct leap_core cores[];
};

int leap_io_put(struct leap_cluster *cl, int port, u32 val)
{
	if (port < 0x40 || port >= 0x80)
		return -EINVAL;

	writel_relaxed(val, cl->base + REG_LEAP_IO_PUT_TAKE(port));
	return 0;
}
EXPORT_SYMBOL_GPL(leap_io_put);

int leap_io_update(struct leap_cluster *cl, int port, u32 val)
{
	if (port < 0x40 || port >= 0x80)
		return -EINVAL;

	writel_relaxed(val, cl->base + REG_LEAP_IO_UPDATE_PEEK(port));
	return 0;
}
EXPORT_SYMBOL_GPL(leap_io_update);

int leap_io_take(struct leap_cluster *cl, int port, u32 *val)
{
	if (port < 0x40 || port >= 0x80)
		return -EINVAL;

	*val = readl_relaxed(cl->base + REG_LEAP_IO_PUT_TAKE(port));
	return 0;
}
EXPORT_SYMBOL_GPL(leap_io_take);

int leap_io_peek(struct leap_cluster *cl, int port, u32 *val)
{
	if (port < 0x0 || port >= 0x80)
		return -EINVAL;

	*val = readl_relaxed(cl->base + REG_LEAP_IO_UPDATE_PEEK(port));
	return 0;
}
EXPORT_SYMBOL_GPL(leap_io_peek);

void leap_enable(struct leap_core *core)
{
	writel_relaxed(3, core->base + REG_LEAP_CORE_ENABLE);
}
EXPORT_SYMBOL_GPL(leap_enable);

void leap_disable(struct leap_core *core)
{
	writel_relaxed(0, core->base + REG_LEAP_CORE_ENABLE);
}
EXPORT_SYMBOL_GPL(leap_disable);

static void *seek_in_firmware(void *p, size_t offset, void *end)
{
	if (offset > end - p)
		return end;

	return p + offset;
}

struct leapfrog_hdr {
#define LEAPFROG_MAGIC 0x1ea9f108
	__le32 magic;
	__le32 fmtversion;
	char imprint[32];
	__le32 nsections;
} __packed;

struct leapfrog_section_hdr {
	__le32 type;
	__le32 load_base;
	__le32 size;
	__le32 flags;
} __packed;

#define LEAPFROG_SECTION_STATE0		0x10000
#define LEAPFROG_SECTION_STATE1		0x10001
#define LEAPFROG_SECTION_STATE2		0x10002
#define LEAPFROG_SECTION_STATE3		0x10003

#define LEAPFROG_SECTION_INST0		0x20000
#define LEAPFROG_SECTION_INST1		0x20001
#define LEAPFROG_SECTION_INST2		0x20002
#define LEAPFROG_SECTION_INST3		0x20003

#define LEAPFROG_SECTION_ROUTINE_CTL	0x30000
#define LEAPFROG_SECTION_WE_SIEVE	0x30001
#define LEAPFROG_SECTION_WF_SIEVE	0x30002

#define LEAPFROG_SECTION_IO_INIT	0x30100
#define LEAPFROG_SECTION_PDM_SPECIAL	0x30101

#define LEAPFROG_SECTION_FLAG_ROUTINE_EN BIT(0)

static const char *leapfrog_get_imprint(struct leapfrog_hdr *hdr)
{
	if (hdr->imprint[0] == 0 ||
			hdr->imprint[ARRAY_SIZE(hdr->imprint) - 1] != 0)
		return "(no imprint)";

	return hdr->imprint;
}

#define for_leapfrog_section(image, end, p) \
	for (p = seek_in_firmware(image, sizeof(struct leapfrog_hdr), end); \
	     (void *) (p + 1) < end; \
	     p = seek_in_firmware(p, sizeof(struct leapfrog_section_hdr) \
							  + le32_to_cpu(p->size) * 4, end))

static void leapfrog_dump_sections(struct leap_core *core, struct leapfrog_hdr *image, void *end)
{
	struct leapfrog_section_hdr *section;
	int section_no = 0;

	for_leapfrog_section(image, end, section)
		dev_dbg(core->dev, "firmware: section %d: type %06x size %04x load base %08x\n",
			section_no++, le32_to_cpu(section->type), le32_to_cpu(section->size),
			le32_to_cpu(section->load_base));
}

int leap_load_image(struct leap_core *core, void *image, size_t size, bool state_only)
{
	struct leapfrog_hdr *image_hdr = image;
	struct leapfrog_section_hdr *section;
	void *end = image + size;
	int section_no;
	u32 enable_routines = 0;

	if (image + sizeof(struct leapfrog_hdr) > end)
		return -EINVAL;

	if (le32_to_cpu(image_hdr->magic) != 0x1ea9f108) {
		dev_err(core->dev, "firmware: bad magic: %x\n", le32_to_cpu(image_hdr->magic));
		return -EINVAL;
	}

	dev_info(core->dev, "loading firmware onto core %d: %s\n",
			 core->no, leapfrog_get_imprint(image));

	leapfrog_dump_sections(core, image, end);

	section_no = -1;
	for_leapfrog_section(image, end, section) {
		u32 type = le32_to_cpu(section->type);
		u32 size = le32_to_cpu(section->size);
		u32 load_base = le32_to_cpu(section->load_base);
		u32 *data, *data_end;
		unsigned int rout_no;
		unsigned int reg_base;
		int reg_stride, load_limit, i;

		section_no += 1;

		/*
		 * Decode the routine number out of the load base for
		 * sections that are per-routine.
		 */
		switch (type) {
		case LEAPFROG_SECTION_ROUTINE_CTL:
		case LEAPFROG_SECTION_WE_SIEVE:
		case LEAPFROG_SECTION_WF_SIEVE:
			rout_no = load_base >> 16;
			load_base &= 0xffff;

			if (rout_no > LEAP_MAX_ROUTINES) {
				dev_err(core->dev, "firmware: section %d routine no out of bounds\n", section_no);
				leapfrog_dump_sections(core, image, end);
				return -EINVAL;
			}
			break;

		default:
			break;
		}

		switch (type) {
		case LEAPFROG_SECTION_STATE0 ... LEAPFROG_SECTION_STATE3:
			reg_stride = 8;
			reg_base = REG_LEAP_STATE_BASE(core->no, type - LEAPFROG_SECTION_STATE0);
			load_limit = 4096;
			break;

		case LEAPFROG_SECTION_INST0 ... LEAPFROG_SECTION_INST3:
			reg_stride = 4;
			reg_base = REG_LEAP_INST_BASE(core->no, type - LEAPFROG_SECTION_STATE0);
			load_limit = 4096;

			if (state_only)
				continue;
			break;

		case LEAPFROG_SECTION_ROUTINE_CTL:
			reg_stride = 0x20;
			reg_base = REG_LEAP_ROUTINE_FLAGS(core->no, rout_no);
			load_limit = 24;

			if (le32_to_cpu(section->flags) & LEAPFROG_SECTION_FLAG_ROUTINE_EN)
				enable_routines |= BIT(rout_no);

			break;

		case LEAPFROG_SECTION_WE_SIEVE:
		case LEAPFROG_SECTION_WF_SIEVE:
			reg_stride = 0x40;
			if (type == LEAPFROG_SECTION_WE_SIEVE)
				reg_base = REG_LEAP_WE_SIEVE_BASE(core->no, rout_no);
			else
				reg_base = REG_LEAP_WF_SIEVE_BASE(core->no, rout_no);
			load_limit = 8;
			break;

		case LEAPFROG_SECTION_IO_INIT:
			reg_stride = 0x8;
			reg_base = REG_LEAP_IO_UPDATE_PEEK(0x0);
			load_limit = 0x80;

			if (state_only)
				continue;
			break;

		case LEAPFROG_SECTION_PDM_SPECIAL:
			reg_stride = 0x4;
			reg_base = REG_LEAP_PDM_SPECIAL_BASE;
			load_limit = 10;
			break;

		default:
			dev_err(core->dev, "firmware: section %d unknown type %x\n", section_no, type);
			leapfrog_dump_sections(core, image, end);
			return -EINVAL;
		}

		data = seek_in_firmware(section, sizeof(struct leapfrog_section_hdr), end);
		data_end = seek_in_firmware(data, size * 4, end);

		if (data_end - data < size) {
			dev_err(core->dev, "firmware: unexpected EOF (unfinished section %d)\n", section_no);
			return -EINVAL;
		}

		if (load_base + size > load_limit) {
			dev_warn(core->dev, "firmware: clipping section %d to load limit");

			if (load_base >= load_limit)
				continue;
			size = load_limit - load_base;
		}

		for (i = load_base; i < load_base + size; i++)
			writel_relaxed(*data++, core->cluster->base + reg_base \
						   + reg_stride * i);
	}

	writel_relaxed(enable_routines, core->base + REG_LEAP_CORE_ROUTINE_CTL);

	return 0;
}
EXPORT_SYMBOL_GPL(leap_load_image);

static ssize_t leap_counters_read_file(struct file *file, char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct leap_core *core = file->private_data;
	__iomem void *cl_base = core->cluster->base;
	ssize_t ret;
	char *buf;
	int i;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = scnprintf(buf, PAGE_SIZE, "Idle: %08x\n",
			readl_relaxed(cl_base + REG_LEAP_IDLE_COUNT(core->no)));

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Busy: %08x\n",
			 readl_relaxed(cl_base + REG_LEAP_BUSY_COUNT(core->no)));
	for (i = 0; i < LEAP_MAX_ROUTINES; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Routine #%d: %08x\n",
				 i, readl_relaxed(cl_base + REG_LEAP_BUSY_ROUTINE_COUNT(core->no, i)));

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);
	kfree(buf);
	return ret;
}

static const struct file_operations leap_counters_fops = {
	.open = simple_open,
	.read = leap_counters_read_file,
	.llseek = default_llseek,
};

static void leap_debugfs_populate(struct leap_cluster *cl)
{
	int i;

	for (i = 0; i < LEAP_NCORES; i++) {
		struct leap_core *core = leap_get_core(cl, i);
		struct dentry *core_dir;
		char dirname[16];

		snprintf(dirname, sizeof(dirname), "core%d", i);
		core_dir = debugfs_create_dir(dirname, cl->debugfs_root);

		debugfs_create_file("counters", 0444, core_dir, core,
				    &leap_counters_fops);
	}
}

struct leap_core *leap_get_core(struct leap_cluster *cluster, int index)
{
	return &cluster->cores[index];
}
EXPORT_SYMBOL_GPL(leap_get_core);

static void apple_leap_release(struct leap_cluster *cl)
{
	reset_control_rearm(cl->rstc);
}

static struct dentry *leap_debugfs_root;

static int apple_leap_probe(struct platform_device *pdev)
{
	struct leap_cluster *cl;
	int i, ncores;

	ncores = 4;
	cl = devm_kzalloc(&pdev->dev, struct_size(cl, cores, ncores), GFP_KERNEL);
	if (!cl)
		return -ENOMEM;
	cl->ncores = ncores;
	cl->dev = &pdev->dev;
	cl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cl->base))
		return PTR_ERR(cl->base);
	platform_set_drvdata(pdev, cl);
	cl->rstc = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(cl->rstc))
		return PTR_ERR(cl->rstc);

	reset_control_reset(cl->rstc);

	for (i = 0; i < ncores; i++) {
		struct leap_core *core = &cl->cores[i];

		core->dev = &pdev->dev;
		core->cluster = cl;
		core->no = i;
		core->base = cl->base + LEAP_CORE_REG_STRIDE * i;
	}

	cl->debugfs_root = debugfs_create_dir(pdev->name, leap_debugfs_root);
	leap_debugfs_populate(cl);

	return 0;
}

static int apple_leap_remove(struct platform_device *pdev)
{
	struct leap_cluster *cl = platform_get_drvdata(pdev);

	apple_leap_release(cl);
	return 0;
}

static const struct of_device_id apple_leap_of_match[] = {
	{ .compatible = "apple,leap", },
	{}
};
MODULE_DEVICE_TABLE(of, apple_leap_of_match);

static struct platform_driver apple_leap_driver = {
	.driver = {
		.name = "apple-leap",
		.of_match_table = apple_leap_of_match,
	},
	.probe = apple_leap_probe,
	.remove = apple_leap_remove,
};

static int __init apple_leap_init(void)
{
	int ret;

	leap_debugfs_root = debugfs_create_dir("apple_leap", NULL);

	ret = platform_driver_register(&apple_leap_driver);
	if (ret) {
		debugfs_remove_recursive(leap_debugfs_root);
		return ret;
	}

	return 0;
}
module_init(apple_leap_init);

static void __exit apple_leap_exit(void)
{
	platform_driver_unregister(&apple_leap_driver);
	debugfs_remove_recursive(leap_debugfs_root);
}
module_exit(apple_leap_exit);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Apple LEAP signal coprocessor driver");
MODULE_LICENSE("Dual MIT/GPL");
