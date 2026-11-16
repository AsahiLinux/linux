/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 Sofus Forstreuter <sofus.c@icloud.com>
 * Copyright (C) 2026 The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include "piodma.h"

#define CFG	0x00

#define IRQ_STS			0x04
#define IRQ_EN			0x08
#define IRQ_DONE		BIT(0)
#define IRQ_CMD_PARSE_ERR	BIT(1)
#define IRQ_SEQ_FIFO_OVERFLOW	BIT(2)
#define IRQ_AXI_BUS_ERR		BIT(3)
#define IRQ_BOUNDS_ERR		BIT(4)

#define STATUS			0x0c
#define STATUS_CMD_FREE		GENMASK(4, 0)
#define STATUS_CMD_COMPLETED	GENMASK(20, 16)
#define STATUS_BUSY		BIT(31)

#define DMA_CFG_MEM_SRC		0x10
#define DMA_CFG_MEM_DAT		0x14
#define DMA_CFG_MEM_DST		0x18
#define DMA_CFG_PIO_RD		0x1c
#define DMA_CFG_PIO_WR		0x20

#define DMA_CFG_PIO_BASE_CNT	4
#define DMA_CFG_PIO_BASE_ADDR_LO(i)	(0x24 + (i) * 4)
#define DMA_CFG_PIO_BASE_ADDR_HI(i)	(0xf0 + (i) * 4)

#define MEM_DST_ADDR_LO	0x44
#define MEM_DST_ADDR_HI	0x48

#define MEM_SRC_ADDR_LO	0x4c
#define MEM_SRC_ADDR_HI	0x50

#define CMD_CTRL	0x54
#define CMD_CTRL_RUN	BIT(0)
#define CMD_CTRL_TYPE	GENMASK(2, 1)
/*
 * Uses high bits of CMD_CTX to fill CMD_TAG, might be related to the
 * seemingly 4 missing DMA_CFG_PIO_BASE_ADDR
 */
#define CMD_CTRL_CTX1	BIT(3)
#define CMD_CTRL_IRQ	BIT(4)
#define CMD_CTRL_CNT	GENMASK(28, 8)

#define HW_VERSION	0xb4
#define ERR		0xc4
#define ERR_PARSE_CNT	GENMASK(15, 0)

#define CMD_CTX		0xd8
#define CMD_CTX_TAG	GENMASK(7, 0)
#define DONE_CMD_TAG	0xdc

#define MEM_RD_ADDR	0x90
#define MEM_WR_ADDR	0x98
#define PIO_RD_ADDR	0xa0
#define PIO_WR_ADDR	0xa8

enum packet_type {
	PK_TYPE_RW = 1,
	PK_TYPE_LINK = 3,
};

#define PK_TYPE		GENMASK(1, 0)
#define PK_RW_START	GENMASK(17, 2)
#define PK_RW_CNT	GENMASK(29, 18)
#define PK_RW_BASE_SEL	GENMASK(31, 30)

#define PK_LINK_SIZE	GENMASK(23, 4)
#define PK_LINK_FLAG	BIT(26) /* ?? */

#define PK_SIZE		PAGE_SIZE
#define PK_MAX_OFF	0x10000

int piodebug;
module_param(piodebug, int, 0644);

#define pio_dbg(level, fmt, args...) do { \
	if (piodebug >= (level)) \
		pr_info("piodma: " fmt, ##args); \
} while (false)

#define pio_err(fmt, args...) \
	pr_err("piodma %s: " fmt, __func__, ##args)

struct piodma_ctx {
	void __iomem *reg;
	const struct resource *res;
	struct {
		u32 *cpu;
		dma_addr_t addr;
		size_t size;
	} buf;
	struct device *dev;
	size_t cnt;
	size_t cmd_cnt;
	size_t base[DMA_CFG_PIO_BASE_CNT];
	struct delayed_work err_work;
	void *cb_prv;
	void (*cb)(void *prv, enum piodma_completion_status status);
};

static void dbg_registers(struct piodma_ctx *ctx)
{
	int status = readl_relaxed(ctx->reg + STATUS);
	int irqstatus = readl_relaxed(ctx->reg + IRQ_STS);
	int err = readl_relaxed(ctx->reg + ERR);

	pio_dbg(1, "irq: %#x %s%s%s%s%s", irqstatus,
		irqstatus & IRQ_DONE ? "OK " : "",
		irqstatus & IRQ_CMD_PARSE_ERR ? "CMD_PARSE_ERR " : "",
		irqstatus & IRQ_SEQ_FIFO_OVERFLOW ? "FIFO_OVERFLOW " : "",
		irqstatus & IRQ_AXI_BUS_ERR ? "AXI_BUS_ERR " : "",
		irqstatus & IRQ_BOUNDS_ERR ? "BOUNDS_ERR " : "");

	if (err)
		pio_err("err %#x parse cnt %ld\n",
		       err, FIELD_GET(ERR_PARSE_CNT, err));

	pio_dbg(2, "status: %#x free %ld completed %ld busy %s\n",
		status,
		FIELD_GET(STATUS_CMD_FREE, status),
		FIELD_GET(STATUS_CMD_COMPLETED, status),
		FIELD_GET(STATUS_BUSY, status) ? "YES" : "NO");

	pio_dbg(2, "cfg %#x ctx %#x tag %#x",
		readl_relaxed(ctx->reg + CFG),
		readl_relaxed(ctx->reg + CMD_CTX),
		readl_relaxed(ctx->reg + DONE_CMD_TAG));

	pio_dbg(2, "MEM_RD_ADDR: %#llx\n",
		readq_relaxed(ctx->reg + MEM_RD_ADDR));
	pio_dbg(2, "MEM_WR_ADDR: %#llx\n",
		readq_relaxed(ctx->reg + MEM_WR_ADDR));

	pio_dbg(2, "PIO_RD_ADDR: %#llx\n",
		readq_relaxed(ctx->reg + PIO_RD_ADDR));
	pio_dbg(2, "PIO_WR_ADDR: %#llx\n",
		readq_relaxed(ctx->reg + PIO_WR_ADDR));
}

static irqreturn_t piodma_irq_handler(int irq, void *prv)
{
	struct piodma_ctx *ctx = prv;
	int status = readl_relaxed(ctx->reg + IRQ_STS);

	writel(0, ctx->reg + IRQ_EN);

	if (piodebug)
		dbg_registers(ctx);

	WARN_ON_ONCE(!ctx->cb);
	if (cancel_delayed_work(&ctx->err_work) && ctx->cb)
		ctx->cb(ctx->cb_prv,
			FIELD_GET(IRQ_DONE, status) ? PIODMA_OK : PIODMA_ERR);

	return IRQ_HANDLED;
}

static void watchdog_func(struct work_struct *work)
{
	struct piodma_ctx *ctx;

	ctx = container_of(to_delayed_work(work), struct piodma_ctx,
			   err_work);
	if (!ctx)
		return;

	writel(0, ctx->reg + IRQ_EN);

	pio_err("timed out\n");
	dbg_registers(ctx);

	WARN_ON_ONCE(!ctx->cb);
	if (ctx->cb)
		ctx->cb(ctx->cb_prv, PIODMA_ERR);
}

static void configure_bases(struct piodma_ctx *ctx)
{
	for (int i = 0; i < DMA_CFG_PIO_BASE_CNT; i++) {
		writel_relaxed((ctx->res->start + ctx->base[i]) >> 4,
			       ctx->reg + DMA_CFG_PIO_BASE_ADDR_LO(i));
		writel_relaxed((u64)(ctx->res->start + ctx->base[i]) >> 36,
			       ctx->reg + DMA_CFG_PIO_BASE_ADDR_HI(i));
	}
}

struct piodma_ctx *piodma_init(struct platform_device * pdev,
			       const struct resource *res,
			       void (*cb)(void *prv,
					  enum piodma_completion_status status),
			       void *prv)
{
	int irq, ret, i;
	struct piodma_ctx *ctx = devm_kzalloc(&pdev->dev,
					      sizeof(struct piodma_ctx),
					      GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-EINVAL);

	ctx->dev = &pdev->dev;
	ctx->cb_prv = prv;
	ctx->cb = cb;
	ctx->res = res;

	ctx->reg = devm_platform_ioremap_resource_byname(pdev, "piodma");
	if (IS_ERR(ctx->reg))
		return ctx->reg;

	irq = platform_get_irq_byname(pdev, "piodma");
	if (irq < 0)
		return ERR_PTR(irq);

	ret = devm_request_irq(ctx->dev, irq, piodma_irq_handler, IRQF_SHARED,
			       "piodma-irq", ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to request piodma irq\n");
		return ERR_PTR(ret);
	}

	INIT_DELAYED_WORK(&ctx->err_work, watchdog_func);

	ctx->buf.size = res->end - res->start;
	ctx->buf.cpu =
		dma_alloc_coherent(ctx->dev, ctx->buf.size, &ctx->buf.addr,
				   GFP_KERNEL);
	if (!ctx->buf.cpu)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < DMA_CFG_PIO_BASE_CNT; i++) {
		if (i * PK_MAX_OFF >= ctx->buf.size)
			break;
		ctx->base[i] = i * PK_MAX_OFF;
	}

	pio_dbg(1, "iova %#12llx size %#6lx", ctx->buf.addr, ctx->buf.size);
	pio_dbg(0, "HW Version %#x", readl(ctx->reg + HW_VERSION));

	return ctx;
}

int piodma_start_packet(struct piodma_ctx *ctx)
{
	ctx->cnt = 0;
	ctx->cmd_cnt = 0;
	memset(ctx->buf.cpu, 0, ctx->buf.size);

	return 0;
}

int piodma_add_packet(struct piodma_ctx *ctx, size_t off, const u32 *data,
		      size_t count)
{
	u8 sel = 255;

	if (WARN_ON_ONCE(off % 4))
		return -EINVAL;

	if (ctx->cnt + count + 2 > ctx->buf.size >> 2)
		return -ENOMEM;

	for (int i = 0; i < DMA_CFG_PIO_BASE_CNT; i++) {
		if (ctx->base[i] < off && off - ctx->base[i] < PK_MAX_OFF)
			sel = i;
	}

	if (sel >= DMA_CFG_PIO_BASE_CNT)
		return -EINVAL;

	if ((ctx->cnt % (PK_SIZE >> 2)) + count + 4 > PK_SIZE >> 2) {
		u64 link_addr = ctx->buf.addr + PK_SIZE;

		ctx->cnt = ALIGN(ctx->cnt, PK_SIZE >> 2);
		ctx->buf.cpu[ctx->cnt - 4] = FIELD_PREP(PK_TYPE, PK_TYPE_LINK) |
			PK_LINK_FLAG;
		ctx->buf.cpu[ctx->cnt - 3] = link_addr & 0xffffffff;
		ctx->buf.cpu[ctx->cnt - 2] = (link_addr >> 32) & 0xffffffff;
		pio_dbg(4, "PK new pkt @ %#12llx\n", link_addr);
	}

	pio_dbg(3, "PK num %#4lx cnt %#12lx size %#08lx dst %#12lx\n",
		ctx->cmd_cnt, ctx->cnt, 1 + count, off);

	if (ctx->cnt + count + 2 > ctx->buf.size >> 2)
		return -ENOMEM;

	ctx->buf.cpu[ctx->cnt++] = FIELD_PREP(PK_TYPE, PK_TYPE_RW) |
		FIELD_PREP(PK_RW_CNT, count - 1) |
		FIELD_PREP(PK_RW_START,
			   (off - ctx->base[sel]) >> 2) |
		FIELD_PREP(PK_RW_BASE_SEL, sel);

	memcpy(&ctx->buf.cpu[ctx->cnt], data,  count * 4);
	ctx->cnt += count;
	ctx->cmd_cnt ++;

	return 0;
}

int piodma_submit_packet(struct piodma_ctx *ctx)
{
	size_t packets = ctx->cnt / (PK_SIZE >> 2);

	pio_dbg(2, "submitting %#04lx pkt %#08lx words @ %#12llx\n",
		 ctx->cmd_cnt, ctx->cnt, ctx->buf.addr);

	/* go through the 'link packets' and set their sizes */
	for (int i = 1; i <= packets; i++) {
		size_t pkt_size = min(PK_SIZE >> 2,
				      ctx->cnt - (PK_SIZE >> 2) * i);

		pio_dbg(4, "PK %d size %#04lx\n", i, pkt_size);
		ctx->buf.cpu[i * (PK_SIZE >> 2) - 4] |=
			FIELD_PREP(PK_LINK_SIZE, pkt_size);
	}

	writel(0xffffffff, ctx->reg + IRQ_EN);
	writel(0, ctx->reg + CFG); /* this is part of the avd tunables */

	/* what does this do? */
	writel(0xfff2, ctx->reg + DMA_CFG_MEM_SRC);
	writel(0xfff2, ctx->reg + DMA_CFG_MEM_DAT); /* DAT? */
	writel(0xfff3, ctx->reg + DMA_CFG_MEM_DST);
	writel(0xfff0, ctx->reg + DMA_CFG_PIO_RD);
	writel(0xfff0, ctx->reg + DMA_CFG_PIO_WR);

	configure_bases(ctx);

	writel((u32)ctx->buf.addr, ctx->reg + MEM_SRC_ADDR_LO);
	writel((u32)((u64)ctx->buf.addr >> 32), ctx->reg + MEM_SRC_ADDR_HI);
	writel(2, ctx->reg + CMD_CTX);

	writel(CMD_CTRL_RUN | CMD_CTRL_IRQ |
		FIELD_PREP(CMD_CTRL_TYPE, 0) |
		FIELD_PREP(CMD_CTRL_CNT, min(PK_SIZE >> 2, ctx->cnt)),
	       ctx->reg + CMD_CTRL);

	schedule_delayed_work(&ctx->err_work, msecs_to_jiffies(2000));
	return 0;
}

void piodma_free(struct piodma_ctx *ctx)
{
	if (!ctx)
		return;
	if (ctx->buf.cpu)
		dma_free_coherent(ctx->dev, ctx->buf.size, ctx->buf.cpu,
				  ctx->buf.addr);
	memset(&ctx->buf, 0, sizeof(ctx->buf));
	pio_dbg(0, "bye\n");
}
