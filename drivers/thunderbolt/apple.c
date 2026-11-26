// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Silicon USB4 and Thunderbolt driver
 * Copyright (c) Sven Peter <sven@kernel.org>
 *
 * This driver implements the Host Router / ACIO coprocessor as well as the
 * Native Host Interface (NHI) as shown in the diagram below.
 * The ACIO is a Cortex-M3 coprocessor which handles the Thunderbolt
 * protocol and exposes its own hardware blocks like the USB4 Native Host
 * Interface (NHI) and its IOMMU to the main SoC bus. The entire block
 * can only be brought up after the unified Type-C PHY has been initialized
 * to Thunderbolt or USB4 mode and needs an out-of-band notification from
 * the Type-C PD driver.
 *
 * +--------------------+
 * |                    |  +--------------------------------------------+
 * | Display Controller |  | Host Router / ACIO                         |
 * |       dcpext0      |  |             +----------------+             |
 * |                    |  |             |     Native     |             |
 * +--------+-----------+  |             |      Host      |             |
 *          |              |             |    Interface   |             |
 *          |              +---------+   +----------------+   +---------+     +--------+
 *          |   +--------->| DP IN   |                        | PCIE DN |<--->| APCIEC |
 *          v   |          | Adapter |   +----------------+   | Adapter |     +--------+
 *    +---------+-+        +---------+   |                |   +---------+
 *    | Display   |        |             |  IOMMU / DART  |             |
 *    | Crossbar  |        |             |                |   +---------+     +------+
 *    +---------+-+        +---------+   +----------------+   | USB3    |<--->| DWC3 |
 *          ^   |          | DP IN   |                        | Adapter |     +------+
 *          |   +--------->| Adapter |                        +---------+
 *          |              +---------+                                  |
 *          |              |                                            |
 *          |              |                 +----------+               |
 * +--------+-----------+  |                 | Type C   |               |
 * |                    |  |                 | Adapter  |               |
 * | Display Controller |  +-----------------+----------+---------------+
 * |       dcpext1      |                     ^       ^
 * |                    |                     |       |
 * +--------------------+                     |       |
 *                                            v       |
 *                                 +--------------+   | SBRX/TX
 *                                 | Apple Type-C |   |
 *                                 |    PHY       |   |
 *                                 +--------------+   |
 *                                       ^            |
 *                                       |            v
 *                                       |     +---------+
 *                                       +---->| Type C  |
 *                                     SSRX/TX | Port    |
 *                                             +---------+
 *
 * ACIO and ATCPHY have strict ordering requirements. When a USB4/Thunderbolt
 * connection is requested after a device has been connected the Type-C PD
 * driver performs the following sequence:
 *
 * 1) Configure the PHY for the negotiated mode and orientation through
 *    typec_mux_set()
 * 2) Pass the cable details to this driver through typec_thunderbolt_switch_set()
 * 3) Power up ACIO, boot its RTKit co-processor and apply the tunables
 * 4) Probe the DART and NHI children now that their MMIO is accessible from
 *    the main SoC bus
 * 5) Register the USB4 domain and write the cable details to the host router
 *    which brings up the link and starts discovery
 *
 * The reverse order is required when the cable is disconnected then:
 *
 * 1) typec_thunderbolt_switch_set() removes the USB4 domain and the ACIO
 *    children, shuts down the co-processor and powers everything off.
 * 2) typec_mux_set() puts the PHY into safe mode and powers it off.
 *
 * ACIO must not be started before the PHY is configured and the PHY must
 * remain active until ACIO has been powered down again. The child devices
 * cannot  remain populated while ACIO is powered off because they are part
 * of the ACIO block and only exposed to the main SoC bus and will SError
 * once ACIO is off.
 * Violating this ordering can panic the kernel with an async SError at best
 * and trigger some internal watchdog that will reset the entire SoC at worst.
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/reset.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/tunable.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/usb/pd.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>

#include "nhi.h"
#include "tb.h"

#define APPLE_CIO_M3_CTRL 0x0c
#define APPLE_CIO_M3_CTRL_START BIT(1)
#define APPLE_CIO_M3_STAT 0xa8
#define APPLE_CIO_M3_STAT_STATE GENMASK(30, 24)

#define APPLE_CIO_NHI_HOP_COUNT 0x0
#define APPLE_CIO_NHI_HOP_COUNT_MASK GENMASK(9, 0)

#define APPLE_CIO_NHI_TXRING_DESC_BASE 0x10000
#define APPLE_CIO_NHI_RXRING_DESC_BASE 0x80000
#define APPLE_CIO_NHI_RING_STRIDE 0x4000

#define APPLE_CIO_NHI_PDF_STRIDE 0x4000

#define APPLE_CIO_NHI_IRQ_STATUS 0xd0000
#define APPLE_CIO_NHI_IRQ_ENABLE 0xd0010
#define APPLE_CIO_NHI_IRQ_THROTTLE 0xd004c
#define APPLE_CIO_NHI_IRQ_THROTTLE_INTERVAL_MASK GENMASK(15, 0)
#define APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC 256

#define APPLE_CIO_SRAM_IOVA_BASE 0x10000000

#define APPLE_CIO_NHI_BOOT_TIMEOUT_MS 10000

/*
 * The Apple vendor-specific extended capability contains a cable-information
 * word which has to be programmed from the USB4/Thunderbolt details reported
 * out-of-band by the Type-C PD controller. It must be programmed into the
 * host router before link discovery starts.
 */
#define TB_VSE_CAP_APPLE 0x00
#define TB_VSE_CAP_APPLE_CABLE_INFO 0x01
#define TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT BIT(0)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE BIT(1)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE BIT(2)
#define TB_VSE_CAP_APPLE_CABLE_INFO_BIDIR_LSRX BIT(3)
#define TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS BIT(4)
#define TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER BIT(9)
#define TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3 BIT(10)

/**
 * struct apple_cio - Apple Converged I/O host router complex
 * @dev: ACIO device
 * @np: ACIO device tree node
 * @rtk: RTKit instance for the ACIO co-processor
 * @rc_base: ACIO root controller registers
 * @rc_res: ACIO root controller MMIO resource
 * @rc_tunable: Tunable sequence for the ACIO root controller
 * @sram_res: ACIO co-processor SRAM resource
 * @sram_base: ACIO co-processor SRAM
 * @reset: ACIO reset controller
 * @pd_list: Power domains used by the ACIO complex
 * @lock: Serializes cable transitions and ACIO power changes
 * @current_cable_info: Cable information currently programmed into ACIO
 * @target_cable_info: Cable information requested by the Type-C PD driver
 * @nhi_boot_completion: Signals completion of the NHI bringup
 * @nhi_boot_status: Status returned by the NHI bringup
 * @tbt_switch: USB4/Thunderbolt switch triggered by the Type-C PD driver
 */
struct apple_cio {
	struct device *dev;
	struct device_node *np;
	struct apple_rtkit *rtk;

	void __iomem *rc_base;
	struct resource *rc_res;
	struct apple_tunable *rc_tunable;

	struct resource *sram_res;
	void __iomem *sram_base;

	struct reset_control *reset;

	struct dev_pm_domain_list *pd_list;

	struct mutex lock;

	u32 current_cable_info;
	u32 target_cable_info;
	struct completion nhi_boot_completion;
	int nhi_boot_status;

	struct typec_thunderbolt_switch_dev *tbt_switch;
};

/**
 * struct apple_nhi - Apple Native Host Interface
 * @dev: NHI device
 * @pdev: NHI platform device
 * @np: NHI device tree node
 * @acio: Parent ACIO complex
 * @tb: USB4 domain and software connection manager
 * @nhi: NHI struct
 * @nhi_base: NHI registers
 * @pdf_base: PDF (Protocol Defined Field) configuration registers
 * @tx_irqs: Transmit ring interrupts indexed by HopID
 * @rx_irqs: Receive ring interrupts indexed by HopID
 * @tx_irq_names: Names of the transmit ring interrupts
 * @rx_irq_names: Names of the receive ring interrupts
 * @n_rings: Number of rings in each direction
 */
struct apple_nhi {
	struct device *dev;
	struct platform_device *pdev;
	struct device_node *np;

	struct apple_cio *acio;

	struct tb *tb;
	struct tb_nhi nhi;

	void __iomem *nhi_base;
	void __iomem *pdf_base;

	int *tx_irqs;
	int *rx_irqs;
	const char **tx_irq_names;
	const char **rx_irq_names;
	size_t n_rings;
};

#define nhi_to_anhi(nhi_) container_of((nhi_), struct apple_nhi, nhi)

/**
 * apple_cio_rtkit_shmem_setup - Map an RTKit shared memory buffer
 * @cookie: ACIO instance passed to RTKit
 * @bfr: RTKit shared memory buffer descriptor
 *
 * Translate the firmware-provided IOVA into the ACIO SRAM mapping and reject
 * buffers that fall outside the reserved SRAM resource.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_rtkit_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_cio *acio = cookie;
	struct resource res = {
		.name = "acio_rtkit_buffer",
		.flags = acio->sram_res->flags,
	};

	if (!bfr->iova)
		return -EIO;

	if (bfr->iova < APPLE_CIO_SRAM_IOVA_BASE) {
		dev_err(acio->dev, "firmware requested invalid buffer before SRAM IOVA base 0x%llx\n",
			bfr->iova);
		return -EFAULT;
	}

	res.start = bfr->iova - APPLE_CIO_SRAM_IOVA_BASE + acio->sram_res->start;
	res.end = res.start + bfr->size - 1;

	if (res.end < res.start) {
		dev_err(acio->dev, "firmware requested invalid buffer %pR\n", &res);
		return -EFAULT;
	}

	if (!resource_contains(acio->sram_res, &res)) {
		dev_err(acio->dev, "firmware requested buffer %pR outside SRAM %pR\n", &res,
			acio->sram_res);
		return -EFAULT;
	}

	bfr->iomem = acio->sram_base + (res.start - acio->sram_res->start);
	bfr->is_mapped = true;
	return 0;
}

static const struct apple_rtkit_ops apple_cio_rtkit_ops = {
	.shmem_setup = apple_cio_rtkit_shmem_setup,
};

static int apple_nhi_probe_irqs(struct apple_nhi *anhi)
{
	int n_irqs;
	char name[64];

	n_irqs = platform_irq_count(anhi->pdev);
	if (n_irqs < 0)
		return dev_err_probe(anhi->dev, n_irqs, "platform_irq_count failed\n");
	if (n_irqs == 0) {
		dev_err(anhi->dev, "No interrupts found\n");
		return -EINVAL;
	}
	if (n_irqs % 2) {
		dev_err(anhi->dev, "Invalid number of interrupts: %d must be even\n", n_irqs);
		return -EINVAL;
	}
	anhi->n_rings = n_irqs / 2;

	anhi->rx_irqs = devm_kcalloc(anhi->dev, anhi->n_rings, sizeof(*anhi->rx_irqs), GFP_KERNEL);
	if (!anhi->rx_irqs)
		return -ENOMEM;
	anhi->tx_irqs = devm_kcalloc(anhi->dev, anhi->n_rings, sizeof(*anhi->tx_irqs), GFP_KERNEL);
	if (!anhi->tx_irqs)
		return -ENOMEM;
	anhi->rx_irq_names = devm_kcalloc(anhi->dev, anhi->n_rings,
					  sizeof(*anhi->rx_irq_names), GFP_KERNEL);
	if (!anhi->rx_irq_names)
		return -ENOMEM;
	anhi->tx_irq_names = devm_kcalloc(anhi->dev, anhi->n_rings,
					  sizeof(*anhi->tx_irq_names), GFP_KERNEL);
	if (!anhi->tx_irq_names)
		return -ENOMEM;

	for (int i = 0; i < anhi->n_rings; ++i) {
		snprintf(name, sizeof(name), "rxring%d", i);
		anhi->rx_irqs[i] = platform_get_irq_byname(anhi->pdev, name);
		if (anhi->rx_irqs[i] < 0)
			return anhi->rx_irqs[i];
		anhi->rx_irq_names[i] = devm_kasprintf(anhi->dev, GFP_KERNEL, "%s-%s",
						       dev_name(anhi->dev), name);
		if (!anhi->rx_irq_names[i])
			return -ENOMEM;

		snprintf(name, sizeof(name), "txring%d", i);
		anhi->tx_irqs[i] = platform_get_irq_byname(anhi->pdev, name);
		if (anhi->tx_irqs[i] < 0)
			return anhi->tx_irqs[i];
		anhi->tx_irq_names[i] = devm_kasprintf(anhi->dev, GFP_KERNEL, "%s-%s",
						       dev_name(anhi->dev), name);
		if (!anhi->tx_irq_names[i])
			return -ENOMEM;
	}

	return 0;
}

static unsigned int apple_cio_ring_index(struct tb_ring *ring)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);

	if (ring->is_tx)
		return ring->hop;
	else
		return ring->hop + anhi->n_rings;
}

static void apple_nhi_ring_interrupt_active(struct tb_ring *ring, bool active)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);
	u32 reg, interval;

	lockdep_assert_held(&ring->nhi->lock);

	if (active && ring->interval_nsec) {
		interval = min_t(u32, ring->interval_nsec,
				 FIELD_MAX(APPLE_CIO_NHI_IRQ_THROTTLE_INTERVAL_MASK) *
				 APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC);
		interval = DIV_ROUND_UP(interval,
					APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC);
		writel(interval, anhi->nhi_base + APPLE_CIO_NHI_IRQ_THROTTLE +
				 4 * idx);
	}

	reg = readl(anhi->nhi_base + APPLE_CIO_NHI_IRQ_ENABLE);

	if (active)
		reg |= BIT(idx);
	else
		reg &= ~BIT(idx);

	writel(reg, anhi->nhi_base + APPLE_CIO_NHI_IRQ_ENABLE);
}

static void apple_nhi_ring_interrupt_mask(struct tb_ring *ring, bool mask)
{
	apple_nhi_ring_interrupt_active(ring, !mask);
}

static irqreturn_t apple_cio_ring_irq(int irq, void *data)
{
	struct tb_ring *ring = data;
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);

	guard(spinlock)(&ring->nhi->lock);
	guard(spinlock)(&ring->lock);

	writel(BIT(idx), anhi->nhi_base + APPLE_CIO_NHI_IRQ_STATUS);
	if (!ring->running)
		return IRQ_HANDLED;

	if (ring->start_poll) {
		apple_nhi_ring_interrupt_mask(ring, true);
		ring->start_poll(ring->poll_data);
	} else {
		schedule_work(&ring->work);
	}

	return IRQ_HANDLED;
}


static int apple_nhi_request_irq(struct tb_ring *ring, bool no_suspend)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	const char *name;

	if (ring->is_tx) {
		ring->irq = anhi->tx_irqs[ring->hop];
		name = anhi->tx_irq_names[ring->hop];
	} else {
		ring->irq = anhi->rx_irqs[ring->hop];
		name = anhi->rx_irq_names[ring->hop];
	}

	return devm_request_irq(anhi->dev, ring->irq, apple_cio_ring_irq,
				no_suspend ? IRQF_NO_SUSPEND : 0, name, ring);
}

static void apple_nhi_release_irq(struct tb_ring *ring)
{
	if (ring->irq <= 0)
		return;

	devm_free_irq(ring->nhi->dev, ring->irq, ring);
	ring->irq = 0;
}

static void apple_nhi_ring_configure(struct tb_ring *ring, u32 flags, u32 e2e_flags)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	void __iomem *options;
	u32 sof_eof_mask;

	lockdep_assert_held(&ring->lock);

	options = anhi->nhi_base + ring->hop * APPLE_CIO_NHI_RING_STRIDE + 0x10;

	if (ring->is_tx) {
		options += APPLE_CIO_NHI_TXRING_DESC_BASE;

		/*
		 * All TX rings share what macOS calls a shared buffer with 232 entries. This is how
		 * macOS splits it up, ring 0 only carries control packets and gets the minimum.
		 */
		if (ring->hop == 0)
			writel(2, options + 4);
		else if (ring->hop <= 5)
			writel(40, options + 4);
		else
			writel(5, options + 4);
	} else {
		options += APPLE_CIO_NHI_RXRING_DESC_BASE;

		sof_eof_mask = ring->sof_mask << 16 | ring->eof_mask;
		writel(sof_eof_mask, options + 4);
		writel(sof_eof_mask, anhi->pdf_base + ring->hop * APPLE_CIO_NHI_PDF_STRIDE);
	}

	/*
	 * The firmware samples the ring configuration when the valid bit is set and E2E flow
	 * control never engages when configured afterwards. Write everything at once like macOS.
	 */
	writel(flags | e2e_flags, options);
}

static const struct tb_nhi_ops apple_nhi_ops = {
	.request_ring_irq = apple_nhi_request_irq,
	.release_ring_irq = apple_nhi_release_irq,
	.ring_interrupt_active = apple_nhi_ring_interrupt_active,
	.ring_interrupt_mask = apple_nhi_ring_interrupt_mask,
	.ring_configure = apple_nhi_ring_configure,
};

static const struct tb_nhi_ring_layout apple_nhi_ring_layout = {
	.tx_desc_base = APPLE_CIO_NHI_TXRING_DESC_BASE,
	.rx_desc_base = APPLE_CIO_NHI_RXRING_DESC_BASE,
	.desc_stride = APPLE_CIO_NHI_RING_STRIDE,
	.tx_options_base = APPLE_CIO_NHI_TXRING_DESC_BASE + 0x10,
	.rx_options_base = APPLE_CIO_NHI_RXRING_DESC_BASE + 0x10,
	.options_stride = APPLE_CIO_NHI_RING_STRIDE,
};

static int apple_nhi_probe(struct platform_device *pdev)
{
	struct apple_cio *acio = dev_get_drvdata(pdev->dev.parent);
	struct apple_nhi *anhi;
	struct apple_tunable *tunable;
	struct resource *res;
	struct tb_port *port;
	int cap_apple;
	int ret = 0;

	anhi = devm_kzalloc(&pdev->dev, sizeof(*anhi), GFP_KERNEL);
	if (!anhi) {
		ret = -ENOMEM;
		goto err;
	}

	anhi->pdev = pdev;
	anhi->dev = &pdev->dev;
	anhi->np = pdev->dev.of_node;
	anhi->acio = acio;
	platform_set_drvdata(pdev, anhi);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(42));
	if (ret)
		goto err;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nhi");
	anhi->nhi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(anhi->nhi_base)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(anhi->nhi_base),
				    "Unable to map NHI regs\n");
		goto err;
	}
	tunable = devm_apple_tunable_parse(&pdev->dev, anhi->np, "apple,tunable-nhi", res);
	if (IS_ERR(tunable)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(tunable), "Unable to load NHI tunable\n");
		goto err;
	}
	apple_tunable_apply(anhi->nhi_base, tunable);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pdf");
	anhi->pdf_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(anhi->pdf_base)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(anhi->pdf_base),
				    "Unable to map PDF regs\n");
		goto err;
	}

	ret = apple_nhi_probe_irqs(anhi);
	if (ret)
		goto err;

	spin_lock_init(&anhi->nhi.lock);
	anhi->nhi.iommu_dma_protection = true;
	anhi->nhi.ops = &apple_nhi_ops;
	anhi->nhi.ring_layout = &apple_nhi_ring_layout;
	anhi->nhi.iobase = anhi->nhi_base;
	anhi->nhi.quirks = QUIRK_NO_DMA_PORT | QUIRK_NO_USB3_BW_ALLOC;

	anhi->nhi.hop_count = readl(anhi->nhi_base + APPLE_CIO_NHI_HOP_COUNT) &
			      APPLE_CIO_NHI_HOP_COUNT_MASK;
	if (anhi->nhi.hop_count != anhi->n_rings) {
		ret = dev_err_probe(anhi->dev, -EINVAL, "Ring IRQs (%zd) != HOP_COUNT (%d)\n",
				    anhi->n_rings, anhi->nhi.hop_count);
		goto err;
	}

	anhi->nhi.tx_rings = devm_kcalloc(&pdev->dev, anhi->nhi.hop_count,
					  sizeof(*anhi->nhi.tx_rings), GFP_KERNEL);
	anhi->nhi.rx_rings = devm_kcalloc(&pdev->dev, anhi->nhi.hop_count,
					  sizeof(*anhi->nhi.rx_rings), GFP_KERNEL);
	if (!anhi->nhi.tx_rings || !anhi->nhi.rx_rings) {
		ret = -ENOMEM;
		goto err;
	}

	anhi->nhi.dev = &pdev->dev;
	init_completion(&anhi->nhi.domain_released);
	anhi->tb = tb_probe(&anhi->nhi);
	if (!anhi->tb) {
		ret = dev_err_probe(anhi->dev, -ENODEV,
				    "Failed to init software connection manager\n");
		goto err;
	}

	ret = tb_domain_add(anhi->tb, false);
	if (ret) {
		dev_err_probe(anhi->dev, ret, "failed to add TB domain\n");
		tb_domain_put(anhi->tb);
		wait_for_completion(&anhi->nhi.domain_released);
		goto err;
	}

	mutex_lock(&anhi->tb->lock);

	if (!anhi->tb->root_switch->drom) {
		dev_err(anhi->dev, "No valid host DROM in the device tree\n");
		ret = -EINVAL;
		goto err_unlock_tb_domain;
	}

	cap_apple = tb_switch_find_vse_cap(anhi->tb->root_switch, TB_VSE_CAP_APPLE);

	if (cap_apple < 0) {
		dev_err(anhi->dev, "Unable to find VSE Apple capability: %d\n",
			cap_apple);
		ret = cap_apple;
		goto err_unlock_tb_domain;
	}

	/*
	 * The ports are locked after reset. Unlock them before writing the cable information which
	 * will bring up the link and start the first scan such that XDomain responses are not
	 * rejected by our own router.
	 */
	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_null(port))
			continue;
		ret = tb_port_unlock(port);
		if (ret)
			dev_warn(anhi->dev, "Failed to unlock port %d: %d\n",
				 port->port, ret);
	}

	ret = tb_sw_write(anhi->tb->root_switch, &acio->target_cable_info, TB_CFG_SWITCH,
			  cap_apple + TB_VSE_CAP_APPLE_CABLE_INFO, 1);
	if (ret) {
		dev_warn(anhi->dev, "Setting VSE Apple cable info failed: %d\n", ret);
		goto err_unlock_tb_domain;
	}

	mutex_unlock(&anhi->tb->lock);

	acio->nhi_boot_status = 0;
	complete(&acio->nhi_boot_completion);
	return 0;

err_unlock_tb_domain:
	mutex_unlock(&anhi->tb->lock);
	tb_domain_remove(anhi->tb);
	wait_for_completion(&anhi->nhi.domain_released);
err:
	acio->nhi_boot_status = ret;
	complete(&acio->nhi_boot_completion);
	return ret;
}

static void apple_nhi_remove(struct platform_device *pdev)
{
	struct apple_nhi *anhi = platform_get_drvdata(pdev);

	tb_domain_remove(anhi->tb);
	wait_for_completion(&anhi->nhi.domain_released);
}

static const struct of_device_id apple_nhi_match[] = {
	{
		.compatible = "apple,t8103-usb4-nhi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_nhi_match);

static struct platform_driver apple_nhi_driver = {
	.driver = {
		.name = "thunderbolt-apple-nhi",
		.of_match_table = apple_nhi_match,
	},
	.probe = apple_nhi_probe,
	.remove = apple_nhi_remove,
};

/**
 * apple_cio_stop - Stop and power down the ACIO complex
 * @acio: ACIO complex to stop
 *
 * Remove the NHI and DART children before shutting down the co-processor and
 * dropping the power-domain links. The Type-C PHY must remain configured
 * until this function returns. The caller must hold @acio->lock.
 */
static void apple_cio_stop(struct apple_cio *acio)
{
	int ret, i;

	lockdep_assert_held(&acio->lock);

	/*
	 * First, shutdown the blocks inside the ACIO complex, like the NHI and the IOMMU.
	 * After we shut down the ACIO co-processor we will no longer be able to access
	 * the MMIO space of these so make sure nothing tries to do just that.
	 */
	of_platform_depopulate(acio->dev);

	/* Try to shut down and power off the co-processor gracefully */
	ret = apple_rtkit_poweroff(acio->rtk);
	if (ret)
		dev_warn(acio->dev,
			 "Failed to shutdown M3 RTKit, continuing ACIO shutdown anyway\n");
	apple_rtkit_free(acio->rtk);

	/* Finally, remove the links to the PD domains to power everything off */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		if (acio->pd_list->pd_links[i])
			device_link_del(acio->pd_list->pd_links[i]);
		acio->pd_list->pd_links[i] = NULL;
	}

	acio->current_cable_info = 0;
}

/**
 * apple_cio_start - Power up and start the ACIO complex
 * @acio: ACIO complex to start
 *
 * The Type-C PHY must already be configured for USB4 or Thunderbolt. Power up
 * ACIO, boot its RTKit co-processor, populate its child devices and wait for
 * the NHI to register the USB4 domain. All completed steps are unwound on
 * failure. The caller must hold @acio->lock.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_start(struct apple_cio *acio)
{
	struct device_link *link;
	int i, ret;
	u32 state;

	lockdep_assert_held(&acio->lock);

	/* Create device links to the power domains in order to power them on */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		link = device_link_add(acio->dev, acio->pd_list->pd_devs[i],
				       DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (!link) {
			ret = -ENODEV;
			goto remove_links;
		}
		acio->pd_list->pd_links[i] = link;
	}

	/*
	 * After the power domains are on we need to signal and wait for the ACIO block
	 * to actually start before we can bring up the co-processor.
	 */
	ret = reset_control_deassert(acio->reset);
	if (ret) {
		dev_err(acio->dev, "ACIO block failed to start: %d\n", ret);
		goto remove_links;
	}

	/* Start and wait for the co-processor to boot */
	writel(APPLE_CIO_M3_CTRL_START, acio->rc_base + APPLE_CIO_M3_CTRL);
	acio->rtk = apple_rtkit_init(acio->dev, acio, NULL, 0, &apple_cio_rtkit_ops);
	if (IS_ERR(acio->rtk)) {
		ret = PTR_ERR(acio->rtk);
		dev_err(acio->dev, "Failed to initialize RTKit: %d\n", ret);
		goto remove_links;
	}

	ret = apple_rtkit_boot(acio->rtk);
	if (ret) {
		dev_err(acio->dev, "M3 RTKit failed to boot: %d\n", ret);
		goto err_free_rtkit;
	}

	ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
				 state & APPLE_CIO_M3_STAT_STATE, 100, 500000);
	if (ret < 0) {
		dev_err(acio->dev, "M3 firmware failed to get ready: %d\n", ret);
		goto err_shutdown_rtkit;
	}

	apple_tunable_apply(acio->rc_base, acio->rc_tunable);

	/*
	 * Bring up devices which are part of ACIO and are now accessible by the main SoC
	 * and specifically wait for the NHI to be up to prevent concurrent shutdowns.
	 */
	reinit_completion(&acio->nhi_boot_completion);
	ret = of_platform_populate(acio->np, NULL, NULL, acio->dev);
	if (ret) {
		dev_err(acio->dev, "failed to populate children: %d\n", ret);
		goto err_depopulate;
	}

	if (!wait_for_completion_timeout(&acio->nhi_boot_completion,
					 msecs_to_jiffies(APPLE_CIO_NHI_BOOT_TIMEOUT_MS))) {
		dev_err(acio->dev, "Timed out waiting for the NHI to come up\n");
		ret = -ETIMEDOUT;
		goto err_depopulate;
	}
	if (acio->nhi_boot_status) {
		ret = acio->nhi_boot_status;
		goto err_depopulate;
	}

	acio->current_cable_info = acio->target_cable_info;
	return 0;

err_depopulate:
	of_platform_depopulate(acio->dev);
err_shutdown_rtkit:
	/* Ignore errors here since we're about to cut power to the entire block anyway */
	apple_rtkit_poweroff(acio->rtk);
err_free_rtkit:
	apple_rtkit_free(acio->rtk);
remove_links:
	/* Cut power to reset the entire block  */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		if (acio->pd_list->pd_links[i])
			device_link_del(acio->pd_list->pd_links[i]);
		acio->pd_list->pd_links[i] = NULL;
	}
	return ret;
}

/**
 * apple_cio_tbt_switch_set - Handle a USB4/Thunderbolt cable transition
 * @sw: USB4/Thunderbolt switch
 * @data: New cable state and negotiated cable details
 *
 * Translate the cable details into the Apple host-router representation and
 * synchronously power ACIO up or down. The Type-C PD driver must configure
 * the PHY before an active transition and must request the inactive state
 * before shutting the PHY down.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_tbt_switch_set(struct typec_thunderbolt_switch_dev *sw,
				    const struct typec_thunderbolt_switch_data *data)
{
	struct apple_cio *acio = typec_thunderbolt_switch_get_drvdata(sw);

	guard(mutex)(&acio->lock);

	dev_dbg(acio->dev, "set cable state: %d\n", data->state);

	switch (data->state) {
	case TYPEC_THUNDERBOLT_SWITCH_OFF:
		acio->target_cable_info = 0;
		break;
	case TYPEC_THUNDERBOLT_SWITCH_TBT:
		acio->target_cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3;
		if (data->tbt.cable_mode & TBT_CABLE_ACTIVE_PASSIVE) {
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE;
			if (!(data->tbt.cable_mode & TBT_CABLE_LINK_TRAINING))
				acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_BIDIR_LSRX;
		}
		/* bit 16 of the Device Discover Mode VDO is 1 for a legacy TBT2 adapter */
		if (TBT_ADAPTER(data->tbt.device_mode))
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER;
		if (TBT_CABLE_SPEED(data->tbt.cable_mode) == TBT_CABLE_10_AND_20GBPS)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS;
		if (data->orientation == TYPEC_ORIENTATION_REVERSE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE;
		dev_dbg(acio->dev,
			"TBT cable: cable_mode 0x%x, device_mode 0x%x, enter_vdo 0x%x, orientation %d -> cable info 0x%x\n",
			data->tbt.cable_mode, data->tbt.device_mode,
			data->tbt.enter_vdo, data->orientation,
			acio->target_cable_info);
		break;
	case TYPEC_THUNDERBOLT_SWITCH_USB4:
		acio->target_cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		if (FIELD_GET(EUDO_CABLE_TYPE_MASK, data->usb4.eudo) != EUDO_CABLE_TYPE_PASSIVE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE;
		if (FIELD_GET(EUDO_CABLE_SPEED_MASK, data->usb4.eudo) == EUDO_CABLE_SPEED_USB4_GEN3)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS;
		if (data->orientation == TYPEC_ORIENTATION_REVERSE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE;
		dev_dbg(acio->dev, "USB4 cable: eudo 0x%x, orientation %d -> cable info 0x%x\n",
			data->usb4.eudo, data->orientation, acio->target_cable_info);
		break;
	}

	if (acio->target_cable_info == acio->current_cable_info)
		return 0;

	/*
	 * Transitions between different cables without a shutdown inbetween are invalid and can
	 * only happen when there's a bug inside the Type-C PD driver. If we tried such a
	 * transition, ACIO would crash and then trigger some watchdog that would reset the entire
	 * SoC a few seconds later. Shutting down instead only makes the connected device not work
	 * but we should be able to recover once the next cable is plugged in.
	 */
	if (acio->current_cable_info && acio->target_cable_info) {
		dev_err(acio->dev,
			"Invalid cable transition from 0x%x to 0x%x, shutting down instead\n",
			acio->current_cable_info, acio->target_cable_info);
		acio->target_cable_info = 0;
	}

	/*
	 * Bring up or power down the ACIO complex
	 * current_cable_info will be updated in the start/stop functions
	 */
	if (acio->target_cable_info)
		return apple_cio_start(acio);

	apple_cio_stop(acio);
	return 0;
}

static int apple_cio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_cio *acio;
	int ret;

	acio = devm_kzalloc(dev, sizeof(*acio), GFP_KERNEL);
	if (!acio)
		return -ENOMEM;
	platform_set_drvdata(pdev, acio);

	ret = devm_mutex_init(dev, &acio->lock);
	if (ret)
		return ret;
	init_completion(&acio->nhi_boot_completion);
	acio->dev = &pdev->dev;
	acio->np = dev->of_node;

	acio->sram_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!acio->sram_res)
		return dev_err_probe(dev, -EIO, "Failed to get SRAM resource\n");
	acio->sram_base = devm_ioremap_resource(dev, acio->sram_res);
	if (IS_ERR(acio->sram_base))
		return dev_err_probe(dev, PTR_ERR(acio->sram_base), "Failed to map SRAM\n");

	acio->rc_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc");
	acio->rc_base = devm_ioremap_resource(&pdev->dev, acio->rc_res);
	if (IS_ERR(acio->rc_base))
		return dev_err_probe(dev, PTR_ERR(acio->rc_base), "Unable to map rc regs\n");
	acio->rc_tunable =
		devm_apple_tunable_parse(dev, acio->np, "apple,tunable-rc", acio->rc_res);
	if (IS_ERR(acio->rc_tunable))
		return dev_err_probe(dev, PTR_ERR(acio->rc_tunable), "Unable to load rc tunable\n");

	acio->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(acio->reset))
		return dev_err_probe(dev, PTR_ERR(acio->reset), "Unable to get CIO reset\n");

	/*
	 * If there is only a single domain listed in the device tree the platform driver
	 * framework will already attach it. Thus, if we find an already attached domain
	 * here something's wrong in the device tree because we expect at least three separate
	 * domains that we have to control manually.
	 */
	if (dev->pm_domain) {
		dev_err(dev, "PM domain already attached, check if the DT lists three domains\n");
		return -EINVAL;
	}

	/*
	 * Find and attach the PM domains but don't power them on yet since we must only
	 * do that after the PHY has already been configured into USB4/Thunderbolt mode.
	 */
	struct dev_pm_domain_attach_data pd_data = {
		.pd_flags = PD_FLAG_NO_DEV_LINK,
	};
	ret = devm_pm_domain_attach_list(dev, &pd_data, &acio->pd_list);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Unable to attach PM domains\n");
	else if (ret < 3)
		return dev_err_probe(dev, -EINVAL, "Not enough PM domains\n");

	/* And finally register the OOB notification for Thunderbolt/USB4 cables */
	struct typec_thunderbolt_switch_desc desc = {
		.fwnode = pdev->dev.fwnode,
		.set = apple_cio_tbt_switch_set,
		.drvdata = acio,
	};
	acio->tbt_switch = typec_thunderbolt_switch_register(dev, &desc);
	if (IS_ERR(acio->tbt_switch))
		return dev_err_probe(dev, PTR_ERR(acio->tbt_switch),
				     "Unable to register thunderbolt switch\n");

	return 0;
}

static void apple_cio_remove(struct platform_device *pdev)
{
	struct apple_cio *acio = platform_get_drvdata(pdev);

	typec_thunderbolt_switch_unregister(acio->tbt_switch);

	guard(mutex)(&acio->lock);
	if (acio->current_cable_info)
		apple_cio_stop(acio);
}

static int apple_cio_prepare(struct device *dev)
{
	struct apple_cio *acio = dev_get_drvdata(dev);

	guard(mutex)(&acio->lock);

	if (acio->current_cable_info) {
		dev_err(dev, "unable to suspend while a USB4/Thunderbolt connection is active\n");
		return -EBUSY;
	}

	return 0;
}

static const struct dev_pm_ops apple_cio_pm_ops = {
	.prepare = apple_cio_prepare,
};

static const struct of_device_id apple_acio_match[] = {
	{
		.compatible = "apple,t8103-usb4-acio",
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_acio_match);

static struct platform_driver apple_cio_driver = {
	.driver = {
		.name = "thunderbolt-apple-acio",
		.of_match_table = apple_acio_match,
		.pm = pm_sleep_ptr(&apple_cio_pm_ops),
	},
	.probe = apple_cio_probe,
	.remove = apple_cio_remove,
};

static struct platform_driver * const apple_cio_drivers[] = {
	&apple_nhi_driver,
	&apple_cio_driver,
};

static int __init apple_cio_init(void)
{
	return platform_register_drivers(apple_cio_drivers,
					 ARRAY_SIZE(apple_cio_drivers));
}

static void __exit apple_cio_exit(void)
{
	platform_unregister_drivers(apple_cio_drivers,
				    ARRAY_SIZE(apple_cio_drivers));
}

module_init(apple_cio_init);
module_exit(apple_cio_exit);

MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple Silicon USB4/Thunderbolt driver");
