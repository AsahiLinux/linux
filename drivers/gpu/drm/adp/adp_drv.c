// SPDX-License-Identifier: GPL-2.0-only

#include <linux/anon_inodes.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/file.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <asm/current.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_file.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#define ADP_INT_STATUS 0x34
#define ADP_INT_STATUS_INT_MASK 0x7
#define ADP_INT_STATUS_VBLANK 0x1
#define ADP_CTRL 0x100
#define ADP_CTRL_VBLANK_ON 0x12
#define ADP_CTRL_FIFO_ON 0x601
#define ADP_SCREEN_SIZE 0x0c
#define ADP_SCREEN_HSIZE GENMASK(15, 0)
#define ADP_SCREEN_VSIZE GENMASK(31, 16)

#define ADBE_FIFO 0x10c0
#define ADBE_FIFO_SYNC 0xc0000000

#define ADBE_BLEND_BYPASS 0x2020
#define ADBE_BLEND_EN1 0x2028
#define ADBE_BLEND_EN2 0x2074
#define ADBE_BLEND_EN3 0x202c
#define ADBE_BLEND_EN4 0x2034
#define ADBE_MASK_BUF 0x2200

#define ADBE_SRC_START 0x4040
#define ADBE_SRC_SIZE 0x4048
#define ADBE_DST_START 0x4050
#define ADBE_DST_SIZE 0x4054
#define ADBE_STRIDE 0x4038
#define ADBE_FB_BASE 0x4030

#define ADBE_LAYER_EN1 0x4020
#define ADBE_LAYER_EN2 0x4068
#define ADBE_LAYER_EN3 0x40b4
#define ADBE_LAYER_EN4 0x40f4
#define ADBE_SCALE_CTL 0x40ac
#define ADBE_SCALE_CTL_BYPASS 0x100000

#define ADBE_LAYER_CTL 0x1038
#define ADBE_LAYER_CTL_ENABLE 0x10000

#define ADBE_PIX_FMT 0x402c
#define ADBE_PIX_FMT_XRGB32 0x53e4001

#define DSI_GEN_HDR 0x6c
#define DSI_GEN_PLD_DATA 0x70

#define DSI_CMD_PKT_STATUS 0x74

#define GEN_PLD_R_EMPTY BIT(4)
#define GEN_PLD_W_FULL BIT(3)
#define GEN_PLD_W_EMPTY BIT(2)
#define GEN_CMD_FULL BIT(1)
#define GEN_CMD_EMPTY BIT(0)
#define GEN_RD_CMD_BUSY BIT(6)
#define CMD_PKT_STATUS_TIMEOUT_US 20000

static int adp_open(struct inode *inode, struct file *filp)
{
	/*
	 * The modesetting driver does not check the non-desktop connector
	 * property and keeps the device open and locked. If the touchbar daemon
	 * opens the device first modesetting breaks the whole X session.
	 * Simply refuse to open the device for X11 server processes as
	 * workaround.
	 */
	if (current->comm[0] == 'X')
		return -EBUSY;

	return drm_open(inode, filp);
}

static const struct file_operations adp_fops = {
	.owner          = THIS_MODULE,
	.open           = adp_open,
	.release        = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl   = drm_compat_ioctl,
	.poll           = drm_poll,
	.read           = drm_read,
	.llseek         = noop_llseek,
	.mmap           = drm_gem_mmap,
	DRM_GEM_DMA_UNMAPPED_AREA_FOPS
};

static int adp_drm_gem_dumb_create(struct drm_file *file_priv,
					struct drm_device *drm,
					struct drm_mode_create_dumb *args)
{
	args->height = ALIGN(args->height, 64);
	args->size = args->pitch * args->height;

	return drm_gem_dma_dumb_create_internal(file_priv, drm, args);
}

static const struct drm_driver adp_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &adp_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP_WITH_DUMB_CREATE(adp_drm_gem_dumb_create),
	.name = "adp",
	.desc = "Apple Display Pipe DRM Driver",
	.date = "20230412",
	.major = 0,
	.minor = 1,
};

struct adp_drv_private {
	struct drm_device drm;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct mipi_dsi_host dsi;
	void __iomem *be;
	void __iomem *fe;
	void __iomem *mipi;
	u32 *mask_buf;
	u64 mask_buf_size;
	dma_addr_t mask_iova;
	int be_irq;
	int fe_irq;
	spinlock_t irq_lock;
	struct drm_pending_vblank_event *event;
};

struct adp_plane {
	struct drm_plane base_plane;
	u8 id;
};

#define to_adp(x) container_of(x, struct adp_drv_private, drm)
#define crtc_to_adp(x) container_of(x, struct adp_drv_private, crtc)
#define conn_to_adp(x) container_of(x, struct adp_drv_private, connector)
#define mipi_to_adp(x) container_of(x, struct adp_drv_private, dsi)

static int adp_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state;
	struct drm_crtc_state *crtc_state;

	new_plane_state = drm_atomic_get_new_plane_state(state, plane);

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state, new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);
	return drm_atomic_helper_check_plane_state(new_plane_state,
						   crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   true, true);
}

static void adp_plane_atomic_update(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct adp_drv_private *adp;
	struct drm_rect src_rect;
	struct drm_gem_dma_object *obj;
	struct drm_framebuffer *fb;
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	u32 src_pos, src_size, dst_pos, dst_size;
	if (!plane || !new_state)
		return;

	fb = new_state->fb;
	if (!fb)
		return;
	adp = to_adp(plane->dev);

	drm_rect_fp_to_int(&src_rect, &new_state->src);
	src_pos = src_rect.x1 << 16 | src_rect.y1;
	dst_pos = new_state->dst.x1 << 16 | new_state->dst.y1;
	src_size = drm_rect_width(&src_rect) << 16 | drm_rect_height(&src_rect);
	dst_size = drm_rect_width(&new_state->dst) << 16 |
		drm_rect_height(&new_state->dst);
	writel(src_pos, adp->be + ADBE_SRC_START);
	writel(src_size, adp->be + ADBE_SRC_SIZE);
	writel(dst_pos, adp->be + ADBE_DST_START);
	writel(dst_size, adp->be + ADBE_DST_SIZE);
	writel(fb->pitches[0], adp->be + ADBE_STRIDE);
	obj = drm_fb_dma_get_gem_obj(fb, 0);
	if (obj)
		writel(obj->dma_addr + fb->offsets[0], adp->be + ADBE_FB_BASE);

	writel(0x1, adp->be + ADBE_LAYER_EN1);
	writel(0x1, adp->be + ADBE_LAYER_EN2);
	writel(0x1, adp->be + ADBE_LAYER_EN3);
	writel(0x1, adp->be + ADBE_LAYER_EN4);
	writel(ADBE_SCALE_CTL_BYPASS, adp->be + ADBE_SCALE_CTL);
	writel(ADBE_LAYER_CTL_ENABLE | 0x1, adp->be + ADBE_LAYER_CTL);
	writel(ADBE_PIX_FMT_XRGB32, adp->be + ADBE_PIX_FMT);

}

static void adp_plane_atomic_disable(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct adp_drv_private *adp = to_adp(plane->dev);
	writel(0x0, adp->be + ADBE_LAYER_EN1);
	writel(0x0, adp->be + ADBE_LAYER_EN2);
	writel(0x0, adp->be + ADBE_LAYER_EN3);
	writel(0x0, adp->be + ADBE_LAYER_EN4);
	writel(ADBE_LAYER_CTL_ENABLE, adp->be + ADBE_LAYER_CTL);
}

static const struct drm_plane_helper_funcs adp_plane_helper_funcs = {
	.atomic_check = adp_plane_atomic_check,
	.atomic_update = adp_plane_atomic_update,
	.atomic_disable = adp_plane_atomic_disable,
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS
};

static const struct drm_plane_funcs adp_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	DRM_GEM_SHADOW_PLANE_FUNCS
};

static const u32 plane_formats[] = {
	DRM_FORMAT_XRGB8888,
};

#define ALL_CRTCS 1

static struct adp_plane *adp_plane_new(struct adp_drv_private *adp, u8 id)
{
	struct drm_device *drm = &adp->drm;
	struct adp_plane *plane;
	enum drm_plane_type plane_type;

	plane_type = (id == 0) ? DRM_PLANE_TYPE_PRIMARY :
		DRM_PLANE_TYPE_OVERLAY;

	plane = drmm_universal_plane_alloc(drm, struct adp_plane, base_plane,
					   ALL_CRTCS, &adp_plane_funcs,
					   plane_formats, ARRAY_SIZE(plane_formats),
					   NULL, plane_type, "plane %d", id);
	if (!plane) {
		drm_err(drm, "failed to allocate plane");
		return ERR_PTR(-ENOMEM);
	}
	plane->id = id;

	drm_plane_helper_add(&plane->base_plane, &adp_plane_helper_funcs);
	return plane;
}

static void adp_enable_vblank(struct adp_drv_private *adp)
{
	u32 cur_ctrl;

	writel(ADP_INT_STATUS_INT_MASK, adp->fe + ADP_INT_STATUS);

	cur_ctrl = readl(adp->fe + ADP_CTRL);
	writel(cur_ctrl | ADP_CTRL_VBLANK_ON, adp->fe + ADP_CTRL);
}

static int adp_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct adp_drv_private *adp = to_adp(dev);
	adp_enable_vblank(adp);

	return 0;
}

static void adp_disable_vblank(struct adp_drv_private *adp)
{
	u32 cur_ctrl;

	cur_ctrl = readl(adp->fe + ADP_CTRL);
	writel(cur_ctrl & ~ADP_CTRL_VBLANK_ON, adp->fe + ADP_CTRL);
	writel(ADP_INT_STATUS_INT_MASK, adp->fe + ADP_INT_STATUS);
}

static void adp_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct adp_drv_private *adp = to_adp(dev);

	adp_disable_vblank(adp);
}


static void adp_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct adp_drv_private *adp = crtc_to_adp(crtc);
	writel(0x1, adp->be + ADBE_BLEND_EN2);
	writel(0x10, adp->be + ADBE_BLEND_EN1);
	writel(0x1, adp->be + ADBE_BLEND_EN3);
	writel(0x1, adp->be + ADBE_BLEND_BYPASS);
	writel(0x1, adp->be + ADBE_BLEND_EN4);
}

static void adp_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct adp_drv_private *adp = crtc_to_adp(crtc);
	struct drm_crtc_state *old_state = drm_atomic_get_old_crtc_state(state, crtc);

	drm_atomic_helper_disable_planes_on_crtc(old_state, false);

	writel(0x0, adp->be + ADBE_BLEND_EN2);
	writel(0x0, adp->be + ADBE_BLEND_EN1);
	writel(0x0, adp->be + ADBE_BLEND_EN3);
	writel(0x0, adp->be + ADBE_BLEND_BYPASS);
	writel(0x0, adp->be + ADBE_BLEND_EN4);
	drm_crtc_vblank_off(crtc);
}

static void adp_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	u32 frame_num = 1;
	struct adp_drv_private *adp = crtc_to_adp(crtc);
	struct drm_crtc_state *new_state = drm_atomic_get_new_crtc_state(state, crtc);
	u64 new_size = ALIGN(new_state->mode.hdisplay *
			     new_state->mode.vdisplay * 4, PAGE_SIZE);

	if (new_size != adp->mask_buf_size) {
		if (adp->mask_buf)
			dma_free_coherent(crtc->dev->dev, adp->mask_buf_size,
					  adp->mask_buf, adp->mask_iova);
		adp->mask_buf = NULL;
		if (new_size != 0) {
			adp->mask_buf = dma_alloc_coherent(crtc->dev->dev, new_size,
							   &adp->mask_iova, GFP_KERNEL);
			memset(adp->mask_buf, 0xFF, new_size);
			writel(adp->mask_iova, adp->be + ADBE_MASK_BUF);
		}
		adp->mask_buf_size = new_size;
	}
	writel(ADBE_FIFO_SYNC | frame_num, adp->be + ADBE_FIFO);
	//FIXME: use adbe flush interrupt
	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_vblank_get(crtc);
		adp->event = crtc->state->event;
	}
	crtc->state->event = NULL;
	spin_unlock_irq(&crtc->dev->event_lock);
}

static const struct drm_crtc_funcs adp_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = adp_crtc_enable_vblank,
	.disable_vblank = adp_crtc_disable_vblank,
};


static const struct drm_crtc_helper_funcs adp_crtc_helper_funcs = {
	.atomic_enable = adp_crtc_atomic_enable,
	.atomic_disable = adp_crtc_atomic_disable,
	.atomic_flush = adp_crtc_atomic_flush,
};

static int adp_setup_crtc(struct adp_drv_private *adp)
{
	struct drm_device *drm = &adp->drm;
	struct adp_plane *primary;
	int ret;

	primary = adp_plane_new(adp, 0);
	if (IS_ERR(primary))
		return PTR_ERR(primary);

	ret = drm_crtc_init_with_planes(drm, &adp->crtc, &primary->base_plane,
					NULL, &adp_crtc_funcs, NULL);
	if (ret)
		return ret;

	drm_crtc_helper_add(&adp->crtc, &adp_crtc_helper_funcs);
	return 0;
}

static int adp_get_modes(struct drm_connector *connector)
{
	struct adp_drv_private *adp = conn_to_adp(connector);
	struct drm_display_mode *mode;
	u32 size;

	size = readl(adp->fe + ADP_SCREEN_SIZE);
	mode = drm_mode_create(connector->dev);

	mode->vdisplay = FIELD_GET(ADP_SCREEN_VSIZE, size);
	mode->hdisplay = FIELD_GET(ADP_SCREEN_HSIZE, size);
	mode->hsync_start = mode->hdisplay + 8;
	mode->hsync_end = mode->hsync_start + 80;
	mode->htotal = mode->hsync_end + 40;
	mode->vsync_start = mode->vdisplay + 1;
	mode->vsync_end = mode->vsync_start + 15;
	mode->vtotal = mode->vsync_end + 6;
	mode->clock = (mode->vtotal * mode->htotal * 60) / 1000;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	mode->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	return 1;
}

static int adp_detect_ctx(struct drm_connector *connector,
		   struct drm_modeset_acquire_ctx *ctx,
		   bool force) {
	connector->display_info.non_desktop = true;
	drm_object_property_set_value(&connector->base,
				      connector->dev->mode_config.non_desktop_property,
				      connector->display_info.non_desktop);
	return connector_status_connected;
}

static const struct drm_connector_funcs adp_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs adp_connector_helper_funcs = {
	.get_modes = adp_get_modes,
	.detect_ctx = adp_detect_ctx,
};

static const struct drm_mode_config_funcs adp_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int adp_setup_mode_config(struct adp_drv_private *adp)
{
	struct drm_device *drm = &adp->drm;
	int ret;
	u32 size;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	/*
	 * Query screen size restrict the frame buffer size to the screen size
	 * aligned to the next multiple of 64. This is not necessary but can be
	 * used as simple check for non-desktop devices.
	 * Xorg's modesetting driver does not care about the connector
	 * "non-desktop" property. The max frame buffer width or height can be
	 * easily checked and a device can be reject if the max width/height is
	 * smaller than 120 for example.
	 * Any touchbar daemon is not limited by this small framebuffer size.
	 */
	size = readl(adp->fe + ADP_SCREEN_SIZE);

	drm->mode_config.min_width = 32;
	drm->mode_config.min_height = 32;
	drm->mode_config.max_width = ALIGN(FIELD_GET(ADP_SCREEN_HSIZE, size), 64);
	drm->mode_config.max_height = ALIGN(FIELD_GET(ADP_SCREEN_VSIZE, size), 64);
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.prefer_shadow = 0;
	drm->mode_config.funcs = &adp_mode_config_funcs;

	ret = adp_setup_crtc(adp);
	if (ret) {
		drm_err(drm, "failed to create crtc");
		return ret;
	}

	adp->encoder.possible_crtcs = ALL_CRTCS;
	ret = drm_simple_encoder_init(drm, &adp->encoder, DRM_MODE_ENCODER_DSI);
	if (ret) {
		drm_err(drm, "failed to init encoder");
		return ret;
	}
	drm_connector_helper_add(&adp->connector,
				 &adp_connector_helper_funcs);
	ret = drm_connector_init(drm, &adp->connector, &adp_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ret;

	drm_connector_attach_encoder(&adp->connector, &adp->encoder);

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret < 0) {
		drm_err(drm, "failed to initialize vblank");
		return ret;
	}

	drm_mode_config_reset(drm);

	return 0;
}

static int adp_parse_of(struct platform_device *pdev, struct adp_drv_private *adp)
{
	adp->be = devm_platform_ioremap_resource_byname(pdev, "be");
	if (IS_ERR(adp->be)) {
		dev_err(&pdev->dev, "failed to map display backend mmio");
		return PTR_ERR(adp->be);
	}

	adp->fe = devm_platform_ioremap_resource_byname(pdev, "fe");
	if (IS_ERR(adp->fe)) {
		dev_err(&pdev->dev, "failed to map display pipe mmio");
		return PTR_ERR(adp->fe);
	}

	adp->mipi = devm_platform_ioremap_resource_byname(pdev, "mipi");
	if (IS_ERR(adp->mipi)) {
		dev_err(&pdev->dev, "failed to map mipi mmio");
		return PTR_ERR(adp->mipi);
	}

	adp->be_irq = platform_get_irq_byname(pdev, "be");
	if (adp->be_irq < 0) {
		dev_err(&pdev->dev, "failed to find be irq");
		return adp->be_irq;
	}

	adp->fe_irq = platform_get_irq_byname(pdev, "fe");
	if (adp->fe_irq < 0) {
		dev_err(&pdev->dev, "failed to find fe irq");
		return adp->fe_irq;
	}
	return 0;
}


static int adp_dsi_gen_pkt_hdr_write(struct adp_drv_private *adp, u32 hdr_val)
{
	int ret;
	u32 val, mask;

	ret = readl_poll_timeout(adp->mipi + DSI_CMD_PKT_STATUS,
				 val, !(val & GEN_CMD_FULL), 1000,
				 CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(adp->drm.dev, "failed to get available command FIFO\n");
		return ret;
	}

	writel(hdr_val, adp->mipi + DSI_GEN_HDR);

	mask = GEN_CMD_EMPTY | GEN_PLD_W_EMPTY;
	ret = readl_poll_timeout(adp->mipi + DSI_CMD_PKT_STATUS,
				 val, (val & mask) == mask,
				 1000, CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(adp->drm.dev, "failed to write command FIFO\n");
		return ret;
	}

	return 0;
}

static int adp_dsi_write(struct adp_drv_private *adp,
			 const struct mipi_dsi_packet *packet)
{
	const u8 *tx_buf = packet->payload;
	int len = packet->payload_length, pld_data_bytes = sizeof(u32), ret;
	__le32 word;
	u32 val;

	while (len) {
		if (len < pld_data_bytes) {
			word = 0;
			memcpy(&word, tx_buf, len);
			writel(le32_to_cpu(word), adp->mipi + DSI_GEN_PLD_DATA);
			len = 0;
		} else {
			memcpy(&word, tx_buf, pld_data_bytes);
			writel(le32_to_cpu(word), adp->mipi + DSI_GEN_PLD_DATA);
			tx_buf += pld_data_bytes;
			len -= pld_data_bytes;
		}

		ret = readl_poll_timeout(adp->mipi + DSI_CMD_PKT_STATUS,
					 val, !(val & GEN_PLD_W_FULL), 1000,
					 CMD_PKT_STATUS_TIMEOUT_US);
		if (ret) {
			dev_err(adp->drm.dev,
				"failed to get available write payload FIFO\n");
			return ret;
		}
	}

	word = 0;
	memcpy(&word, packet->header, sizeof(packet->header));
	return adp_dsi_gen_pkt_hdr_write(adp, le32_to_cpu(word));
}

static int adp_dsi_read(struct adp_drv_private *adp,
			const struct mipi_dsi_msg *msg)
{
	int i, j, ret, len = msg->rx_len;
	u8 *buf = msg->rx_buf;
	u32 val;

	/* Wait end of the read operation */
	ret = readl_poll_timeout(adp->mipi + DSI_CMD_PKT_STATUS,
				 val, !(val & GEN_RD_CMD_BUSY),
				 1000, CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(adp->drm.dev, "Timeout during read operation\n");
		return ret;
	}

	for (i = 0; i < len; i += 4) {
		/* Read fifo must not be empty before all bytes are read */
		ret = readl_poll_timeout(adp->mipi + DSI_CMD_PKT_STATUS,
					 val, !(val & GEN_PLD_R_EMPTY),
					 1000, CMD_PKT_STATUS_TIMEOUT_US);
		if (ret) {
			dev_err(adp->drm.dev, "Read payload FIFO is empty\n");
			return ret;
		}

		val = readl(adp->mipi + DSI_GEN_PLD_DATA);
		for (j = 0; j < 4 && j + i < len; j++)
			buf[i + j] = val >> (8 * j);
	}

	return ret;
}

static ssize_t adp_dsi_host_transfer(struct mipi_dsi_host *host,
				     const struct mipi_dsi_msg *msg)
{
	struct adp_drv_private *adp = mipi_to_adp(host);
	struct mipi_dsi_packet packet;
	int ret, nb_bytes;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret) {
		dev_err(adp->drm.dev, "failed to create packet: %d\n", ret);
		return ret;
	}

	ret = adp_dsi_write(adp, &packet);
	if (ret)
		return ret;

	if (msg->rx_buf && msg->rx_len) {
		ret = adp_dsi_read(adp, msg);
		if (ret)
			return ret;
		nb_bytes = msg->rx_len;
	} else {
		nb_bytes = packet.size;
	}

	return nb_bytes;
}

static int adp_dsi_host_attach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *dev)
{
	return 0;
}

static int adp_dsi_host_detach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *dev)
{
	return 0;
}

static const struct mipi_dsi_host_ops adp_dsi_host_ops = {
	.transfer = adp_dsi_host_transfer,
	.attach = adp_dsi_host_attach,
	.detach = adp_dsi_host_detach,
};

static irqreturn_t adp_fe_irq(int irq, void *arg)
{
	struct adp_drv_private *adp = (struct adp_drv_private *)arg;
	u32 int_status;
	u32 int_ctl;

	spin_lock(&adp->irq_lock);

	int_status = readl(adp->fe + ADP_INT_STATUS);
	if (int_status & ADP_INT_STATUS_VBLANK) {
		drm_crtc_handle_vblank(&adp->crtc);
		spin_lock(&adp->crtc.dev->event_lock);
		if (adp->event) {
			int_ctl = readl(adp->fe + ADP_CTRL);
			if ((int_ctl & 0xF00) == 0x600) {
				drm_crtc_send_vblank_event(&adp->crtc, adp->event);
				adp->event = NULL;
				drm_crtc_vblank_put(&adp->crtc);
			}
		}
		spin_unlock(&adp->crtc.dev->event_lock);
	}

	writel(int_status, adp->fe + ADP_INT_STATUS);

	spin_unlock(&adp->irq_lock);

	return IRQ_HANDLED;
}

static int adp_probe(struct platform_device *pdev)
{
	struct adp_drv_private *adp;
	int err;

	adp = devm_drm_dev_alloc(&pdev->dev, &adp_driver, struct adp_drv_private, drm);
	if (IS_ERR(adp))
		return PTR_ERR(adp);

	spin_lock_init(&adp->irq_lock);

	dev_set_drvdata(&pdev->dev, &adp->drm);

	err = adp_parse_of(pdev, adp);
	if (err < 0)
		return err;

	adp->dsi.dev = &pdev->dev;
	adp->dsi.ops = &adp_dsi_host_ops;
	err = mipi_dsi_host_register(&adp->dsi);
	if (err < 0)
		return err;

	adp_disable_vblank(adp);
	writel(ADP_CTRL_FIFO_ON | ADP_CTRL_VBLANK_ON, adp->fe + ADP_CTRL);

	err = adp_setup_mode_config(adp);
	if (err < 0)
		return err;

	err = devm_request_irq(&pdev->dev, adp->fe_irq, adp_fe_irq, 0,
			       "adp-fe", adp);
	if (err)
		return err;

	err = drm_dev_register(&adp->drm, 0);
	if (err)
		return err;
	return 0;
}

static void adp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_device *drm = dev_get_drvdata(dev);
	struct adp_drv_private *adp = to_adp(drm);

	adp_disable_vblank(adp);
	mipi_dsi_host_unregister(&adp->dsi);
	drm_dev_unregister(drm);
	dev_set_drvdata(dev, NULL);
	drm_atomic_helper_shutdown(drm);
}

static const struct of_device_id adp_of_match[] = {
	{ .compatible = "apple,h7-display-pipe", },
	{ },
};
MODULE_DEVICE_TABLE(of, adp_of_match);

static struct platform_driver adp_platform_driver = {
	.driver = {
		.name = "adp",
		.of_match_table = adp_of_match,
	},
	.probe = adp_probe,
	.remove = adp_remove,
};

module_platform_driver(adp_platform_driver);

MODULE_DESCRIPTION("Apple Display Pipe DRM driver");
MODULE_LICENSE("GPL v2");
