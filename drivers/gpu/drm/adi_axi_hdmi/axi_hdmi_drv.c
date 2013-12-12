/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_address.h>
#include <linux/of_i2c.h>
#include <linux/of_dma.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "axi_hdmi_drv.h"
#include "axi_hdmi_crtc.h"
#include "axi_hdmi_encoder.h"

#define DRIVER_NAME	"axi_hdmi_drm"
#define DRIVER_DESC	"AXI HDMI DRM"
#define DRIVER_DATE	"20120930"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static void axi_hdmi_output_poll_changed(struct drm_device *dev)
{
	struct axi_hdmi_private *private = dev->dev_private;
	drm_fbdev_cma_hotplug_event(private->fbdev);
}

static struct drm_mode_config_funcs axi_hdmi_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = axi_hdmi_output_poll_changed,
};

static void axi_hdmi_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &axi_hdmi_mode_config_funcs;
}

/* FIXME - these would be in a header */

struct drm_adi_gem_info {
	uint32_t handle;		/* buffer handle (in) */
	uint32_t pad;
	uint64_t paddr;			/* physical address (out) */
	uint32_t size;			/* virtual size for mmap'ing (out) */
	uint32_t __pad;
};

#define DRM_ADI_GEM_INFO		0x00
#define DRM_ADI_NEXT_FRAME		0x01
#define DRM_ADI_NUM_IOCTLS		0x02

#define DRM_IOCTL_ADI_GEM_INFO		DRM_IOWR(DRM_COMMAND_BASE + DRM_ADI_GEM_INFO, struct drm_adi_gem_info)
#define DRM_IOCTL_ADI_NEXT_FRAME	DRM_IOWR(DRM_COMMAND_BASE + DRM_ADI_NEXT_FRAME, uint32_t)

/* FIXME - end header */

static int ioctl_gem_info(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_adi_gem_info *args = data;
	struct drm_gem_object *obj;
	struct drm_gem_cma_object *cma_obj;
	int ret = 0;

	//VERB("%p:%p: handle=%d", dev, file_priv, args->handle);

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj)
		return -ENOENT;
	cma_obj = to_drm_gem_cma_obj(obj);

	args->paddr = cma_obj->paddr;
	args->size = obj->size;

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

struct vdma_addr_regs {
	u32 vsize;          /* 0x0 Vertical size */
	u32 hsize;          /* 0x4 Horizontal size */
	u32 frmdly_stride;  /* 0x8 Frame delay and stride */
	u32 buf_addr[16];   /* 0xC - 0x48 Src addresses */
};

/* Per DMA specific operations should be embedded in the channel structure
 */
struct xilinx_dma_chan {
	void *regs;   /* Control status registers */
	struct vdma_addr_regs *addr_regs; /* Direct address registers */
	spinlock_t lock;                  /* Descriptor operation lock */
	struct list_head active_list;	  /* Active descriptors */
	struct list_head pending_list;	  /* Descriptors waiting */
	struct list_head removed_list;       /* Descriptors queued for removal */
	struct dma_chan common;           /* DMA common channel */
};

static int ioctl_next_frame(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct axi_hdmi_private *p = dev->dev_private;
	struct xilinx_dma_chan *chan = container_of(p->dma, struct xilinx_dma_chan, common);
	uint32_t handle = *(uint32_t*)data;
	struct drm_gem_object *obj;
	struct drm_gem_cma_object *cma_obj;

	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (!obj)
		return -ENOENT;
	cma_obj = to_drm_gem_cma_obj(obj);

	/* HACK - set new addresses latched on next frame */
	chan->addr_regs->buf_addr[0] = cma_obj->paddr;
	chan->addr_regs->buf_addr[1] = cma_obj->paddr;
	chan->addr_regs->buf_addr[2] = cma_obj->paddr;
	chan->addr_regs->vsize = 1080;

	return 0;
}

static struct drm_ioctl_desc ioctls[DRM_COMMAND_END - DRM_COMMAND_BASE] = {
	DRM_IOCTL_DEF_DRV(ADI_GEM_INFO, ioctl_gem_info, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(ADI_NEXT_FRAME, ioctl_next_frame, DRM_UNLOCKED|DRM_AUTH),
};

static int axi_hdmi_load(struct drm_device *dev, unsigned long flags)
{
	struct axi_hdmi_private *private = dev_get_drvdata(dev->dev);
	struct drm_encoder *encoder;
	int ret;

	dev->dev_private = private;

	drm_mode_config_init(dev);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	axi_hdmi_mode_config_init(dev);

	private->crtc = axi_hdmi_crtc_create(dev);
	if (IS_ERR(private->crtc)) {
		ret = PTR_ERR(private->crtc);
		goto err_crtc;
	}

	encoder = axi_hdmi_encoder_create(dev);
	if (IS_ERR(encoder)) {
	    ret = PTR_ERR(encoder);
	    goto err_crtc;
	}

	private->fbdev = drm_fbdev_cma_init(dev, 32, 1, 1);
	if (IS_ERR(private->fbdev)) {
		DRM_ERROR("failed to initialize drm fbdev\n");
		ret = PTR_ERR(private->fbdev);
		goto err_crtc;
	}

	return 0;

err_crtc:
	drm_mode_config_cleanup(dev);
	return ret;
}

static int axi_hdmi_unload(struct drm_device *dev)
{
	struct axi_hdmi_private *private = dev->dev_private;

	drm_fbdev_cma_fini(private->fbdev);
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);

	return 0;
}

static void axi_hdmi_lastclose(struct drm_device *dev)
{
	struct axi_hdmi_private *private = dev->dev_private;
	drm_fbdev_cma_restore_mode(private->fbdev);
}

static const struct file_operations axi_hdmi_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= drm_gem_cma_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
	.release	= drm_release,
};

static struct drm_driver axi_hdmi_driver = {
	.driver_features	= DRIVER_BUS_PLATFORM |
				  DRIVER_MODESET | DRIVER_GEM,
	.load			= axi_hdmi_load,
	.unload			= axi_hdmi_unload,
	.lastclose		= axi_hdmi_lastclose,
	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_cma_dumb_destroy,
	.ioctls			= ioctls,
	.num_ioctls		= DRM_ADI_NUM_IOCTLS,
	.fops			= &axi_hdmi_driver_fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
};

static int axi_hdmi_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct axi_hdmi_private *private;
	struct device_node *slave_node;
	struct resource *res;

	private = devm_kzalloc(&pdev->dev, sizeof(*private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	private->base = devm_request_and_ioremap(&pdev->dev, res);
	if (!private->base)
		return -EBUSY;

	private->hdmi_clock = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(private->hdmi_clock))
		return -EPROBE_DEFER;

	slave_node = of_parse_phandle(np, "encoder-slave", 0);
	if (!slave_node)
		return -EINVAL;

	private->is_rgb = of_property_read_bool(np, "adi,is-rgb");
	private->embedded_sync = of_property_read_bool(np, "adi,embedded-sync");
	
	private->encoder_slave = of_find_i2c_device_by_node(slave_node);
	of_node_put(slave_node);

	if (!private->encoder_slave || !private->encoder_slave->dev.driver)
		return -EPROBE_DEFER;

	private->dma = dma_request_slave_channel(&pdev->dev, "video");
	if (private->dma == NULL)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, private);

	return drm_platform_init(&axi_hdmi_driver, pdev);
}

static int axi_hdmi_platform_remove(struct platform_device *pdev)
{
	struct axi_hdmi_private *private = platform_get_drvdata(pdev);
	drm_platform_exit(&axi_hdmi_driver, pdev);
	dma_release_channel(private->dma);
	return 0;
}

static const struct of_device_id adv7511_encoder_of_match[] = {
	{ .compatible = "adi,axi-hdmi-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, adv7511_encoder_of_match);

static struct platform_driver adv7511_encoder_driver = {
	.driver = {
		.name = "axi-hdmi",
		.owner = THIS_MODULE,
		.of_match_table = adv7511_encoder_of_match,
	},
	.probe = axi_hdmi_platform_probe,
	.remove = axi_hdmi_platform_remove,
};
module_platform_driver(adv7511_encoder_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("");
