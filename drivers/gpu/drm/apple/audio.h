#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <linux/types.h>

struct device;
struct device_node;
struct dcp_sound_cookie;

typedef void (*dcp_audio_hotplug_callback)(struct device *dev, bool connected);

struct dcp_audio_pdata {
	struct device *dcp_dev;
	struct device_node *dpaudio_node;
};

void dcp_audiosrv_set_hotplug_cb(struct device *dev, struct device *audio_dev,
								 dcp_audio_hotplug_callback cb);
int dcp_audiosrv_prepare(struct device *dev, struct dcp_sound_cookie *cookie);
int dcp_audiosrv_startlink(struct device *dev, struct dcp_sound_cookie *cookie);
int dcp_audiosrv_stoplink(struct device *dev);
int dcp_audiosrv_unprepare(struct device *dev);
int dcp_audiosrv_get_elements(struct device *dev, void *elements, size_t maxsize);
int dcp_audiosrv_get_product_attrs(struct device *dev, void *attrs, size_t maxsize);

#endif /* __AUDIO_H__ */
