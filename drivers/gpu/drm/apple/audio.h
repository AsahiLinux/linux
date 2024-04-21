#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <linux/types.h>

struct device;
struct platform_device;
struct dcp_sound_cookie;

int dcp_audiosrv_prepare(struct device *dev, struct dcp_sound_cookie *cookie);
int dcp_audiosrv_startlink(struct device *dev, struct dcp_sound_cookie *cookie);
int dcp_audiosrv_stoplink(struct device *dev);
int dcp_audiosrv_unprepare(struct device *dev);
int dcp_audiosrv_get_elements(struct device *dev, void *elements, size_t maxsize);
int dcp_audiosrv_get_product_attrs(struct device *dev, void *attrs, size_t maxsize);

void dcpaud_connect(struct platform_device *pdev, bool connected);
void dcpaud_disconnect(struct platform_device *pdev);

#endif /* __AUDIO_H__ */
