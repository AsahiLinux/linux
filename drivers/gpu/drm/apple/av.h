#ifndef __AV_H__
#define __AV_H__

#include "parser.h"

//int avep_audiosrv_startlink(struct apple_dcp *dcp, struct dcp_sound_cookie *cookie);
//int avep_audiosrv_stoplink(struct apple_dcp *dcp);

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
void av_service_connect(struct apple_dcp *dcp);
void av_service_disconnect(struct apple_dcp *dcp);
#else
static inline void av_service_connect(struct apple_dcp *dcp) { }
static inline void av_service_disconnect(struct apple_dcp *dcp) { }
#endif

#endif /* __AV_H__ */
