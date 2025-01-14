#ifndef __AV_H__
#define __AV_H__

#include "parser.h"

//int avep_audiosrv_startlink(struct apple_dcp *dcp, struct dcp_sound_cookie *cookie);
//int avep_audiosrv_stoplink(struct apple_dcp *dcp);

void av_service_connect(struct apple_dcp *dcp);
void av_service_disconnect(struct apple_dcp *dcp);

#endif /* __AV_H__ */
