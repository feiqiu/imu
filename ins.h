#ifndef __LXT_INS_H__
#define __LXT_INS_H__

#include "location.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int ins_align_status(void);

extern void ins_update(float roll, float pitch, float yaw, ins_gps_t *ins_gps);

extern void ins_align_pos(ins_gps_t *ins_gps, float *init_yaw, float *init_course);

extern void ins_euler_parser(float roll, float pitch, float yaw, euler_t *ahrs);
#ifdef __cplusplus
}
#endif

#endif
