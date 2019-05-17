#ifndef __LXT_AHRS_H__
#define __LXT_AHRS_H__

#include "quat_euler.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void ahrs_init(float freq);

extern void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az);

extern void ahrs_euler(euler_t *euler);

extern void ahrs_gravity_axis(short m, float *ax, float *ay, float *az);

extern void ahrs_set_period(float sec);

extern void ahrs_euler_align_rotate(void);
#ifdef __cplusplus
}
#endif

#endif
