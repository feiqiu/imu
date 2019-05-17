#ifndef __LXT_IMU_H__
#define __LXT_IMU_H__

#include "ins.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void imu_init(float freq);

extern void imu_update(float gx, float gy, float gz, float ax, float ay, float az, ins_gps_t *ins_gps);

extern void imu_gravity_axis(short m, float *ax, float *ay, float *az);

extern void imu_set_period(float sec);

extern void imu_gps(ins_gps_t *ins_gps);
#ifdef __cplusplus
}
#endif

#endif
