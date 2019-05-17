#include <math.h>
#include "ahrs.h"
#include "ins.h"
#include "../log/log.h"
#include "quat_euler.h"

#define AHRS_GYRO_CORR			(1.25f / 360) //360度多出1.25度

#define AHRS_DEC_DEEP			125

#define AHRS_GYRO_TH			0.5

#define AHRS_TWO_KP_FAST		2.0f

#define AHRS_TWO_KP_NEVER		0.0f

#define AHRS_GYRO_SEC			50

typedef struct {
	float freq;
	float period;
	axis_rotate_t axis_rotate;
	float correction_factor;
	volatile float two_kp;
	volatile float two_ki;
	quat_t quat;
	volatile float integral_fbx;
	volatile float integral_fby;
	volatile float integral_fbz;
	float sec_gx;
	float sec_gy;
	float sec_gz;
} ahrs_mgr;

static ahrs_mgr g_ahrs_euler;
static ahrs_mgr g_ahrs_gravity;

extern void ahrs_set_period(float sec) {
	g_ahrs_euler.period = sec;
	g_ahrs_gravity.period = sec;
}

static void ahrs_init_euler(float freq) {
	ahrs_mgr * ahrs = &g_ahrs_euler;

	quat_init(&ahrs->quat);
	ahrs->two_kp = AHRS_TWO_KP_FAST;
	ahrs->two_ki = 0;
	ahrs->axis_rotate.x = 0;
	ahrs->axis_rotate.y = 0;
	ahrs->axis_rotate.z = 0;
	ahrs->integral_fbx = 0;
	ahrs->integral_fby = 0;
	ahrs->integral_fbz = 0;
	ahrs->freq = freq;
	ahrs->period = 1.0f / freq;
	ahrs->correction_factor = 1.0f - AHRS_GYRO_CORR;
}

static void ahrs_init_gravity(float freq) {
	ahrs_mgr * ahrs = &g_ahrs_gravity;

	quat_init(&ahrs->quat);
	ahrs->two_kp = AHRS_TWO_KP_FAST;
	ahrs->two_ki = 0;
	ahrs->axis_rotate.x = 0;
	ahrs->axis_rotate.y = 0;
	ahrs->axis_rotate.z = 0;
	ahrs->integral_fbx = 0;
	ahrs->integral_fby = 0;
	ahrs->integral_fbz = 0;
	ahrs->freq = freq;
	ahrs->period = 1.0f / freq;
	ahrs->correction_factor = 1.0f - AHRS_GYRO_CORR;
}

extern void ahrs_init(float freq) {
	ahrs_init_euler(freq);
	ahrs_init_gravity(freq);
}

extern void ahrs_gravity_axis(short m, float *ax, float *ay, float *az) {
	ahrs_mgr * ahrs = &g_ahrs_gravity;
	float halfvx = 0, halfvy = 0, halfvz = 0;

	halfvx = ahrs->quat.x * ahrs->quat.z - ahrs->quat.w * ahrs->quat.y;
	halfvy = ahrs->quat.w * ahrs->quat.x + ahrs->quat.y * ahrs->quat.z;
	halfvz = ahrs->quat.w * ahrs->quat.w - 0.5f + ahrs->quat.z * ahrs->quat.z;

	*ax = halfvx * 2 * m;
	*ay = halfvy * 2 * m;
	*az = halfvz * 2 * m;
}

static void ahrs_euler_dynamic_two_kp(void) {
	ahrs_mgr * ahrs = &g_ahrs_euler;

	if (!ins_align_status()) {
		ahrs->two_kp = AHRS_TWO_KP_FAST;
		return;
	}

	if ((ahrs->sec_gx < AHRS_GYRO_TH) && (ahrs->sec_gy < AHRS_GYRO_TH) && (ahrs->sec_gz < AHRS_GYRO_TH)) {
		ahrs->two_kp = AHRS_TWO_KP_FAST;
	} else {
		ahrs->two_kp = AHRS_TWO_KP_NEVER;
	}
}

static void ahrs_update_quat(ahrs_mgr * ahrs, float gx, float gy, float gz, float ax, float ay, float az) {
	float norm = 0;
	float halfvx = 0, halfvy = 0, halfvz = 0;
	float halfex = 0, halfey = 0, halfez = 0;

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		norm = sqrt(ax * ax + ay * ay + az * az);
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;

		halfvx = ahrs->quat.x * ahrs->quat.z - ahrs->quat.w * ahrs->quat.y;
		halfvy = ahrs->quat.w * ahrs->quat.x + ahrs->quat.y * ahrs->quat.z;
		halfvz = ahrs->quat.w * ahrs->quat.w - 0.5f + ahrs->quat.z * ahrs->quat.z;

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		if (ahrs->two_ki > 0.0f) {
			ahrs->integral_fbx += ahrs->two_ki * halfex * ahrs->period;
			ahrs->integral_fby += ahrs->two_ki * halfey * ahrs->period;
			ahrs->integral_fbz += ahrs->two_ki * halfez * ahrs->period;
			gx += ahrs->integral_fbx;
			gy += ahrs->integral_fby;
			gz += ahrs->integral_fbz;
		} else {
			ahrs->integral_fbx = 0.0f;
			ahrs->integral_fby = 0.0f;
			ahrs->integral_fbz = 0.0f;
		}

		gx += ahrs->two_kp * halfex;
		gy += ahrs->two_kp * halfey;
		gz += ahrs->two_kp * halfez;
	}

	gx *= (ahrs->period * ahrs->correction_factor);
	gy *= (ahrs->period * ahrs->correction_factor);
	gz *= (ahrs->period * ahrs->correction_factor);

	ahrs->quat = quat_rotate(&ahrs->quat, gx, gy, gz);
}

static void ahrs_gyro(float gx, float gy, float gz) {
	short i = 0;
	ahrs_mgr * ahrs = &g_ahrs_euler;
	static short cnt = 0, offset = 0;
	static float buff_gx[AHRS_GYRO_SEC] = { 0 };
	static float buff_gy[AHRS_GYRO_SEC] = { 0 };
	static float buff_gz[AHRS_GYRO_SEC] = { 0 };
	float sum_buff_gx = 0, sum_buff_gy = 0, sum_buff_gz = 0;

	buff_gx[offset] = gz;
	buff_gy[offset] = gx;
	buff_gz[offset] = gy;
	offset = (offset + 1) % AHRS_GYRO_SEC;
	if (cnt < AHRS_GYRO_SEC) {
		cnt++;
	}

	for (i = 0; i < AHRS_GYRO_SEC; i++) {
		sum_buff_gx += buff_gx[offset];
		sum_buff_gy += buff_gy[offset];
		sum_buff_gz += buff_gz[offset];
	}

	ahrs->sec_gx = fabs(RAD2DEG(sum_buff_gx * ahrs->period));
	ahrs->sec_gy = fabs(RAD2DEG(sum_buff_gy * ahrs->period));
	ahrs->sec_gz = fabs(RAD2DEG(sum_buff_gz * ahrs->period));
}

extern void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az) {
	ahrs_gyro(gx, gy, gz);
	ahrs_euler_dynamic_two_kp();

	ahrs_update_quat(&g_ahrs_euler, gx, gy, gz, ax, ay, az);
	ahrs_update_quat(&g_ahrs_gravity, gx, gy, gz, ax, ay, az);
}

extern void ahrs_euler(euler_t *euler) {
	ahrs_mgr * ahrs = &g_ahrs_euler;
//	vector_t vector;
	quat_t new_quat = ahrs->quat;

//	vector = quat_vector_init(0, 0, 1);
	if ((fabs(ahrs->axis_rotate.x) < 0.1) && (fabs(ahrs->axis_rotate.y) < 0.1) && (fabs(ahrs->axis_rotate.z) < 0.1)) {
		*euler = quat_to_euler(&ahrs->quat);
	} else {
		*euler = quat_to_euler(&ahrs->quat);
		//LogNop("旋转前四元数:%12.6f,%12.6f,%12.6f,,%12.6f", ahrs->quat.w, ahrs->quat.x, ahrs->quat.y, ahrs->quat.z);
		//LogNop("旋转前:%7.2f,%7.2f,%7.2f", AHRS_RAD2DEG(euler->roll), AHRS_RAD2DEG(euler->pitch), AHRS_RAD2DEG(euler->yaw));
//		new_quat = quat_rotate_axis(&ahrs->quat, &vector, ahrs->rotate / 2);
		new_quat = quat_rotate_axis_xyz(&ahrs->quat, &ahrs->axis_rotate);
		*euler = quat_to_euler(&new_quat);
		//LogNop("旋转后四元数:%12.6f,%12.6f,%12.6f,%12.6f", new_quat.w, new_quat.x, new_quat.y, new_quat.z);
		//LogNop("旋转后:%7.2f,%7.2f,%7.2f,旋转:%10.6f", AHRS_RAD2DEG(euler->roll), AHRS_RAD2DEG(euler->pitch), AHRS_RAD2DEG(euler->yaw), AHRS_RAD2DEG(ahrs->rotate));
	}
}

extern void ahrs_euler_align_rotate(void) {
	ahrs_mgr * ahrs = &g_ahrs_euler;

	ahrs->axis_rotate = quat_to_zero(&ahrs->quat);
}

