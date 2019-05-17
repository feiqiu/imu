#include "ahrs.h"
#include "ins.h"
#include "location.h"
#include <stdio.h>
#include "../log/log.h"

#define GYRO_ZERO_OFFSET_DEEP			3000

#define GYRO_ZERO_OFFSET_SEC			50

typedef struct {
	float gx_zero;
	float gy_zero;
	float gz_zero;

	float gx[GYRO_ZERO_OFFSET_DEEP];
	float gy[GYRO_ZERO_OFFSET_DEEP];
	float gz[GYRO_ZERO_OFFSET_DEEP];

	short offset;
	short cnt;
	int flag;
} gyro_zero_offset_t;

static gyro_zero_offset_t g_gyro_zero;

static void imu_gyro_zero(float gx, float gy, float gz, short v) {
	short i = 0, offset = 0, avr_cnt = 0;
	float avr_sum_gx = 0, avr_sum_gy = 0, avr_sum_gz = 0;
	float diff = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0, avr_gx = 0, avr_gy = 0, avr_gz = 0, var_gx = 0, var_gy = 0, var_gz = 0;
	gyro_zero_offset_t * gyro_zero = &g_gyro_zero;

	if (v != 0) {
		gyro_zero->cnt = 0;
		gyro_zero->offset = 0;
		return;
	}

	gyro_zero->gx[gyro_zero->offset] = gx;
	gyro_zero->gy[gyro_zero->offset] = gy;
	gyro_zero->gz[gyro_zero->offset] = gz;
	gyro_zero->offset = (gyro_zero->offset + 1) % GYRO_ZERO_OFFSET_DEEP;
	if (gyro_zero->cnt < GYRO_ZERO_OFFSET_DEEP) {
		gyro_zero->cnt++;
		return;
	}

	static unsigned cnt = 0;
	if (cnt++ % GYRO_ZERO_OFFSET_SEC != 0) {
		return;
	}

	offset = gyro_zero->offset;
	avr_cnt = GYRO_ZERO_OFFSET_DEEP - GYRO_ZERO_OFFSET_SEC - GYRO_ZERO_OFFSET_SEC;
	for (i = 0; i < GYRO_ZERO_OFFSET_DEEP; i++) {
		sum_gx += gyro_zero->gx[offset];
		sum_gy += gyro_zero->gy[offset];
		sum_gz += gyro_zero->gz[offset];

		if ((i >= GYRO_ZERO_OFFSET_SEC) && (i < GYRO_ZERO_OFFSET_DEEP - GYRO_ZERO_OFFSET_SEC)) {
			avr_sum_gx += gyro_zero->gx[offset];
			avr_sum_gy += gyro_zero->gy[offset];
			avr_sum_gz += gyro_zero->gz[offset];
		}
		offset = (offset + 1) % GYRO_ZERO_OFFSET_DEEP;
	}

	avr_gx = sum_gx / GYRO_ZERO_OFFSET_DEEP;
	avr_gy = sum_gy / GYRO_ZERO_OFFSET_DEEP;
	avr_gz = sum_gz / GYRO_ZERO_OFFSET_DEEP;

	for (i = 0; i < GYRO_ZERO_OFFSET_DEEP; i++) {
		diff = gyro_zero->gx[i] - avr_gx;
		var_gx += (diff * diff);

		diff = gyro_zero->gy[i] - avr_gy;
		var_gy += (diff * diff);

		diff = gyro_zero->gz[i] - avr_gz;
		var_gz += (diff * diff);
	}

	var_gx = var_gx / GYRO_ZERO_OFFSET_DEEP;
	var_gy = var_gy / GYRO_ZERO_OFFSET_DEEP;
	var_gz = var_gz / GYRO_ZERO_OFFSET_DEEP;

	LogImp("零漂检测:%f,%f,%f,方差:%f\r\n", gyro_zero->gx_zero, gyro_zero->gy_zero, gyro_zero->gz_zero, var_gx + var_gy + var_gz);

	if (var_gx + var_gy + var_gz < 0.01) {
		gyro_zero->flag = 1;

		gyro_zero->gx_zero = avr_sum_gx / avr_cnt;
		gyro_zero->gy_zero = avr_sum_gy / avr_cnt;
		gyro_zero->gz_zero = avr_sum_gz / avr_cnt;
	}
}

typedef struct {
	float freq;
	float period;
	float meter;
	short number;
	ins_gps_t loc_ins;
} imu_mgr_t;

static imu_mgr_t g_imu_mgr;

//pc
float gx_zero = 0, gy_zero = 0, gz_zero = 0;

extern void imu_update(float gx, float gy, float gz, float ax, float ay, float az, ins_gps_t *ins_gps) {
	euler_t euler;
	ins_gps_t ins_init_pos;
	imu_mgr_t *imu = &g_imu_mgr;
	gyro_zero_offset_t * gyro_zero = &g_gyro_zero;
	float yaw, roll, pitch, init_yaw = 0, init_course = 0;
//	static uint64_t tic = 0;
//
//	tic++;
//	LogUart("%8llu,%12.6f,%12.6f,%12.6f,%12.6f,%12.6f,%12.6f," //
//			"%c,%12.6f,%4d,%3d,"//
//			"%10d,%10d,%3d,%3d,"//
//			"%4d,%10.6f,%10.6f,%10.6f,%10.6f",//
//			tic++, gx, gy, gz, ax, ay, az, //
//			ins_gps->pos, ins_gps->course, ins_gps->alt, ins_gps->v, //
//			ins_gps->lat, ins_gps->lon, ins_gps->lat_sn, ins_gps->lon_ew, //
//			ins_gps->v, imu->period, gyro_zero->gx_zero, gyro_zero->gy_zero, gyro_zero->gz_zero);

	imu_gyro_zero(gx, gy, gz, ins_gps->v);
//	return;
	if (gyro_zero->flag) {
		gx_zero = gyro_zero->gx_zero;
		gy_zero = gyro_zero->gy_zero;
		gz_zero = gyro_zero->gz_zero;
	}

	if (ins_gps->v != 0) {
		ahrs_update(DEG2RAD(gx - gx_zero), DEG2RAD(gy - gy_zero), DEG2RAD(gz - gz_zero), ax, ay, az);
	} else {
		ahrs_update(0, 0, 0, ax, ay, az);
	}

	ahrs_euler(&euler);
	yaw = RAD2DEG(euler.yaw);
	roll = RAD2DEG(euler.roll);
	pitch = RAD2DEG(euler.pitch);

	ins_update(roll, pitch, yaw, ins_gps);

	ins_euler_parser(roll, pitch, yaw, &euler);
	loc_pos_update(&euler, ins_gps->v, &imu->loc_ins);

	imu->meter += ins_gps->v / 3.6f * imu->period;
	if (imu->number++ >= (int) imu->freq) {
		imu->number = 0;
		ins_align_pos(&ins_init_pos, &init_yaw, &init_course);
		if (imu->loc_ins.pos == 'A') {
			LogImp("里程:%8.2f,周期:%10.6f,零漂:%10.6f,%10.6f,%10.6f", //
					imu->meter, imu->period, gyro_zero->gx_zero, gyro_zero->gy_zero, gyro_zero->gz_zero);
			LogImp("v:%3d," //
							"init_yaw:%7.2f,%7.2f,"//
							"loc_yaw:%7.2f,"//
							"raw_yaw:%7.2f,%7.2f,%7.2f,"//
							"ins_yaw:%7.2f,%7.2f,%7.2f,"//
							"GPS:%c,%c,%10d,%c,%10d,%5.2f,INS:%c,%c,%10d,%c,%10d,%5.2f,%d",//
			ins_gps->v, //
					init_yaw, init_course, //
					euler.yaw, //
					roll, pitch, yaw, //
					euler.roll, euler.pitch, euler.yaw, //
					ins_gps->pos, ins_gps->lat_sn, ins_gps->lat, ins_gps->lon_ew, ins_gps->lon, ins_gps->course, //
					imu->loc_ins.pos, imu->loc_ins.lat_sn, imu->loc_ins.lat, imu->loc_ins.lon_ew, imu->loc_ins.lon, imu->loc_ins.course, ins_gps->alt);

		}
	}
}

extern void imu_gravity_axis(short m, float *ax, float *ay, float *az) {
	ahrs_gravity_axis(m, ax, ay, az);
}

extern void imu_set_period(float sec) {
	imu_mgr_t *imu = &g_imu_mgr;

	imu->period = sec;
	loc_set_period(sec);
	ahrs_set_period(sec);
}

extern void imu_gps(ins_gps_t *ins_gps) {
	imu_mgr_t *imu = &g_imu_mgr;

	*ins_gps = imu->loc_ins;
}

extern void imu_init(float freq) {
	imu_mgr_t *imu = &g_imu_mgr;

	imu->freq = freq;
	imu->period = 1.0f / freq;

	loc_init(imu->freq);
	ahrs_init(imu->freq);
}
