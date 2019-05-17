// ins_init_align.cpp : 定义控制台应用程序的入口点。
//

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <malloc.h>
#include "imu.h"

static FILE * fp = NULL;

static FILE * file_open(char *file) {
	fp = fopen(file, "r");
	if (fp == NULL) {
		printf("%s open failed\r\n", file);
		return NULL;
	}
	return NULL;
}

float extern gx_zero;
float extern gy_zero;
float extern gz_zero;

static int file_read_imu(void) {
	int ret = -1;
	int tic = 0;
	float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
	char pos = 0;
	float course = 0, alt = 0, gps_v = 0;
	unsigned int lat = 0, lon, lat_sn, lon_ew, obd_v = 0;
	char buf[512] = { 0 };
	ins_gps_t ins_gps;

	if (fp == NULL) {
		printf("data file is not open\r\n");
		return -1;
	}

	if (fgets(buf, sizeof(buf), fp) == NULL) {
		return -1;
	}

	static int tic_ = 0;
	float period = 0;
	ret = sscanf(buf, "%d,%f,%f,%f,%f,%f,%f,%c,%f,%f,%f,%d,%d,%d,%d,%d,%f,%f,%f,%f", &tic, &gx, &gy, &gz, &ax, &ay, &az, //
			&pos, &course, &alt, &gps_v, &lat, &lon, &lat_sn, &lon_ew, &obd_v, &period, &gx_zero, &gy_zero, &gz_zero);

	if (ret == 20) {
		if (tic != tic_ + 1) {
		//	printf("----------------%d\r\n", tic);
		}
		tic_ = tic;
	}

//	ret = sscanf(buf, "%d,%f,%f,%f,%f,%f,%f,%c,%f,%f,%f,%d,%d,%d,%d,%d", &tic, &gx, &gy, &gz, &ax, &ay, &az, //
//		&pos, &course, &alt, &gps_v, &lat, &lon, &lat_sn, &lon_ew, &obd_v);
//
//	if (ret == 20) {
//		if (tic != tic_ + 1) {
//			printf("----------------%d\r\n", tic);
//		}
//		tic_ = tic;
//	}

	ins_gps.alt = alt;
	ins_gps.course = course;
	ins_gps.lat = lat;
	ins_gps.lat_sn = lat_sn;
	ins_gps.lon = lon;
	ins_gps.lon_ew = lon_ew;
	ins_gps.pos = pos;
	ins_gps.v = obd_v;

	if (ret == 20) {
		imu_set_period(period);
//		imu_set_period(period / 1000);
//		printf("%d--%f,%f,%f->%f,%f,%f->%f,%f,%f\r\n", tic_, gx_zero, gy_zero, gz_zero, //
//				gyro_zero->gx_zero, gyro_zero->gy_zero, gyro_zero->gz_zero, //
//				gx_zero - gyro_zero->gx_zero, gy_zero - gyro_zero->gy_zero, gz_zero - gyro_zero->gz_zero);

//		if (ins_gps.v != 0) {
		imu_update(gx, gy, gz, ax, ay, az, &ins_gps);
//		imu_update(gx - gx_zero, gy - gy_zero, gz - gz_zero, ax, ay, az, &ins_gps); //0.031495,  0.183599, -0.021241
//			imu_update(gx - 0.053224, gy - 0.185552, gz - (-0.027100), ax, ay, az, &ins_gps);//0.053224,  0.185552, -0.027100
//		} else {
//			imu_update(0, 0, 0, ax, ay, az, &ins_gps);
//		}
	}

	return ret == 20 ? 20 : 0;
}
//
//static void quat_zero(void) {
//	quat_t quat, quat_;
//	vector_t vector;
//	euler_t euler;
//
//	vector = quat_vector_init(0, 0, 1);
//	quat_set(&quat, 0, 0.6f, 1, 1);
//
//	quat = quat_rotate_axis_3d(&quat, &vector, AHRS_DEG2RAD(60) / 2);
//	euler = quat_to_euler(&quat);
//	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", AHRS_RAD2DEG(euler.roll), AHRS_RAD2DEG(euler.pitch), AHRS_RAD2DEG(euler.yaw));
//
//	printf("-----------------------------zero begin-------------------------\r\n");
//	float rotate = quat_to_zero(&quat);
//	printf("-----------------------------zero end-------------------------\r\n");
//
//	printf("-----------------------------init rotate %f begin-------------------------\r\n", AHRS_RAD2DEG(rotate));
//
//	quat_ = quat_rotate_axis_3d(&quat, &vector, rotate / 2);
//	euler = quat_to_euler(&quat_);
//	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", AHRS_RAD2DEG(euler.roll), AHRS_RAD2DEG(euler.pitch), AHRS_RAD2DEG(euler.yaw));
//
//	printf("-----------------------------init rotate %f end-------------------------\r\n", AHRS_RAD2DEG(rotate));
//}


static void quat_rotate_euler(void) {
	quat_t quat, quat_;
	vector_t vector_x, vector_y, vector_z;
	euler_t euler, euler_;
	float x = 0, y = 0, z = 0;

	vector_x = quat_vector_init(1, 0, 0);
	vector_y = quat_vector_init(0, 1, 0);
	vector_z = quat_vector_init(0, 0, 1);

	quat_set(&quat, 1, 1, 1, 1);

	euler = quat_to_euler(&quat);
	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));

	quat_ = quat_rotate_axis_3d(&quat, &vector_z, DEG2RAD(90) / 2);
	euler = quat_to_euler(&quat_);
	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));

	quat_ = quat_rotate_axis_vector(&quat, &vector_z, DEG2RAD(90));
	euler = quat_to_euler(&quat_);
	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));

	printf("z------------------------------------------------\r\n");
	z = quat_yaw_to_zero(&quat_);
	printf("x------------------------------------------------\r\n");
	quat = quat_rotate_axis_vector(&quat_, &vector_z, z / 2);
	y = quat_pitch_to_zero(&quat);
	printf("y------------------------------------------------\r\n");
	quat = quat_rotate_axis_vector(&quat_, &vector_z, z / 2);
	quat = quat_rotate_axis_vector(&quat, &vector_y, y / 2);
	x = quat_roll_to_zero(&quat);
	printf("zero------------------------------------------------\r\n");

	quat = quat_rotate_axis_vector(&quat_, &vector_z, z / 2);
	quat = quat_rotate_axis_vector(&quat, &vector_y, y / 2);
	quat = quat_rotate_axis_vector(&quat, &vector_x, x / 2);

	euler = quat_to_euler(&quat);
	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));


	printf("quat_to_zero------------------------------------------------\r\n");
	quat_to_zero(&quat_);

//	quat_init(&quat);
//
//	quat = quat_rotate(&quat, AHRS_DEG2RAD(45), AHRS_DEG2RAD(45), 0);
//
//	euler = quat_to_euler(&quat);
//	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", AHRS_RAD2DEG(euler.roll), AHRS_RAD2DEG(euler.pitch), AHRS_RAD2DEG(euler.yaw));
//
//	quat =  quat_from_euler(&euler);
//	euler = quat_to_euler(&quat);
//	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", AHRS_RAD2DEG(euler.roll), AHRS_RAD2DEG(euler.pitch), AHRS_RAD2DEG(euler.yaw));

//	euler.roll = AHRS_DEG2RAD(45);
//	euler.pitch = AHRS_DEG2RAD(45);
//	euler.yaw = AHRS_DEG2RAD(0);
//	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", AHRS_RAD2DEG(euler.roll), AHRS_RAD2DEG(euler.pitch), AHRS_RAD2DEG(euler.yaw));
//
//	quat =  quat_from_euler(&euler);
//	euler = quat_to_euler(&quat);
//	printf("欧拉角:%7.2f,%7.2f,%7.2f\r\n", AHRS_RAD2DEG(euler.roll), AHRS_RAD2DEG(euler.pitch), AHRS_RAD2DEG(euler.yaw));
}

int main(int argc, char **argv) {
//if (argc < 2) {
//	return -1;
//}
//
//	quat_rotate_euler();
//	return 0;
	imu_init(50.505050);
//file_open(argv[1]);
	file_open("E:\\workspace\\imu\\raw.DAT");
	while (file_read_imu() >= 0) {
	}

	return 0;
}
