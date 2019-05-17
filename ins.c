#include "ins.h"
#include "location.h"
#include "ahrs.h"
#include <stdio.h>
#include <math.h>
#include "imu.h"
#include "../log/log.h"

#define INS_ALIGN_DEEP			5
#define INS_ALIGN_V				20
#define INS_ALIGN_LOC_TH_V  	30
#define INS_ALIGN_LOC_TH_DIS    25

#define INS_EARTH_RADIUS 	6378137.0f

typedef struct {
	char pos;
	float course;
	unsigned int alt;
	unsigned int v;
	float yaw;
} align_info_t;

typedef struct {
	unsigned long long cnt;
	align_info_t info[INS_ALIGN_DEEP];
} align_init_t;

typedef struct {
	char horz_flag;
	char init_flag;
	float init_yaw;
	float gps_pitch;
	float init_course;
	float horz_pitch;
	float horz_roll;
	float sign_flag;
	ins_gps_t ins_gps;
	align_init_t align;
	ins_gps_t last_gps;
} ins_mgr_t;

static ins_mgr_t g_ins;

static void ins_max_min(align_init_t * align, float *max_yaw, float *min_yaw, float *max_course, float *min_course) {
	int i = 0;

	*max_yaw = *min_yaw = align->info[0].yaw;
	*max_course = *min_course = align->info[0].course;

	for (i = 0; i < INS_ALIGN_DEEP; i++) {
		if (*max_yaw < align->info[i].yaw) {
			*max_yaw = align->info[i].yaw;
		}

		if (*min_yaw > align->info[i].yaw) {
			*min_yaw = align->info[i].yaw;
		}

		if (*max_course < align->info[i].course) {
			*max_course = align->info[i].course;
		}

		if (*min_course > align->info[i].course) {
			*min_course = align->info[i].course;
		}
	}
}

static void ins_info_pre(align_init_t * align, float max_yaw, float min_yaw, float max_course, float min_course) {
	int i = 0;

	if (max_yaw - min_yaw > 180) {
		for (i = 0; i < INS_ALIGN_DEEP; i++) {
			if (align->info[i].yaw > 180) {
				align->info[i].yaw = align->info[i].yaw - 360;
			}
		}
	}

	if (max_course - min_course > 180) {
		for (i = 0; i < INS_ALIGN_DEEP; i++) {
			if (align->info[i].course > 180) {
				align->info[i].course = align->info[i].course - 360;
			}
		}
	}
}

static double ins_distance(double lat1, double lng1, double lat2, double lng2) {
	double rad_lat1 = DEG2RAD(lat1);
	double rad_lat2 = DEG2RAD(lat2);

	double a = rad_lat1 - rad_lat2;
	double b = DEG2RAD(lng1) - DEG2RAD(lng2);

	double distance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(rad_lat1) * cos(rad_lat2) * pow(sin(b / 2), 2)));
	distance = distance * INS_EARTH_RADIUS;
	distance = round(distance * 10000) / 10000.0;

	return distance;
}

static double ins_format(int location) {
	double lo_h = (location / 10000) / 100;
	double lo_l = (location / 10000) % 100;
	double lo_l_ = (location % 10000) / 10000.0;

	lo_l = (lo_l + lo_l_) / 60;
	lo_h += lo_l;

	return lo_h;
}

static double ins_distance_cmp(ins_gps_t *gps, ins_gps_t *imu) {
	double gps_lat = 0, gps_lon = 0, imu_lat = 0, imu_lon = 0;

	gps_lat = ins_format(gps->lat);
	gps_lon = ins_format(gps->lon);

	imu_lat = ins_format(imu->lat);
	imu_lon = ins_format(imu->lon);
	if (gps->lat_sn == 'S') {
		gps_lat = 0 - gps_lat;
		imu_lat = 0 - imu_lat;
	}

	if (gps->lon_ew == 'W') {
		gps_lon = 0 - gps_lon;
		imu_lon = 0 - imu_lon;
	}

	return ins_distance(gps_lat, gps_lon, imu_lat, imu_lon);
}

static void ins_align(float roll, float pitch, float yaw, ins_gps_t *ins_gps) {
	ins_mgr_t *ins = &g_ins;
	align_init_t * align = &ins->align;
	float v_avr = 0, v_sum = 0;
	float yaw_var = 0, yaw_avr = 0, yaw_sum = 0;
	int offset = 0, i = 0, alt_min = 0, alt_max = 0, distance = 0;
	float course_var = 0, course_avr = 0, course_sum = 0;

	if (ins_gps->pos != 'A' || ins_gps->v < INS_ALIGN_V) {
		offset = align->cnt = 0;
		return;
	}

	if ((ins->last_gps.lat == ins_gps->lat) && (ins->last_gps.lon == ins_gps->lon)) {
		return;
	}

	ins->last_gps = *ins_gps;
	offset = align->cnt % INS_ALIGN_DEEP;
	align->info[offset].yaw = yaw;
	align->info[offset].alt = ins_gps->alt;
	align->info[offset].pos = ins_gps->pos;
	align->info[offset].course = ins_gps->course;
	align->info[offset].v = ins_gps->v;

	align->cnt++;
	if (align->cnt < INS_ALIGN_DEEP) {
		return;
	}

	float max_yaw = 0, min_yaw = 0, max_course = 0, min_course = 0;
	ins_max_min(align, &max_yaw, &min_yaw, &max_course, &min_course);
	ins_info_pre(align, max_yaw, min_yaw, max_course, min_course);

	alt_min = alt_max = align->info[0].alt;
	for (i = 0; i < INS_ALIGN_DEEP; i++) {
		yaw_sum += align->info[i].yaw;
		course_sum += align->info[i].course;
		v_sum += align->info[i].v;
		distance += align->info[i].v; //t=1s, s=vt->s=v
		if (align->info[i].alt > alt_max) {
			alt_max = align->info[i].alt;
		}
		if (align->info[i].alt < alt_min) {
			alt_min = align->info[i].alt;
		}
	}

	distance = distance / 3.6;
	v_avr = v_sum / INS_ALIGN_DEEP;
	yaw_avr = yaw_sum / INS_ALIGN_DEEP;
	course_avr = course_sum / INS_ALIGN_DEEP;

	char horz_flag = 1;
	for (i = 0; i < INS_ALIGN_DEEP; i++) {
		yaw_var += (align->info[i].yaw - yaw_avr) * (align->info[i].yaw - yaw_avr);
		course_var += (align->info[i].course - course_avr) * (align->info[i].course - course_avr);
		if (horz_flag == 1 && align->info[i].alt != ins_gps->alt) {
			horz_flag = 0;
		}
	}

	if (horz_flag) {
		ins->horz_flag = horz_flag;
		ins->horz_roll = roll;
		ins->horz_pitch = pitch;
		LogNop("检测到水平:%f--%f", roll, pitch);
	}

	yaw_var = yaw_var / INS_ALIGN_DEEP;
	course_var = course_var / INS_ALIGN_DEEP;

	//LogNop("ins align var:%10lld,%8.2f,%8.2f\n",//
	//	ins_gps->cnt, course_var, yaw_var);

//	if (ins->init_flag) {
//		return;
//	}

	yaw_avr = yaw_avr < 0 ? yaw_avr + 360 : yaw_avr;
	course_avr = course_avr < 0 ? course_avr + 360 : course_avr;

	min_yaw = min_yaw < 0 ? min_yaw + 360 : min_yaw;
	max_yaw = max_yaw < 0 ? max_yaw + 360 : max_yaw;

	min_course = min_course < 0 ? min_course + 360 : min_course;
	max_course = max_course < 0 ? max_course + 360 : max_course;

	float diff_min_course = fabs(course_avr - min_course);
	float diff_max_course = fabs(course_avr - max_course);

	float diff_min_yaw = fabs(yaw_avr - min_yaw);
	float diff_max_yaw = fabs(yaw_avr - max_yaw);

	diff_min_yaw = (diff_min_yaw < 180) ? diff_min_yaw : 360 - diff_min_yaw;
	diff_max_yaw = (diff_max_yaw < 180) ? diff_max_yaw : 360 - diff_max_yaw;

	diff_min_course = (diff_min_course < 180) ? diff_min_course : 360 - diff_min_course;
	diff_max_course = (diff_max_course < 180) ? diff_max_course : 360 - diff_max_course;

	float diff_yaw_range = diff_min_yaw > diff_max_yaw ? diff_min_yaw : diff_max_yaw;
	float diff_course_range = diff_min_course > diff_max_course ? diff_min_course : diff_max_course;

	float diff_yaw = fabs(yaw_avr - yaw);
	float diff_course = fabs(course_avr - ins_gps->course);

	diff_yaw = diff_yaw < 180 ? diff_yaw : 360 - diff_yaw;
	diff_course = diff_course < 180 ? diff_course : 360 - diff_course;

	LogImp("对准,方差:%f,%f,差值:%f,%f,航向:%f,%f,范围:%f,%f", yaw_var, course_var, diff_yaw, diff_course, yaw, ins_gps->course, diff_yaw_range, diff_course_range);

	ins_gps_t imu;
	imu_gps(&imu);
	float dis_diff = ins_distance_cmp(ins_gps, &imu);
	if (ins->init_flag && dis_diff > INS_ALIGN_LOC_TH_DIS && v_avr > INS_ALIGN_LOC_TH_V) {
		LogImp("对准位置:%f", dis_diff);
		loc_align(ins_gps);
	}

	if (!(diff_yaw < 0.4 && diff_yaw_range < 0.6 && diff_course < 0.8 && diff_course_range < 1.2)) {
		return;
	}

	float yaw_course = ins->init_course + ins->sign_flag * (yaw_avr - 0);
	yaw_course = yaw_course >= 360 ? yaw_course - 360 : yaw_course;
	yaw_course = yaw_course < 0 ? yaw_course + 360 : yaw_course;

	float diff_yaw_course = fabs(yaw_course - course_avr);
	diff_yaw_course = (diff_yaw_course < 180) ? diff_yaw_course : 360 - diff_yaw_course;

	if (yaw_var < 0.1 && course_var < 0.2) {
		LogImp("高精度对准,方差:%f,%f", yaw_var, course_var);
		goto align_handle;
	} else if (yaw_var < 0.2 && course_var < 0.4) {
		LogImp("中精度对准,方差:%f,%f", yaw_var, course_var);
		goto align_handle;
	} else if (yaw_var < 0.4 && course_var < 0.8) {
		LogImp("低精度对准,方差:%f,%f", yaw_var, course_var);
		goto align_handle;
	}

	return;
	align_handle: ;
	if (ins->init_flag && diff_yaw_course < sqrt(course_var)) {
		loc_align(ins_gps);
		LogImp("当前航向对准精度更高%f,%f", diff_yaw_course, sqrt(course_var));
		return;
	}
	ahrs_euler_align_rotate();
	ins->sign_flag = fabs(roll) > 90 ? -1 : 1;
	ins->init_flag = 1;
	ins->gps_pitch = asin((alt_max - alt_min) * 1.0f / distance);
	ins->gps_pitch = RAD2DEG(ins->gps_pitch);
	ins->gps_pitch = ins->gps_pitch > 30 ? 30 : ins->gps_pitch;
	ins->init_yaw = yaw_avr;
	ins->init_course = course_avr;
	ins->ins_gps = *ins_gps;

	loc_align(ins_gps);

	LogImp("对准航向俯仰角:%d-%d-%d", alt_max, alt_min, distance);
	LogImp("对准航向信息:%c\talt:%10d\tlat:%c,%10d\tlon:%c,%10d\tcourse:%8.2f\tv:%10d,pitch:%6.2f,sign:%f\r\n",	//
			ins_gps->pos, ins_gps->alt, ins_gps->lat_sn, ins_gps->lat, ins_gps->lon_ew, ins_gps->lon, ins_gps->course,	//
			ins_gps->v, ins->gps_pitch, ins->sign_flag);
}

extern void ins_align_pos(ins_gps_t *ins_gps, float *init_yaw, float *init_course) {
	ins_mgr_t *ins = &g_ins;

	*ins_gps = ins->ins_gps;
	*init_yaw = ins->init_yaw;
	*init_course = ins->init_course;
}

extern void ins_euler_parser(float roll, float pitch, float yaw, euler_t *euler) {
	float yaw_course = 0;
	ins_mgr_t *ins = &g_ins;

	yaw = yaw < 0 ? yaw + 360 : yaw;
//	yaw_course = ins->init_course - (yaw - ins->init_yaw);
	yaw_course = ins->init_course + ins->sign_flag * (yaw - 0);
	yaw_course = yaw_course >= 360 ? yaw_course - 360 : yaw_course;
	yaw_course = yaw_course < 0 ? yaw_course + 360 : yaw_course;

//	LogUart("%d,%f,%f->%f,%f", ins->horz_flag, roll, pitch, ins->horz_roll, ins->horz_pitch);
	if (!ins->horz_flag) {
		euler->roll = roll;
		euler->pitch = ins->gps_pitch;
	} else {
		if (fabs(roll - ins->horz_roll) < fabs(pitch - ins->horz_pitch)) {
			euler->roll = roll - ins->horz_roll;
			euler->pitch = pitch - ins->horz_pitch;
		} else {
			euler->roll = pitch - ins->horz_pitch;
			euler->pitch = roll - ins->horz_roll;
		}
	}

	euler->yaw = yaw_course;
}

extern void ins_update(float roll, float pitch, float yaw, ins_gps_t *ins_gps) {
	yaw = yaw < 0 ? yaw + 360 : yaw;
	ins_align(roll, pitch, yaw, ins_gps);
}

extern int ins_align_status(void) {
	ins_mgr_t *ins = &g_ins;

	return ins->init_flag;
}
