#ifndef _LOC_UPDATE_H__
#define _LOC_UPDATE_H__

#include "quat_euler.h"

typedef struct {
	char pos;
	char lon_ew;
	char lat_sn;
	unsigned int lon;
	unsigned int lat;
	unsigned int alt;
	float course;
	unsigned short v;
} ins_gps_t;

extern void loc_init(float Hz);

extern void loc_align(ins_gps_t *loc_ins);

extern void loc_pos_update(euler_t *euler, unsigned short velocity, ins_gps_t * loc_out);

extern void loc_set_period(float sec);

#endif
