#include "math.h"
#include "ahrs.h"
#include "location.h"
#include "../log/log.h"

#define pi             3.14159265f

typedef struct {
	float period;
	float sx;
	float sy;
	float sz;
	float lat_meter;
	float lon_meter;
	ins_gps_t loc_ins;
	euler_t euler;
} loc_mgr_t;

static loc_mgr_t g_loc;

extern void loc_set_period(float sec) {
	g_loc.period = sec;
}

extern void loc_init(float Hz) {
	g_loc.period = 1.0f / Hz;
	g_loc.loc_ins.pos = 'V';
	g_loc.loc_ins.lat_sn = 'S';
	g_loc.loc_ins.lon_ew = 'W';
}

extern void loc_align(ins_gps_t * loc_ins) {
	float lon = 0, lat = 0;

	g_loc.sx = 0;
	g_loc.sy = 0;
	g_loc.sz = 0;

	g_loc.loc_ins = *loc_ins;
	lon = loc_ins->lon / 1000000 + (loc_ins->lon % 1000000) / 600000.0f;
	lat = loc_ins->lat / 1000000 + (loc_ins->lat % 1000000) / 600000.0f;

	g_loc.lat_meter = 111000.0f;
	g_loc.lon_meter = 111000.0f * cos(lat * pi / 180);
	g_loc.loc_ins.lon = (unsigned int) (lon * 1000000);
	g_loc.loc_ins.lat = (unsigned int) (lat * 1000000);
}

extern void loc_pos_update(euler_t *euler, unsigned short v, ins_gps_t * loc_out) {
	float vx = 0, vy = 0, vz = 0, yaw = 0, pitch = 0;
	unsigned int lon = 0, lat = 0, alt = 0;
	float lon_f = 0, lat_f = 0, velocity_m_s = 0;

	if (g_loc.loc_ins.pos != 'A') {
		*loc_out = g_loc.loc_ins;
		return;
	}

	velocity_m_s = v * 0.99 / 3.6f;//0.99根据查看轨迹发现里程误差的修正系数
	yaw = DEG2RAD(euler->yaw);
	pitch = DEG2RAD(euler->pitch);

	vx = velocity_m_s * cos(pitch) * sin(yaw);
	vy = velocity_m_s * cos(pitch) * cos(yaw);
	vz = velocity_m_s * sin(pitch);

//	g_loc.sx = g_loc.sx + vx * g_loc.period * 0.98;
//	g_loc.sy = g_loc.sy + vy * g_loc.period * 0.98;
//	g_loc.sz = g_loc.sz + vz * g_loc.period * 0.98;

	g_loc.sx = g_loc.sx + vx * g_loc.period;
	g_loc.sy = g_loc.sy + vy * g_loc.period;
	g_loc.sz = g_loc.sz + vz * g_loc.period;

	lon = g_loc.loc_ins.lon + (g_loc.sx / g_loc.lon_meter * 1000000);
	lat = g_loc.loc_ins.lat + (g_loc.sy / g_loc.lat_meter * 1000000);
	alt = g_loc.loc_ins.alt + g_loc.sz;

	lon_f = lon / 1000000 + (lon % 1000000) / 1000000.0 * 0.6;
	lat_f = lat / 1000000 + (lat % 1000000) / 1000000.0 * 0.6;

	loc_out->pos = 'A';
	loc_out->lon = lon_f * 1000000;
	loc_out->lat = lat_f * 1000000;
	loc_out->alt = alt;
	loc_out->v = v;
	loc_out->lon_ew = g_loc.loc_ins.lon_ew;
	loc_out->lat_sn = g_loc.loc_ins.lat_sn;
	loc_out->course = euler->yaw;
	/*
	 LogUart("v:%d,yaw:%f,v_:%f,%f,%f,s_:%f,%f,%f,meter:%f,%f,loc_lat:%d,%d--%f,%f,lat:%d,%d,%f,%f",//
	 velocity, loc_ahrs->yaw, vx, vy, vz,//
	 g_loc.sx, g_loc.sy, g_loc.sz,
	 g_loc.lon_meter, g_loc.lat_meter,//
	 g_loc.loc_ins.lat, g_loc.loc_ins.lon,//
	 (g_loc.sx / g_loc.lon_meter * 1000000), (g_loc.sy / g_loc.lat_meter * 1000000),//
	 lat, lon,lat_f , lon_f);*/
}
