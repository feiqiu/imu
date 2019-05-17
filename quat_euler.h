#ifndef __LXT__QUAT_EULER_H__
#define __LXT__QUAT_EULER_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float w;
	float x;
	float y;
	float z;
} quat_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
} euler_t;

typedef struct {
	float x;
	float y;
	float z;
} vector_t;

typedef struct {
	float x;
	float y;
	float z;
} axis_rotate_t;

#define RAD2DEG(x)      ((x) * 57.2957795f)
#define DEG2RAD(x)      ((x) * 0.0174532925f)

extern void quat_init(quat_t * quat);

extern void quat_set(quat_t * quat, float w, float x, float y, float z);

extern axis_rotate_t quat_to_zero(quat_t *quat);

extern quat_t quat_from_euler(euler_t *euler);

extern euler_t quat_to_euler(quat_t *quat);

extern quat_t quat_rotate(quat_t * quat, float rad_x, float rad_y, float rad_z);

//pure imaginary 3d
extern quat_t quat_rotate_axis_3d(quat_t *quat, vector_t *vector, float rad);

extern quat_t quat_rotate_axis_vector(quat_t *quat, vector_t *vector, float rad);

extern quat_t quat_rotate_axis_xyz(quat_t *quat, axis_rotate_t * axis);

extern vector_t quat_vector_init(float x, float y, float z);

extern quat_t quat_from_vector(vector_t *vector, float rad);

extern float quat_yaw_to_zero(quat_t *quat);

extern float quat_pitch_to_zero(quat_t *quat);

extern float quat_roll_to_zero(quat_t *quat);

#ifdef __cplusplus
}
#endif

#endif
