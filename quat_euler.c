#include <math.h>
#include <stdio.h>
#include "../log/log.h"
#include "quat_euler.h"

static void quat_norm(quat_t *quat) {
	float norm = 0;

	norm = sqrt(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
	quat->w = quat->w / norm;
	quat->x = quat->x / norm;
	quat->y = quat->y / norm;
	quat->z = quat->z / norm;
}

extern void quat_init(quat_t * quat) {
	quat->w = 1;
	quat->x = 0;
	quat->y = 0;
	quat->z = 0;
}

extern void quat_set(quat_t * quat, float w, float x, float y, float z) {
	quat->w = w;
	quat->x = x;
	quat->y = y;
	quat->z = z;
	quat_norm(quat);
}

extern quat_t quat_rotate(quat_t * quat, float rad_x, float rad_y, float rad_z) {
	quat_t quat_ = *quat;
	float w = quat->w, x = quat->x, y = quat->y, z = quat->z;

	rad_x *= 0.5f;
	rad_y *= 0.5f;
	rad_z *= 0.5f;

	quat_.w += (-x * rad_x - y * rad_y - z * rad_z);
	quat_.x += (w * rad_x + y * rad_z - z * rad_y);
	quat_.y += (w * rad_y - x * rad_z + z * rad_x);
	quat_.z += (w * rad_z + x * rad_y - y * rad_x);

	quat_norm(&quat_);
	return quat_;
}

extern euler_t quat_to_euler(quat_t *quat) {
	euler_t euler;
	quat_t quat_ = *quat;

	quat_norm(&quat_);
	euler.roll = atan2f(quat_.w * quat_.x + quat_.y * quat_.z, 0.5f - quat_.x * quat_.x - quat_.y * quat_.y);
	euler.pitch = asinf(-2.0f * (quat_.x * quat_.z - quat_.w * quat_.y));
	euler.yaw = atan2f(quat_.x * quat_.y + quat_.w * quat_.z, 0.5f - quat_.y * quat_.y - quat_.z * quat_.z);

	return euler;
}

extern quat_t quat_from_euler(euler_t *euler) {
	quat_t quat;

	quat.w = cos(euler->roll / 2) * cos(euler->pitch / 2) * cos(euler->yaw / 2) + sin(euler->roll / 2) * sin(euler->pitch / 2) * sin(euler->yaw / 2);
	quat.x = sin(euler->roll / 2) * cos(euler->pitch / 2) * cos(euler->yaw / 2) - cos(euler->roll / 2) * sin(euler->pitch / 2) * sin(euler->yaw / 2);
	quat.y = cos(euler->roll / 2) * sin(euler->pitch / 2) * cos(euler->yaw / 2) + sin(euler->roll / 2) * cos(euler->pitch / 2) * sin(euler->yaw / 2);
	quat.z = cos(euler->roll / 2) * cos(euler->pitch / 2) * sin(euler->yaw / 2) - sin(euler->roll / 2) * sin(euler->pitch / 2) * cos(euler->yaw / 2);

	quat_norm(&quat);
	return quat;
}

extern axis_rotate_t quat_to_zero(quat_t *quat) {
	axis_rotate_t axis;
//	quat_t quat_ = *quat;
//	vector_t vector_y, vector_z;
//
//	vector_y = quat_vector_init(0, 1, 0);
//	vector_z = quat_vector_init(0, 0, 1);

	axis.z = quat_yaw_to_zero(quat);

//	quat_ = quat_rotate_axis_vector(quat, &vector_z, axis.z / 2);
//	axis.y = quat_pitch_to_zero(&quat_);
//
//	quat_ = quat_rotate_axis_vector(quat, &vector_z, axis.z / 2);
//	quat_ = quat_rotate_axis_vector(&quat_, &vector_y, axis.y / 2);
//	axis.x = quat_roll_to_zero(&quat_);

	return axis;
}

extern quat_t quat_rotate_axis_xyz(quat_t *quat, axis_rotate_t * axis) {
	quat_t quat_;
	vector_t vector_z;
//	vector_t vector_x, vector_y, vector_z;

//	vector_x = quat_vector_init(1, 0, 0);
//	vector_y = quat_vector_init(0, 1, 0);
	vector_z = quat_vector_init(0, 0, 1);

	quat_ = quat_rotate_axis_vector(quat, &vector_z, axis->z / 2);
//	quat_ = quat_rotate_axis_vector(&quat_, &vector_y, axis->y / 2);
//	quat_ = quat_rotate_axis_vector(&quat_, &vector_x, axis->x / 2);
	return quat_;
}

extern float quat_yaw_to_zero(quat_t *quat) {
	euler_t euler;
	vector_t vector;
	quat_t new_quat;
	euler_t euler_plus, euler_minus;
	float rotate_plus, rotate_minus;

	euler = quat_to_euler(quat);
	vector = quat_vector_init(0, 0, 1);
	LogNop("欧拉角航向归零前:%12.6f,%12.6f,%12.6f\r\n", RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));

	rotate_plus = euler.yaw;
	rotate_minus = -euler.yaw;

	new_quat = quat_rotate_axis_vector(quat, &vector, rotate_plus / 2);
	euler_plus = quat_to_euler(&new_quat);

	new_quat = quat_rotate_axis_vector(quat, &vector, rotate_minus / 2);
	euler_minus = quat_to_euler(&new_quat);

	if (fabs(euler_plus.yaw) < fabs(euler_minus.yaw)) {
		LogNop("欧拉角航向归零后:%12.6f,%12.6f,%12.6f,旋转:%12.6f\r\n", RAD2DEG(euler_plus.roll), RAD2DEG(euler_plus.pitch), RAD2DEG(euler_plus.yaw), RAD2DEG(rotate_plus));
		return rotate_plus;
	} else {
		LogNop("欧拉角航向归零后:%12.6f,%12.6f,%12.6f,旋转:%12.6f\r\n", RAD2DEG(euler_minus.roll), RAD2DEG(euler_minus.pitch), RAD2DEG(euler_minus.yaw), RAD2DEG(rotate_minus));
		return rotate_minus;
	}
}

extern float quat_pitch_to_zero(quat_t *quat) {
	euler_t euler;
	vector_t vector;
	quat_t new_quat;
	euler_t euler_plus, euler_minus;
	float rotate_plus, rotate_minus;

	euler = quat_to_euler(quat);
	vector = quat_vector_init(0, 1, 0);
	LogNop("欧拉角航向归零前:%12.6f,%12.6f,%12.6f\r\n", RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));

	rotate_plus = euler.pitch;
	rotate_minus = -euler.pitch;

	new_quat = quat_rotate_axis_vector(quat, &vector, rotate_plus / 2);
	euler_plus = quat_to_euler(&new_quat);

	new_quat = quat_rotate_axis_vector(quat, &vector, rotate_minus / 2);
	euler_minus = quat_to_euler(&new_quat);

	if (fabs(euler_plus.pitch) < fabs(euler_minus.pitch)) {
		LogNop("欧拉角航向归零后:%12.6f,%12.6f,%12.6f,旋转:%12.6f\r\n", RAD2DEG(euler_plus.roll), RAD2DEG(euler_plus.pitch), RAD2DEG(euler_plus.yaw), RAD2DEG(rotate_plus));
		return rotate_plus;
	} else {
		LogNop("欧拉角航向归零后:%12.6f,%12.6f,%12.6f,旋转:%12.6f\r\n", RAD2DEG(euler_minus.roll), RAD2DEG(euler_minus.pitch), RAD2DEG(euler_minus.yaw), RAD2DEG(rotate_minus));
		return rotate_minus;
	}
}

extern float quat_roll_to_zero(quat_t *quat) {
	euler_t euler;
	vector_t vector;
	quat_t new_quat;
	euler_t euler_plus, euler_minus;
	float rotate_plus, rotate_minus;

	euler = quat_to_euler(quat);
	vector = quat_vector_init(1, 0, 0);
	LogNop("欧拉角航向归零前:%12.6f,%12.6f,%12.6f\r\n", RAD2DEG(euler.roll), RAD2DEG(euler.pitch), RAD2DEG(euler.yaw));

	rotate_plus = euler.roll;
	rotate_minus = -euler.roll;

	new_quat = quat_rotate_axis_vector(quat, &vector, rotate_plus / 2);
	euler_plus = quat_to_euler(&new_quat);

	new_quat = quat_rotate_axis_vector(quat, &vector, rotate_minus / 2);
	euler_minus = quat_to_euler(&new_quat);

	if (fabs(euler_plus.roll) < fabs(euler_minus.roll)) {
		LogNop("欧拉角航向归零后:%12.6f,%12.6f,%12.6f,旋转:%12.6f\r\n", RAD2DEG(euler_plus.roll), RAD2DEG(euler_plus.pitch), RAD2DEG(euler_plus.yaw), RAD2DEG(rotate_plus));
		return rotate_plus;
	} else {
		LogNop("欧拉角航向归零后:%12.6f,%12.6f,%12.6f,旋转:%12.6f\r\n", RAD2DEG(euler_minus.roll), RAD2DEG(euler_minus.pitch), RAD2DEG(euler_minus.yaw), RAD2DEG(rotate_minus));
		return rotate_minus;
	}
}

static quat_t quat_conj(quat_t *quat) {
	quat_t conj;

	conj.w = quat->w;
	conj.x = -quat->x;
	conj.y = -quat->y;
	conj.z = -quat->z;

	return conj;
}

static quat_t quat_mul(quat_t *quat, quat_t *quat_) {
	quat_t mul;

	mul.w = quat->w * quat_->w - quat->x * quat_->x - quat->y * quat_->y - quat->z * quat_->z;
	mul.x = quat->w * quat_->x + quat->x * quat_->w + quat->y * quat_->z - quat->z * quat_->y;
	mul.y = quat->w * quat_->y + quat->y * quat_->w + quat->z * quat_->x - quat->x * quat_->z;
	mul.z = quat->w * quat_->z + quat->z * quat_->w + quat->x * quat_->y - quat->y * quat_->x;

	return mul;
}

extern vector_t quat_vector_init(float x, float y, float z) {
	vector_t vector;
	float norm = sqrt(x * x + y * y + z * z);

	vector.x = x / norm;
	vector.y = y / norm;
	vector.z = z / norm;

//	LogNop("向量:%12.6f,%12.6f,%12.6f", vector.x, vector.y, vector.z);
	return vector;
}

extern quat_t quat_from_vector(vector_t *vector, float rad) {
	quat_t quat;

	quat.w = cos(rad);
	quat.x = sin(rad) * vector->x;
	quat.y = sin(rad) * vector->y;
	quat.z = sin(rad) * vector->z;

	quat_norm(&quat);

//	LogNop("旋转轴四元数:%12.6f,%12.6f,%12.6f,%12.6f,向量:%12.6f,%12.6f,%12.6f,%12.6f", //
//			quat.w, quat.x, quat.y, quat.z, vector->x, vector->y, vector->z, AHRS_RAD2DEG(rad));
	return quat;
}

extern quat_t quat_rotate_axis_3d(quat_t *quat, vector_t *vector, float rad) {
	quat_t mul, conj;

	mul = quat_from_vector(vector, rad);
	conj = quat_conj(&mul);

	mul = quat_mul(&mul, quat);
	mul = quat_mul(&mul, &conj);

	LogNop("旋转四元数:%12.6f,%12.6f,%12.6f,%12.6f--->旋转后四元数:%12.6f,%12.6f,%12.6f,%12.6f,向量:%12.6f,%12.6f,%12.6f,%12.6f", //
			quat->w, quat->x, quat->y, quat->z, mul.w, mul.x, mul.y, mul.z, vector->x, vector->y, vector->z, RAD2DEG(rad));
	quat_norm(&mul);
	LogNop("旋转四元数归一:%12.6f,%12.6f,%12.6f,%12.6f--->旋转后四元数:%12.6f,%12.6f,%12.6f,%12.6f,向量:%12.6f,%12.6f,%12.6f,%12.6f", //
			quat->w, quat->x, quat->y, quat->z, mul.w, mul.x, mul.y, mul.z, vector->x, vector->y, vector->z, RAD2DEG(rad));
	return mul;
}

extern quat_t quat_rotate_axis_vector(quat_t *quat, vector_t *vector, float rad) {
	quat_t mul;

	mul = quat_from_vector(vector, rad);

	mul = quat_mul(&mul, quat);

	//LogNop("旋转四元数:%12.6f,%12.6f,%12.6f,%12.6f--->旋转后四元数:%12.6f,%12.6f,%12.6f,%12.6f,向量:%12.6f,%12.6f,%12.6f,%12.6f", //
	//		quat->w, quat->x, quat->y, quat->z, mul.w, mul.x, mul.y, mul.z, vector->x, vector->y, vector->z, AHRS_RAD2DEG(rad));
	quat_norm(&mul);
	//LogNop("旋转四元数归一:%12.6f,%12.6f,%12.6f,%12.6f--->旋转后四元数:%12.6f,%12.6f,%12.6f,%12.6f,向量:%12.6f,%12.6f,%12.6f,%12.6f", //
	//		quat->w, quat->x, quat->y, quat->z, mul.w, mul.x, mul.y, mul.z, vector->x, vector->y, vector->z, AHRS_RAD2DEG(rad));
	return mul;
}
