#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

// 初始化运动学参数
void KIN_Init(float wheel_radius_m, float half_track_m, float max_rpm);

// 计算左右侧目标值（device units: 0.1 r/min, 带限制）
// vx: 前向速度 (m/s), omega: 角速度 (rad/s)
void KIN_ComputeLeftRight(float vx, float omega, int32_t *out_left, int32_t *out_right);

#endif
