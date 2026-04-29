#include "kinematics.h"
#include <math.h>
#include "robot_config.h"

#define M_PI 3.14159265358979323846f
void KIN_Init(float wheel_radius_m, float half_track_m, float max_rpm)
{
}

void KIN_ComputeLeftRight(float vx, float omega, int32_t *out_left, int32_t *out_right)
{
    if (!out_left || !out_right) return;


    float wheel_r = robot_config.wheel_radius_m;
    float half_track = robot_config.half_track_m;
    float max_rpm = robot_config.max_rpm;


    float v_left = vx - omega * half_track;
    float v_right = vx + omega * half_track;


    float left_rpm = (v_left / (2.0f * (float)M_PI * wheel_r)) * 60.0f;
    float right_rpm = (v_right / (2.0f * (float)M_PI * wheel_r)) * 60.0f;

    float left_dev = left_rpm * robot_config.vel_unit_scale;
    float right_dev = right_rpm * robot_config.vel_unit_scale;

    int32_t left_out = (int32_t)roundf(left_dev * 1);
    int32_t right_out = (int32_t)roundf(right_dev * 1);

    if (left_out > max_rpm) left_out = max_rpm;
    if (left_out < -max_rpm) left_out = -max_rpm;
    if (right_out > max_rpm) right_out = max_rpm;
    if (right_out < -max_rpm) right_out = -max_rpm;

    *out_left = left_out;
    *out_right = right_out;
}
