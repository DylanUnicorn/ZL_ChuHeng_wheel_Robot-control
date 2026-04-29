#include "accel.h"

/**
 * @brief  梯形控制
 * @param  设定；反馈；加速度
 * @retval 输出值
 */
float ramp_control(float set, float ref, float accel)
{
    float ramp = LIMIT_MAX_MIN(accel, 1, 0) * (set - ref);

    return fabs(ramp + ref) > fabs(set) ? set : ref + ramp;
}
