#ifndef _ACCEL_H
#define _ACCEL_H

#include "tools.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// 加速策略选择
float ramp_control(float set, float ref, float accel);

#endif
