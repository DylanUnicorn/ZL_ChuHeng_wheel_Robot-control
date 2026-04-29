#ifndef _CRC_32_H
#define _CRC_32_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

uint32_t crc32_core(uint32_t *ptr, uint32_t len);

#endif
