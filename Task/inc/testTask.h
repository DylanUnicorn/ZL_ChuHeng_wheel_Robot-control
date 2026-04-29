#ifndef __TEST_TASK_H
#define __TEST_TASK_H

#include "ZLA_Motor.h"

/* 暴露驱动节点实例供 CAN 回调函数使用 */
/* 说明：node 1 控制前轮（前左、前右），node 2 控制后轮（后左、后右）
 * 变量名使用 `front_drive`/`rear_drive` 表示驱动节点（不是单个轮子）
 */
extern ZLA_Motor front_drive;   // 驱动节点 1: 前轮驱动（front-left, front-right）
extern ZLA_Motor rear_drive;    // 驱动节点 2: 后轮驱动（rear-left, rear-right）

/* 调试模式开关（0 = Normal, 1 = Debug） */
extern int test_debug_mode;

/* 任务入口函数 */
void test_task(void const * argument);

/* 设置调试模式接口 */
void TestTask_SetDebug(int enable);

#endif
