#ifndef APP_RC_H
#define APP_RC_H

#include "types.h"
#include "cfg_rc.h"

/* ============================================================================
 * App 层 — 遥控业务（PWM/PPM 备选方案，阶段 A：只采集、只打印）
 *
 * ⚠️ 默认方案是 i-BUS（Service/ibus.c + App/Src/rc_control.c），
 *    本模块被 CFG_RC_SOURCE_PWM 包住，默认配置下编空、不参与构建。
 *
 * 分工：
 *   Bsp（bsp_rc.c）        : 线上的脉冲 → 6 个 µs 值
 *   App（本文件）          : 6 个 µs 值 → "信号有没有" / "每个通道多少" /（阶段 B）→ 左右 Hz
 *
 * 阶段 A 里这个文件**不 include motion.h、不调任何 Motion_* 函数**，
 * 所以它跟正在改动的运动控制程序完全解耦，可以并行开发。
 * ============================================================================ */

#if (CFG_RC_SOURCE_PWM != 0)

/* 初始化遥控采集（内部只调 Bsp_Rc_Init）。App_Init() 里调一次。 */
void App_Rc_Init(void);

/* 主循环每圈调用一次：目前只检测"信号来了/丢了"并各打一行日志 */
void App_Rc_Tick(void);

/* 串口命令 'R' 的回调：打印 6 个通道的 µs 值 */
void App_Rc_Report(void);

#endif /* CFG_RC_SOURCE_PWM */

#endif /* APP_RC_H */
