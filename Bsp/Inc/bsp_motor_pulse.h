#ifndef BSP_MOTOR_PULSE_H
#define BSP_MOTOR_PULSE_H

#include "types.h"
#include "cfg_motor.h"

/* ============================================================================
 * 四路步进脉冲输出（PUL）＋ 四路方向（DIR）
 *
 * 本层只知道"某一路通道"，不知道"左前轮/右后轮"——那是 App 层的语义。
 * 通道号直接取 Config 里的车体编号，两边永远一致。
 * ============================================================================ */

typedef enum
{
    MOTOR_CH_L1 = CFG_MOTOR_FL,   /* TIM1_CH1  PE9   左前 */
    MOTOR_CH_L2 = CFG_MOTOR_RL,   /* TIM1_CH2  PE11  左后 */
    MOTOR_CH_R1 = CFG_MOTOR_FR,   /* TIM2_CH1  PA0   右前 */
    MOTOR_CH_R2 = CFG_MOTOR_RR    /* TIM2_CH2  PA1   右后 */
} MotorChannel;

typedef enum
{
    MOTOR_DIR_CW  = 0,            /* DIR 引脚低电平 */
    MOTOR_DIR_CCW = 1             /* DIR 引脚高电平 */
} MotorDir;

/* ---------------------------------------------------------------------------
 * 初始化：打开 4 路 PWM 通道 + 高级定时器 TIM1 的主输出(MOE)，
 *         然后保持两路定时器停止，等 Bsp_MotorPulse_StartAll() 启动。
 *         （时钟、GPIO、AF 复用、时基、PWM 占空比全部由 CubeMX 生成代码完成）
 * ------------------------------------------------------------------------ */
void Bsp_MotorPulse_Init(void);

/* 设置某一路通道的方向 */
void Bsp_MotorPulse_SetDir(MotorChannel ch, MotorDir dir);

/* ---------------------------------------------------------------------------
 * 设置某一路通道的脉冲频率(Hz)；hz == 0 表示"停这一侧时基"。
 *
 * ⚠️ 同侧两路共用一路定时器时基：设 L1 也会把 L2 的频率改掉（反之亦然），
 *    这是硬件决定的，不是 bug。想左右频率不同 → 左右各给一次即可。
 *
 * ⚠️⚠️ 调用顺序有硬约束：**必须先 Bsp_MotorPulse_StartAll() 再 SetFreq()**。
 *    因为 SetFreq(0) 会 HAL_TIM_Base_Stop() 掉整路时基；如果先 SetFreq 再
 *    StartAll，会把本该停转的那一侧用旧分频值重新启动。
 * ------------------------------------------------------------------------ */
void Bsp_MotorPulse_SetFreq(MotorChannel ch, u32 hz);

/* 启动两路时基（所有通道一起）；要停某侧用 SetFreq(ch, 0) */
void Bsp_MotorPulse_StartAll(void);

/* ---------------------------------------------------------------------------
 * 幂等兜底：确认两路时基**真的**停了
 *
 * 为什么需要它：一次上机调试中出现过"软件侧速度已清零、s_alive_ms 也清零，
 * 但 TIM1/TIM2 的 CR1.CEN 仍为 1（1200 Hz 还在输出）"的现象，轮子持续转。
 * 这类状态无论根因是什么都不该存在——速度命令为 0 就不能有脉冲。
 *
 * 本函数只做寄存器判断，CEN 已为 0 时**什么都不做**（零开销，可每圈调用）。
 * 因此可以直接放进主循环的"速度都是 0"分支做安全网。
 * ------------------------------------------------------------------------ */
void Bsp_MotorPulse_EnsureStopped(void);

/* 诊断计数（纯读取，供调试器观察；标 used 防止被 --gc-sections 回收）
 *   StartCount      —— StartAll() 被调用的次数
 *   StopCount       —— SetFreq(ch, 0) 走停车分支的次数
 *   ForcedStopCount —— EnsureStopped() 真正"抓到"定时器还在跑的次数
 * 判读：ForcedStopCount 持续增长 = 有代码在速度应为 0 时把定时器又打开了。
 *
 * 说明：这三个计数是**文件内静态变量**，调试器直接看即可（不需要函数，
 * 因为 -Wl,--gc-sections 会把没有调用者的 getter 整段回收）：
 *   >-interpreter-exec console "print s_start_cnt"
 *   >-interpreter-exec console "print s_stop_cnt"
 *   >-interpreter-exec console "print s_forced_cnt"
 */
u32 Bsp_MotorPulse_GetForcedStopCount(void);

#endif /* BSP_MOTOR_PULSE_H */
