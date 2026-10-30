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
 * 初始化：只打开高级定时器 TIM1 的主输出(MOE)；
 *         4 路 PWM 通道（CCxE）与计数器（CEN）保持关闭，等 SetFreq(ch, 非零)
 *         启动对应一侧。上电不强开 PWM，避免复位瞬间先出几个脉冲。
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
 * 非零频率先加载分频、再启动该侧；零频率停止该侧（并且只在停着时写 UG）。
 * 不需要先调用 StartAll，避免短暂重启另一侧已停止的电机。
 *
 * "停"是三道闸一起关，任何一道生效就没有脉冲：
 *   ① CEN = 0          —— 计数器停
 *   ② CCxE = 0         —— 通道输出门关（不依赖 CEN，第二道闸）
 *   ③ TIM1 的 MOE = 0  —— 高级定时器主输出，为 0 时硬件强制无效电平（第三道闸）
 * 所以 SetFreq(ch, 0) 之后引脚上**一定**没有脉冲，不需要额外的"兜底确认"函数。
 * ------------------------------------------------------------------------ */
void Bsp_MotorPulse_SetFreq(MotorChannel ch, u32 hz);

/* 旧接口：无条件启动两侧时基（CCxE 重新置位 + CEN 置位）。
 * 遥控路径不调用；正常停车用 SetFreq(ch, 0)。
 * 注意它不看频率寄存器：两侧会以**上一次写入的 PSC** 开始跑。 */
void Bsp_MotorPulse_StartAll(void);

#endif /* BSP_MOTOR_PULSE_H */
