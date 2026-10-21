#ifndef CFG_MOTOR_H
#define CFG_MOTOR_H

/* ============================================================
 * 电机与机械参数
 *
 *    定时器时钟必须与 Core/Src/main.c 的 SystemClock_Config() 一致！
 *    当前：HSE 8MHz, PLLM=8, PLLN=336, PLLP=2 → SYSCLK 168MHz
 *          APB1 = HCLK/4 = 42MHz → TIM2 时钟 = 42x2 = 84MHz
 *          APB2 = HCLK/2 = 84MHz → TIM1 时钟 = 84x2 = 168MHz
 *    改 .ioc 的时钟树后，这里必须同步，否则转速静默出错（不报警）。
 * ============================================================ */
#define CFG_TIM1_CLK_HZ     168000000UL     /* APB2 定时器 */
#define CFG_TIM2_CLK_HZ      84000000UL     /* APB1 定时器 */

/* PWM 载波：固定 ARR，靠改 PSC 调频 */
#define CFG_PUL_ARR         100

/* 脉冲频率限幅
 *
 * 上限 5000Hz 的依据（不是随便定的）：
 *   - 2500Hz @1600 细分 ≈ 94rpm；5000Hz ≈ 188rpm，已经够用；
 *   - M860D 要求脉冲高低电平各 ≥2.5µs，也就是理论上限 200kHz，
 *     但真正卡住的是光耦（TLP 型输入 7~20mA、开关速度有限）
 *     和"频率越高电机越容易失步 → 电流上升"这个过载路径。
 *   - 旧值 100000Hz 会让上位机一条 FR100000 直接把电机拖进失步状态。
 */
#define CFG_PUL_MIN_HZ      30
#define CFG_PUL_MAX_HZ      5000

/* --- 编译期自检：PSC 是 16 位，超了就静默截断，转速会错得莫名其妙 ----- */
#if ((CFG_TIM1_CLK_HZ / (CFG_PUL_MIN_HZ * CFG_PUL_ARR)) > 65536UL)
#error "CFG_PUL_MIN_HZ 太低：TIM1 的 PSC 超过 16 位，会被静默截断"
#endif
#if ((CFG_TIM2_CLK_HZ / (CFG_PUL_MIN_HZ * CFG_PUL_ARR)) > 65536UL)
#error "CFG_PUL_MIN_HZ 太低：TIM2 的 PSC 超过 16 位，会被静默截断"
#endif
#if (CFG_PUL_MAX_HZ > (CFG_TIM2_CLK_HZ / CFG_PUL_ARR))
#error "CFG_PUL_MAX_HZ 太高：会算出 0 或负的 PSC（最低可到 tim_clk/ARR）"
#endif

/* 电机编号（车体语义，不是通道号） */
#define CFG_MOTOR_FL        0
#define CFG_MOTOR_RL        1
#define CFG_MOTOR_FR        2
#define CFG_MOTOR_RR        3

#endif /* CFG_MOTOR_H */
