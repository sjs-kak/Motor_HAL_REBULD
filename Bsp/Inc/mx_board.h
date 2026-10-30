#ifndef BSP_MX_BOARD_H
#define BSP_MX_BOARD_H

/* ============================================================================
 * 防腐层（Anti-Corruption Layer）
 *
 * 全工程**唯一**允许 #include CubeMX 生成头文件的地方
 * （main.h / tim.h / usart.h / gpio.h）。
 *
 * 其它任何一层想用 htim1、huart1、GPIO_PIN_x、DIR_xx_Pin，
 * 都必须经过这里翻译成 BOARD_xxx 名字。
 * 这样将来在 .ioc 里改外设名/换引脚/换串口，只改这一个文件。
 *
 * ⚠️ 本文件属于 Bsp 层：不允许出现任何上层（Service/App）概念。
 * ============================================================================ */

#include "main.h"
#include "tim.h"
#include "usart.h"

/* ---------------------------------------------------------------------------
 * 定时器：左右两侧各一路独立时基（差速转向需要左右频率不同）
 *   左侧 FL/RL -> TIM1
 *   右侧 FR/RR -> TIM2
 * ------------------------------------------------------------------------ */
#define BOARD_HTIM_LEFT           (&htim1)
#define BOARD_HTIM_RIGHT          (&htim2)

/* ---------------------------------------------------------------------------
 * 串口一：与上位机通信的那一路 —— USART1，PB6=TX / PB7=RX（AF7）。
 *
 * ⚠️ 2026-10-26 由 PA9/PA10 改到 PB6/PB7（CubeMX + Core/Src/usart.c 已同步）。
 *    为什么要挪：PA9/PA10 那组在板上被别的东西占着，PB6/PB7 才引到排针。
 *    GPIO 复用由 .ioc / Core/Src/usart.c 配置，本文件只翻译名字。
 * ------------------------------------------------------------------------ */
#define BOARD_HUART_HOST          (&huart1)
#define BOARD_UART_HOST_INSTANCE  USART1

/* ---------------------------------------------------------------------------
 * 串口二：遥控接收机的 SERVO i-BUS —— USART3，只收，RX=PB11（AF7）。
 *
 *   FS-iA6B 的 SERVO 口出的是 115200 8N1 的串行帧（32 字节，0x20 0x40 帧头），
 *   不是 PPM 脉冲。所以这里走串口，不用输入捕获。
 *   PB10（USART3_TX）不接，接收机只发不收。
 *
 *   与下面的 PWM 方案（TIM3_CH1 / PA6）**二选一**，由 cfg_rc.h 的
 *   CFG_RC_SOURCE_IBUS / CFG_RC_SOURCE_PWM 决定；两套硬件定义互不冲突。
 * ------------------------------------------------------------------------ */
#define BOARD_HUART_RC            (&huart3)
#define BOARD_UART_RC_INSTANCE    USART3

/* ---------------------------------------------------------------------------
 * PUL 脉冲输出引脚（AF 复用）
 *
 * ⚠️ CubeMX 不为 AF 复用引脚生成 User Label 宏，下面 8 行是手工补的引脚表。
 *    来源：Core/Src/tim.c 的 HAL_TIM_MspPostInit()
 *      tim.c:203  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;  (GPIOE, AF1_TIM1)
 *      tim.c:225  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;   (GPIOA, AF1_TIM2)
 *
 * ⚠️ 在 CubeMX 里改过引脚分配后，这里必须同步修改；CubeMX 不会提醒你，
 *    编译器也不会报错（PUL 是硬件定时器输出，写错了只是"某个轮子不转"）。
 *
 * 更彻底的做法：以后在 CubeMX 里给 PE9/PE11/PA0/PA1 设 User Label
 * (PUL_LF / PUL_LR / PUL_RF / PUL_RR)，生成的宏就可以直接替换下面 8 行。
 * ------------------------------------------------------------------------ */
#define PUL_LF_PIN   GPIO_PIN_9        /* PE9   TIM1_CH1  左前 */
#define PUL_LF_PORT  GPIOE
#define PUL_LR_PIN   GPIO_PIN_11       /* PE11  TIM1_CH2  左后 */
#define PUL_LR_PORT  GPIOE
#define PUL_RF_PIN   GPIO_PIN_0        /* PA0   TIM2_CH1  右前 */
#define PUL_RF_PORT  GPIOA
#define PUL_RR_PIN   GPIO_PIN_1        /* PA1   TIM2_CH2  右后 */
#define PUL_RR_PORT  GPIOA

/* ---------------------------------------------------------------------------
 * 遥控接收机 —— PWM / PPM 单线备选方案（TIM3_CH1 + PA6）
 *
 * 需要 cfg_rc.h 里 CFG_RC_SOURCE_PWM = 1 才会被 bsp_rc.c 用到；
 * 默认方案是上面的 i-BUS（USART3 / PB11）。两套定义同时存在不冲突。
 *
 * ⚠️ TIM3 与 PA6 **故意不在 .ioc 里**：多人并行开发期间，只要有人再点一次
 *    CubeMX 的 GENERATE，Core/ 下正在被改的 main.c 就有被覆盖的风险。
 *    因此这一路的实例/引脚/AF/中断号全部由 Bsp/Src/bsp_rc.c 手工配置，
 *    与 tim.c 生成的那两个定时器互不相干。
 *    TODO（等运动控制稳定后）：搬进 .ioc，同时删掉 bsp_rc.c 里手工建句柄
 *    的那一段和下面的 BOARD_RC_IRQ_HANDLER 强定义，否则链接期重复定义。
 *
 * 换引脚时改这里的 6 行即可（bsp_rc.c 只引用宏，不写死引脚名）。
 *   PA6  = TIM3_CH1  (AF2)  ← 首选，板上好引，且不与现有 8 个电机脚冲突
 *   备选：PC6/PB4 = TIM3_CH1；PB6/PD12 = TIM4_CH1
 *   ⚠️ 不能用 PA0：那是 TIM2_CH1 的右前脉冲。
 *   ⚠️ 也不能用 PB6/PB7：那两个已经给了 USART1（上位机）。
 *
 * 接收机供电 5V（手册 4.0~6.5V，不能接 3.3V），信号 3.3V CMOS 可直连 MCU。
 * ------------------------------------------------------------------------ */
#define BOARD_RC_TIM_INSTANCE     TIM3
#define BOARD_RC_TIM_CHANNEL      TIM_CHANNEL_1
#define BOARD_RC_TIM_IRQn         TIM3_IRQn
#define BOARD_RC_IRQ_HANDLER      TIM3_IRQHandler
#define BOARD_RC_TIM_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
#define BOARD_RC_GPIO_PORT        GPIOA
#define BOARD_RC_GPIO_PIN         GPIO_PIN_6
#define BOARD_RC_GPIO_AF          GPIO_AF2_TIM3
#define BOARD_RC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/* 捕获极性：接收机输出"正脉冲 PPM"用 RISING。
 * 若 Bsp_Rc_GetEdgeCount() 一直是 0、而 CH1 上确实有 3.3V 摆幅，
 * 说明这台接收机是"反相 PPM"（空闲高、脉冲低），把下面这行改成
 * TIM_INPUTCHANNELPOLARITY_FALLING，同时把 PULL 改成 GPIO_PULLUP。 */
#define BOARD_RC_IC_POLARITY      TIM_INPUTCHANNELPOLARITY_RISING
#define BOARD_RC_GPIO_PULL        GPIO_PULLDOWN

/* ---------------------------------------------------------------------------
 * DIR 方向引脚：CubeMX 已生成（Core/Inc/main.h:60-67），这里只做改名
 * ------------------------------------------------------------------------ */
#define DIR_LF_PIN   DIR_LF_Pin
#define DIR_LF_PORT  DIR_LF_GPIO_Port
#define DIR_LR_PIN   DIR_LR_Pin
#define DIR_LR_PORT  DIR_LR_GPIO_Port
#define DIR_RF_PIN   DIR_RF_Pin
#define DIR_RF_PORT  DIR_RF_GPIO_Port
#define DIR_RR_PIN   DIR_RR_Pin
#define DIR_RR_PORT  DIR_RR_GPIO_Port

#endif /* BSP_MX_BOARD_H */
