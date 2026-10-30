#ifndef BSP_RC_H
#define BSP_RC_H

#include "types.h"
#include "cfg_rc.h"

/* ============================================================================
 * Bsp 层 — 遥控接收机（PWM / PPM 单线）采集
 *
 * ⚠️ 这是**备选**方案：默认走 i-BUS（USART3/PB11，见 Service/ibus.c）。
 *    整个模块被 cfg_rc.h 的 CFG_RC_SOURCE_PWM 包住，默认配置下它是空的：
 *    .c 编不出任何符号，也不会去占 TIM3/PA6。
 *    只有把 CFG_RC_SOURCE_PWM 改成 1 才会生效，届时 app_rc.c 也必须编译。
 *
 * 本模块只负责一件事：**把线上的一串脉冲变成 6 个 µs 值**。
 * 通道含义、死区、映射到电机频率一律在 App 层（app_rc.c）决定 ——
 * 这里不知道"油门"和"转向"是什么，只知道"第 n 个槽多宽"。
 *
 * 硬件：TIM3_CH1 = PA6，输入捕获，1 µs 分辨率（见 Config/Inc/cfg_rc.h）。
 *       ⚠️ TIM3 不在 .ioc 里，句柄由本文件手工建立，与 CubeMX 生成代码无关。
 *
 * 阶段 A：采集 + 打印，不驱动电机。CFG_RC_DRIVE_ENABLE = 0。
 * ============================================================================ */

#if (CFG_RC_SOURCE_PWM != 0)

/* 配置 GPIO/TIM3/中断并开始捕获。App_Init() 里调一次。 */
void Bsp_Rc_Init(void);

/* ---------------------------------------------------------------------------
 * 取最近一帧的通道值（µs）
 *   us/n : 上层缓冲区与希望取几个通道
 *   返回 1 = 拿到（信号新鲜，age <= CFG_RC_LOST_MS），0 = 没信号/还没解出完整帧
 *
 * ⚠️ 帧不完整（只收到一半）时返回 0，绝不会把半帧当有效数据交出去。
 * ------------------------------------------------------------------------ */
u8 Bsp_Rc_GetChannels(u16 *us, u8 n);

/* 信号是否新鲜（有完整帧且未超时）。阶段 A 用它打"信号来了/丢了"日志。 */
u8 Bsp_Rc_HasSignal(void);

/* 距最近一次捕获沿过了多少毫秒。没接过线时返回自 Init 起的时间。 */
u32 Bsp_Rc_GetAgeMs(void);

/* 调试计数：总捕获沿数 / 解出的完整帧数。
 *   edges=0            → 一个沿都没抓到：查接线、接收机灯、IC 极性
 *   edges>0 而 frames=0 → 抓到了但组不成帧：多半是 CFG_RC_SYNC_US 判据不对 */
u32 Bsp_Rc_GetEdgeCount(void);
u32 Bsp_Rc_GetFrameCount(void);

/* 初始化是否成功（0 = TIM3 配置失败，其余接口一律返回"无信号"） */
u8 Bsp_Rc_IsReady(void);
#endif /* CFG_RC_SOURCE_PWM */

#endif /* BSP_RC_H */
