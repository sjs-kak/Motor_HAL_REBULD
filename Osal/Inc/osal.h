#ifndef OSAL_OSAL_H
#define OSAL_OSAL_H

#include "types.h"

/* ============================================================================
 * OSAL — 操作系统抽象层（Operating System Abstraction Layer）
 *
 * 全工程唯一允许直接调用 HAL_Delay() / HAL_GetTick() 的地方。
 * 上层一律改成 Osal_DelayMs() / Osal_GetTickMs()。
 *
 * 为什么值得加这一层：
 *   现在裸机 → HAL_Delay 就是空转等 SysTick；
 *   将来换 FreeRTOS → 只要把这两个函数改成 vTaskDelay / xTaskGetTickCount，
 *   上层（Bsp/Service/App）一行都不用动。
 *
 * 本层依赖 Config（类型）和 CubeMX 生成层，不依赖任何上层。
 * ============================================================================ */

void Osal_DelayMs(u32 ms);        /* 阻塞延时 ms 毫秒（现在 = HAL_Delay） */
u32  Osal_GetTickMs(void);        /* 系统启动以来的毫秒滴答（现在 = HAL_GetTick） */

/* Short ISR/main snapshot protection; restore the original interrupt mask. */
u32  Osal_EnterCritical(void);
void Osal_ExitCritical(u32 key);

#endif /* OSAL_OSAL_H */
