#ifndef CFG_RTOS_H
#define CFG_RTOS_H

/* ============================================================================
 * RTOS 参数（X5）—— 目前**还没启用**，裸机主循环在跑
 *
 * 为什么现在就把这些常数写出来：
 *   README §9 的 R1-R6 已经把 FreeRTOS 排进路线图。如果不先把"优先级/栈深/
 *   队列长度"这些数定成常量，迁移时会有一堆魔数散落在 osal 层和任务函数里，
 *   出了栈溢出/优先级反转根本没法调。这里先给一版保守估算，
 *   真上 RTOS 时按实测（uxTaskGetStackHighWaterMark）再调。
 *
 * 取值依据：
 *   - 栈深单位是 **StackType_t（4 字节）**，不是字节；
 *   - 优先级数字越大越高（FreeRTOS 语义）；
 *   - 运动任务栈给 256 字 = 1 KB，因为它要跑 S 曲线插值（局部数组不小）；
 *   - 通信任务栈 384 字 = 1.5 KB，协议解析 + 字符串格式化都在它里面；
 *   - 日志任务优先级最低，串口慢，不能挡住运动。
 * ============================================================================ */

/* ---- 任务栈深（单位：字 / 4 字节）---- */
#define CFG_RTOS_STACK_MOTION      256u    /* 运动控制：S 曲线插值 + 加减速 */
#define CFG_RTOS_STACK_COMM        384u    /* 通信：收帧、解析、回包、日志字符串 */
#define CFG_RTOS_STACK_LOG         256u    /* 日志：阻塞发串口，慢允许 */
#define CFG_RTOS_STACK_IDLE        128u    /* 空闲任务（FreeRTOS 要求最小 128） */

/* ---- 任务优先级（数字越大越高，configMAX_PRIORITIES 默认 7）---- */
#define CFG_RTOS_PRIO_MOTION       4u      /* 最高：脉冲/急停不能被通信拖慢 */
#define CFG_RTOS_PRIO_COMM         3u
#define CFG_RTOS_PRIO_LOG          1u
#define CFG_RTOS_PRIO_IDLE         0u

/* ---- 队列长度（元素个数）---- */
#define CFG_RTOS_QUEUE_CMD_LEN     8u      /* 命令队列：连发 20 条也不会瞬间塞爆 */
#define CFG_RTOS_QUEUE_LOG_LEN     16u     /* 日志队列：满了就丢最旧的，绝不阻塞运动 */

/* ---- 时基与临界区 ---- */
#define CFG_RTOS_TICK_HZ           1000u   /* 1 kHz 滴答，和现在的 HAL_GetTick 语义一致 */
#define CFG_RTOS_MAX_SYSCALL_PRIO  5u      /* 对应 configMAX_SYSCALL_INTERRUPT_PRIORITY
                                            * （NVIC 分组 GROUP_4 下按 0-15 意义计算，
                                            *  5 以上优先级的中断不许调用 FreeRTOS API） */

/* ---- 看门狗 ---- */
#define CFG_RTOS_WDG_TIMEOUT_MS    2000u   /* 独立看门狗超时；比 CFG_CMD_TIMEOUT_MS 短，
                                            * 任何任务卡死都会先被硬件复位而不是让车继续跑 */

#endif /* CFG_RTOS_H */
