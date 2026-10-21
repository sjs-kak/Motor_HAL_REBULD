#ifndef BSP_UART_H
#define BSP_UART_H

#include "types.h"

/* ============================================================================
 * 串口（与上位机通信）— 只负责"把收到的字节交给上层"，
 * 不解析协议、不组帧（那是 Service 层 protocol.c 的事）。
 *
 * ⚠️ 绝不能再自己定义 USART1_IRQHandler！
 *    Core/Src/stm32f4xx_it.c 里已经有一个（它调用 HAL_UART_IRQHandler(&huart1)），
 *    重复定义会导致串口整个收不到数据（链接不报错，行为直接错）。
 * ============================================================================ */

/* 收到 1 字节时回调；由上层注册（App/Src/app_init.c 注册 Protocol_FeedByte） */
typedef void (*Bsp_Uart_RxByteHandler)(u8 byte);

void Bsp_Uart_RxStart(void);                                   /* 启动中断接收 */
void Bsp_Uart_SetRxByteHandler(Bsp_Uart_RxByteHandler handler); /* 注册回调 */

/* ---------------------------------------------------------------------------
 * 多路串口：按 Instance 注册接收通道（X2）
 *
 * 为什么必须按 Instance 分发：
 *   中断回调 HAL_UART_RxCpltCallback() 是**全局唯一**的，两路串口共用它。
 *   旧写法在里面只认"主机串口"，一旦以后接相机串口（USART2/3/6），
 *   相机字节就会被当成上位机命令喂进协议状态机 —— 表现为"车自己乱动"。
 *
 * 用法：Bsp_Uart_OpenChannel(&huart2, Cam_RxByte);
 *             ↑ 参数是 void* 是为了不让本头文件拖进 HAL：
 *               App/Service 层也会 include 这个头，而它们拿不到 HAL 头文件。
 *
 * 返回值：0 = 成功；1 = 句柄为空；2 = 通道表已满（CFG_UART_RX_CHANNELS）。
 * ------------------------------------------------------------------------ */
u8 Bsp_Uart_OpenChannel(void *huart, Bsp_Uart_RxByteHandler handler);

/* 主机串口当前是否处于"已武装"状态（诊断/断点观测用，1=正常） */
u8 Bsp_Uart_IsArmed(void);

/* 接收通道保活：主循环每圈调一次（开销 = 一个标志判断）。
 * 错误回调里若因为 HAL 处于 BUSY 而没武装上，这里会补上；
 * 不加这一条的话，连续两次线路错误就可能让串口永久失聪。 */
void Bsp_Uart_Service(void);

/* --- 发送 ---------------------------------------------------------------
 * 上层（Service 的 log.c 里 __io_putchar / 以后协议回包）只调这两个函数，
 * 不要去直接抓 HAL 句柄 —— Service 层不允许 include usart.h。
 *
 * ⚠️ 阻塞发送，但**有超时**（CFG_UART_TX_TIMEOUT_MS）：
 *    超时就放弃这几个字节并计数，绝不把主循环卡死在这里。
 *    丢字节只影响日志观感；卡住主循环会让掉线保护失效、车一直转。
 * ---------------------------------------------------------------------- */
void Bsp_Uart_SendByte(u8 byte);
void Bsp_Uart_SendBuf(const u8 *buf, u16 len);

/* 发送超时次数（诊断用：非 0 说明链路/电平转换有问题，日志被丢过） */
u32 Bsp_Uart_GetTxDropCount(void);

/* ---------------------------------------------------------------------------
 * 接收诊断计数（"发出去没反应"时用来一刀切断故障范围）
 *
 *   RX_TOTAL == 0 而你一直在发  → 字节根本没到 MCU：接线、透传模式、电平
 *   RX_TOTAL >  0 但仍无反应     → 字节到了，问题在帧状态机/命令解析
 *   RX_ERR  持续增长             → 线路质量差（ORE/FE/NE），先查线
 *
 * 用法：调试器里 `>-interpreter-exec console "print Bsp_Uart_GetRxTotal()"`
 *       或在断点命中时看 watch 窗口。两个函数都是纯读取，无副作用。
 * ------------------------------------------------------------------------ */
u32 Bsp_Uart_GetRxTotal(void);
u32 Bsp_Uart_GetRxErrorCount(void);

#endif /* BSP_UART_H */
