#ifndef BSP_UART_H
#define BSP_UART_H

#include "types.h"

/* ============================================================================
 * 串口（USART1 上位机 + USART3 遥控 i-BUS）— 只负责"把收到的字节交给上层"，
 * 不解析协议、不组帧（那是 Service 层 protocol.c / ibus.c 的事）。
 *
 * ⚠️ 绝不能再自己定义 USART1_IRQHandler / USART3_IRQHandler！
 *    Core/Src/stm32f4xx_it.c 里已经各有一个（分别调 HAL_UART_IRQHandler(&huartN)），
 *    重复定义会导致串口整个收不到数据（链接不报错，行为直接错）。
 * ============================================================================ */

/* 收到 1 字节时回调；由上层注册（App/Src/app_init.c 注册 Protocol_FeedByte） */
typedef void (*Bsp_Uart_RxByteHandler)(u8 byte);

void Bsp_Uart_RxStart(void);                                   /* 启动主机接收 */
void Bsp_Uart_SetRxByteHandler(Bsp_Uart_RxByteHandler handler); /* 注册回调 */

/* ---------------------------------------------------------------------------
 * 遥控串口（USART3 / i-BUS）
 *
 * 语义与主机串口完全一致：只交字节、不解析 —— i-BUS 的组帧在
 * Service/Src/ibus.c 里（App 注册 Ibus_FeedByte）。
 *
 * 为什么不做成 Bsp_Uart_OpenChannel(&huart3, ...) 就完事：
 *   遥控链路是"丢了就失控"的那一路。它需要能在调试器里一眼看出
 *   "接收武装上没有、线路错误多少次"，所以给它一组显式入口和
 *   独立的错误计数，不与主机通道的计数混在一起。
 * ------------------------------------------------------------------------ */
void Bsp_Uart_RcRxStart(void);
void Bsp_Uart_RcSetRxByteHandler(Bsp_Uart_RxByteHandler handler);
u32  Bsp_Uart_RcGetErrorCount(void);

/* 遥控串口是否已武装（1 = 正常）。掉遥控时先看这个：
 *   0          → 中断接收没挂上，问题在本层（大概率 ORE 后没恢复过来）
 *   1 但仍无帧 → 字节到了，问题在 ibus.c 的组帧/校验或接线
 * ------------------------------------------------------------------------ */
u8   Bsp_Uart_RcIsArmed(void);

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

/* ---------------------------------------------------------------------------
 * 遥控接收机（USART3，i-BUS）：第二条独立的接收通道
 *
 * 与主机通道**完全隔离**：各自的武装状态、各自的错误计数、各自的回调。
 * 隔离是必须的——主机链路抖一下不该影响遥控，遥控断了更不能悄悄
 * 记到主机计数里（那会让现场排查看错方向）。
 *
 * 接线：FS-iA6B 的 i-BUS 信号 → PB11（USART3_RX），115200 8N1。
 * ------------------------------------------------------------------------ */
void Bsp_Uart_RcSetRxByteHandler(Bsp_Uart_RxByteHandler handler);
void Bsp_Uart_RcRxStart(void);          /* 清 ORE 后武装接收，App_Init 里调一次 */

/* 遥控线路错误次数（ORE/FE/NE）。非 0 且持续增长 → 先查线与共地，再怀疑接收机 */
u32 Bsp_Uart_RcGetErrorCount(void);

/* 遥控通道是否已武装（1=中断已在等字节）。
 *
 * 判读口诀（现场最有用的一条）：
 *   ARMED == 0           → 中断根本没挂上，问题在**本层**（bsp_uart.c 的 Rearm）
 *   ARMED == 1 但没数据  → 中断在等，是**组帧**或**接线**的问题（ibus.c / PB11）
 * ------------------------------------------------------------------------ */
u8 Bsp_Uart_RcIsArmed(void);

#endif /* BSP_UART_H */
