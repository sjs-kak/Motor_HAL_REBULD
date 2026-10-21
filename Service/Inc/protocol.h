#ifndef SERVICE_PROTOCOL_H
#define SERVICE_PROTOCOL_H

#include "types.h"

/* ============================================================================
 * Service 层 — 帧协议解析（与具体业务无关：去掉"表型/株号/拍照"依然成立）
 *
 * 帧格式：以 0x0D 0x0A ("\r\n") 结尾。
 * 旧代码把这套状态机写在 HAL_UART_RxCpltCallback 里（uart_port.c:45-72），
 * 导致 Bsp 依赖 Service；现在拆开：
 *     Bsp（bsp_uart.c）只把字节丢进来 → Protocol_FeedByte()
 *     Service（这里）负责组帧，由上层用 Protocol_TakeFrame() 取走。
 *
 * ⚠️ 本层不允许 include tim.h/usart.h/gpio.h（串口句柄一律走 Bsp/MxBoard 翻译）。
 * ============================================================================ */

#define PROTOCOL_FRAME_MAX  200     /* 单帧最大字节数（原 uart_port.h 的 USART_REC_LEN） */

/* 喂入 1 字节（在中断上下文调用） */
void Protocol_FeedByte(u8 byte);

/* ---------------------------------------------------------------------------
 * 取一帧（在主循环调用）
 *   out/outSize 上层提供的缓冲区；len 返回帧长（可为 0/NULL）
 *   返回 1 = 取到一帧（out 已补 '\0'），0 = 还没收到完整帧
 * ------------------------------------------------------------------------ */
u8 Protocol_TakeFrame(char *out, u16 outSize, u16 *len);

/* 丢弃当前半帧，重新开始收 */
void Protocol_Reset(void);

#endif /* SERVICE_PROTOCOL_H */
