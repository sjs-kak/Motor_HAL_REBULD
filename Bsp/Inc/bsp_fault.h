#ifndef BSP_FAULT_H
#define BSP_FAULT_H

#include "types.h"

/* ============================================================================
 * 故障现场输出（X6）
 *
 * 原来 Error_Handler() 和 HardFault_Handler() 都是裸的 while(1)：
 * 板子一旦进去，现场（谁崩的、崩在哪、哪个寄存器出错）全部丢失，
 * 只能靠"插调试器再跑一遍"重现 —— 而很多故障是偶发的，重现不了就没了。
 *
 * 这里做两件事：
 *   1. Error_Handler：把**调用者地址**打出来（HAL 里哪个函数返回了错误）；
 *   2. HardFault：把压栈的 8 个寄存器 + CFSR/HFSR/MMFAR/BFAR 打出来。
 *
 * 实现上刻意**不走 HAL、不用 HAL_GetTick()**：
 *   故障可能发生在 SysTick 已死、中断已关、HAL 句柄半初始化的时刻。
 *   HAL_UART_Transmit() 的超时判断依赖 uwTick，而 uwTick 靠 SysTick 中断递增，
 *   在 HardFault（优先级 -1）里永远不会再涨 —— 一旦 TC 不来就是死等。
 *   所以这里直接轮询 USART 的 TXE 位，并且每次写都有自旋上限，
 *   串口坏掉最多丢掉这几行字，绝不会把现场二次搞坏。
 *
 * 上层用法：
 *   main.c 的 Error_Handler()        → Bsp_Fault_DumpError((u32)__builtin_return_address(0));
 *   stm32f4xx_it.c 的 HardFault_Handler() → Bsp_Fault_DumpHardFault(frame);
 * ============================================================================ */

/* 打印 "谁调用了 Error_Handler"（caller_pc = 返回地址） */
void Bsp_Fault_DumpError(u32 caller_pc);

/* 打印 HardFault 现场。frame 指向异常压栈的 8 个字：
 *   [0]=R0 [1]=R1 [2]=R2 [3]=R3 [4]=R12 [5]=LR [6]=PC [7]=xPSR
 * 函数不会返回（末尾死循环等调试器）。 */
void Bsp_Fault_DumpHardFault(const u32 *frame) __attribute__((noreturn));

#endif /* BSP_FAULT_H */
