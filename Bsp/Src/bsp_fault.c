#include "bsp_fault.h"
#include "mx_board.h"

/* ============================================================================
 * 故障现场输出的实现（见 bsp_fault.h 的设计说明）
 *
 * 只依赖两样东西：USART 的 SR/DR 寄存器、SCB 的故障状态寄存器。
 * 不调 HAL、不读 uwTick、不开中断 —— 保证在"什么都坏了"的时刻还能写字。
 * ============================================================================ */

/* 单字节发送的自旋上限（约几十微秒 @168MHz；串口坏了就放弃这一行） */
#define BSP_FAULT_SPIN   200000u

/* 写一个字符：轮询 TXE，带自旋上限。返回 0 表示串口没回应（放弃后续输出） */
static u8 Bsp_Fault_PutChar(char c)
{
    USART_TypeDef *uart = BOARD_HUART_HOST->Instance;
    u32            spin = BSP_FAULT_SPIN;

    while (((uart->SR & USART_SR_TXE) == 0u) && (--spin != 0u))
    {
        /* 空转等发送寄存器空 */
    }
    if (spin == 0u)
    {
        return 0;
    }

    uart->DR = (u16)(u8)c;
    return 1;
}

static void Bsp_Fault_PutStr(const char *s)
{
    if (s == 0)
    {
        return;
    }
    while (*s != '\0')
    {
        if (Bsp_Fault_PutChar(*s) == 0u)
        {
            return;                        /* 串口不通，剩下的字就不写了 */
        }
        s++;
    }
}

static void Bsp_Fault_PutHex32(u32 v)
{
    static const char hex[] = "0123456789ABCDEF";
    int              i;

    Bsp_Fault_PutStr("0x");
    for (i = 28; i >= 0; i -= 4)
    {
        (void)Bsp_Fault_PutChar(hex[(v >> (u32)i) & 0x0Fu]);
    }
}

/* ---------------------------------------------------------------------------
 * Error_Handler：打印调用者地址
 *   caller_pc 是返回地址，用 addr2line 一查就是"哪个 HAL 调用返回了错误"：
 *     arm-none-eabi-addr2line -e build/Debug/Motor_HAL_REBULD.elf 0x08001234
 * ------------------------------------------------------------------------ */
void Bsp_Fault_DumpError(u32 caller_pc)
{
    Bsp_Fault_PutStr("\r\n[FAULT] Error_Handler, caller=");
    Bsp_Fault_PutHex32(caller_pc);
    Bsp_Fault_PutStr("\r\n");
}

/* ---------------------------------------------------------------------------
 * HardFault：打印异常压栈现场 + 故障状态寄存器
 * ------------------------------------------------------------------------ */
void Bsp_Fault_DumpHardFault(const u32 *frame)
{
    Bsp_Fault_PutStr("\r\n[FAULT] HardFault\r\n");

    if (frame != 0)
    {
        Bsp_Fault_PutStr("  R0 =");   Bsp_Fault_PutHex32(frame[0]);
        Bsp_Fault_PutStr("  R1 =");   Bsp_Fault_PutHex32(frame[1]);
        Bsp_Fault_PutStr("  R2 =");   Bsp_Fault_PutHex32(frame[2]);
        Bsp_Fault_PutStr("  R3 =");   Bsp_Fault_PutHex32(frame[3]);
        Bsp_Fault_PutStr("\r\n  R12="); Bsp_Fault_PutHex32(frame[4]);
        Bsp_Fault_PutStr("  LR =");   Bsp_Fault_PutHex32(frame[5]);
        Bsp_Fault_PutStr("  PC =");   Bsp_Fault_PutHex32(frame[6]);
        Bsp_Fault_PutStr("  PSR=");   Bsp_Fault_PutHex32(frame[7]);
        Bsp_Fault_PutStr("\r\n");
    }

    Bsp_Fault_PutStr("  CFSR =");  Bsp_Fault_PutHex32(SCB->CFSR);
    Bsp_Fault_PutStr("  HFSR =");  Bsp_Fault_PutHex32(SCB->HFSR);
    Bsp_Fault_PutStr("\r\n  MMFAR="); Bsp_Fault_PutHex32(SCB->MMFAR);
    Bsp_Fault_PutStr("  BFAR =");  Bsp_Fault_PutHex32(SCB->BFAR);
    Bsp_Fault_PutStr("\r\n");

    /* 有调试器连着就直接断在这里，现场可以立刻看（没有调试器时 BKPT 会
     * 再次升级成 HardFault，所以必须先判断 C_DEBUGEN） */
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0u)
    {
        __BKPT(0);
    }

    while (1)
    {
        /* 停在这里等人工复位 */
    }
}
