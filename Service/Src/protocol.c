#include "protocol.h"

/* ============================================================================
 * 帧状态机（与旧工程 USART1_IRQHandler 里的完全一致）
 *
 *    bit15    : 一帧接收完成
 *    bit14    : 已经收到过 0x0D
 *    bit13~0  : 已收字节数
 *
 * 旧代码用的是全局变量 USART_RX_BUF / USART_RX_STA（uart_port.c:35-36），
 * 任何地方都能改；现在收进本文件 static，只能用这两个 API 访问。
 * ============================================================================ */

static u8  s_rx_buf[PROTOCOL_FRAME_MAX];
static u16 s_rx_sta = 0;

/* 主循环正在把 s_rx_buf 拷到调用者缓冲区时置 1。
 * 期间中断里收到的字节直接丢掉 —— 宁可丢一个字节（这一帧作废），
 * 也不能让中断在拷到一半的时候把缓冲区改掉（那样取出来的命令是坏的）。 */
static volatile u8 s_reading = 0;

void Protocol_FeedByte(u8 byte)
{
    if (s_reading != 0)                     /* 主循环正在取帧：本字节丢弃 */
    {
        return;
    }

    /* 上一帧还没被取走：**丢弃旧帧，让新数据进来**。
     *
     * 旧实现是"丢掉所有新字节"，一旦上层因为跑动作卡住、没及时 TakeFrame，
     * 串口就会永久失聪——上位机看到的正是"发什么都没反应"。
     * 现在改成"新的覆盖旧的"：宁可丢一帧旧命令，也不能停止收数据。
     * 这些语句都在中断上下文，读写 s_rx_sta 是单字操作，不会被撕裂。 */
    if ((s_rx_sta & 0x8000) != 0)
    {
        s_rx_sta = 0;
    }

    if ((s_rx_sta & 0x4000) != 0)           /* 已经收到 0x0D，等 0x0A */
    {
        if (byte != 0x0A)
        {
            s_rx_sta = 0;                    /* 不是 0x0A，接收错误，重新开始 */
        }
        else
        {
            s_rx_sta |= 0x8000;              /* 0x0D 0x0A 齐了，一帧完成 */
        }
    }
    else                                     /* 还没收到 0x0D */
    {
        if (byte == 0x0D)
        {
            s_rx_sta |= 0x4000;
        }
        else
        {
            s_rx_buf[s_rx_sta & 0x3FFF] = byte;
            s_rx_sta++;
            if (s_rx_sta > (PROTOCOL_FRAME_MAX - 1))
            {
                s_rx_sta = 0;                /* 溢出保护：重新开始 */
            }
        }
    }
}

u8 Protocol_TakeFrame(char *out, u16 outSize, u16 *len)
{
    u16 n;
    u16 i;

    if ((s_rx_sta & 0x8000) == 0)            /* 没有完整帧 */
    {
        return 0;
    }

    n = (u16)(s_rx_sta & 0x3FFF);            /* 帧长（不含结束符） */

    if (out == 0 || outSize == 0)
    {
        s_rx_sta = 0;
        return 0;
    }

    if (n > (u16)(outSize - 1))              /* 缓冲区不够：截断 */
    {
        n = (u16)(outSize - 1);
    }

    s_reading = 1;                           /* 拷贝期间不许中断动缓冲区 */
    for (i = 0; i < n; i++)
    {
        out[i] = (char)s_rx_buf[i];
    }
    out[n] = '\0';

    if (len != 0)
    {
        *len = n;
    }

    /* ⚠️ 顺序：**先清状态，再放开中断**。
     *
     * 反过来的话，在 `s_reading = 0` 和 `s_rx_sta = 0` 之间插进来的那个字节
     * 会被中断写进缓冲区（此时 s_rx_sta 仍是 0x8000|n，中断会先把它清零再存），
     * 紧接着又被这一行的清零抹掉 —— 表现就是"偶尔一条命令少了第一个字符"
     * （F1000 变成 1000，回 ERR unknown）。 */
    s_rx_sta = 0;                            /* 清标志，准备收下一帧 */
    s_reading = 0;

    return 1;
}

void Protocol_Reset(void)
{
    s_rx_sta = 0;
}
