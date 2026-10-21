#include "bsp_uart.h"
#include "cfg_comm.h"
#include "mx_board.h"

/* ============================================================================
 * 串口底层：HAL 逐字节中断接收 + "把字节转交给上层注册的回调"
 *
 *    旧代码在 USART1_IRQHandler 里直接读 DR（SPL 时代）；
 *    现在用 HAL_UART_Receive_IT() 每次收 1 字节，收完调 HAL_UART_RxCpltCallback。
 *
 *    旧代码把"帧状态机"也写在这个回调里 → 会导致 Bsp 依赖 Service（方向反了）。
 *    现在回调只做一件事：把字节丢给注册进来的处理函数；没人注册就丢掉。
 *
 * 三条"保命"约定（都是被现场故障逼出来的）：
 *   ① 发送必须带超时 —— 无超时的阻塞发送会把主循环卡死，
 *      主循环一停，Motion_Tick() 的掉线保护就失效，车会一直转；
 *   ② 接收必须能自愈 —— 错误回调里若武装失败，主循环用 Bsp_Uart_Service()
 *      兜底补上，否则连续两次线路错误就让串口永久失聪；
 *   ③ 接收必须按 Instance 分发 —— 回调是全工程唯一的，多路串口共存时
 *      不能"哪路来的都当上位机命令"，否则相机字节会把车开跑。
 * ============================================================================ */

/* 一个接收通道 = 一路串口。
 * rx_byte 必须每路各给一个：HAL 把收到的字节写在这个地址上，
 * 两路共用一个变量会互相踩。 */
typedef struct
{
    UART_HandleTypeDef    *huart;      /* 这一路的 HAL 句柄                 */
    u8                     rx_byte;    /* HAL 逐字节接收暂存（每路独立）     */
    Bsp_Uart_RxByteHandler handler;    /* 上层注册的字节处理器（0 = 丢弃）   */
    volatile u8            armed;      /* 1 = 已武装；0 = 等主循环补一次     */
} Bsp_Uart_Channel;

static Bsp_Uart_Channel s_ch[CFG_UART_RX_CHANNELS];
static u8               s_ch_count = 0;

/* 发送超时丢字节计数（诊断用） */
static volatile u32 s_tx_drop = 0;

/* 接收诊断计数（诊断用，用调试器 print 直接读，见 README §11.8）
 *
 * 用途：串口"发出去没反应"时，这两个数能把故障切成两半 ——
 *   s_rx_total == 0  而且你一直在发   → 字节根本没到 MCU（接线/透传/电平）
 *   s_rx_total >  0  但仍然没反应     → 字节到了，问题在帧状态机或命令解析
 *   s_rx_err   持续增长               → 线路质量差（ORE/FE/NE），先查线再查固件
 */
static volatile u32 s_rx_total = 0;
static volatile u32 s_rx_err   = 0;

/* ---------------------------------------------------------------------------
 * 通道表：查找 / 新建
 * ------------------------------------------------------------------------ */
static Bsp_Uart_Channel *Bsp_Uart_Find(UART_HandleTypeDef *huart)
{
    u8 i;

    for (i = 0; i < s_ch_count; i++)
    {
        if (s_ch[i].huart == huart)
        {
            return &s_ch[i];
        }
    }
    return 0;
}

static Bsp_Uart_Channel *Bsp_Uart_New(UART_HandleTypeDef *huart)
{
    Bsp_Uart_Channel *c = Bsp_Uart_Find(huart);

    if (c != 0)
    {
        return c;
    }
    if (s_ch_count >= CFG_UART_RX_CHANNELS)
    {
        return 0;                          /* 表满：返回值由上层决定怎么处理 */
    }

    c          = &s_ch[s_ch_count];
    c->huart   = huart;
    c->rx_byte = 0;
    c->handler = 0;
    c->armed   = 0;
    s_ch_count++;

    return c;
}

/* ---------------------------------------------------------------------------
 * 重新武装某一路的逐字节接收
 *   HAL 在 ORE/FE 之后可能仍停在 BUSY 状态，此时直接 Receive_IT 会返回
 *   HAL_BUSY 并"静默失败"——调用者看不到任何报错，串口从此收不到数据。
 *   所以这里的返回值必须落地成 armed，由主循环兜底重试。
 * ------------------------------------------------------------------------ */
static void Bsp_Uart_Rearm(Bsp_Uart_Channel *c)
{
    if (HAL_UART_Receive_IT(c->huart, &c->rx_byte, 1) == HAL_OK)
    {
        c->armed = 1;
    }
    else
    {
        c->armed = 0;                      /* 交给 Bsp_Uart_Service() 下一圈再试 */
    }
}

/* ---------------------------------------------------------------------------
 * 公开接口
 * ------------------------------------------------------------------------ */
u8 Bsp_Uart_OpenChannel(void *huart, Bsp_Uart_RxByteHandler handler)
{
    Bsp_Uart_Channel *c;

    if (huart == 0)
    {
        return 1;
    }

    c = Bsp_Uart_New((UART_HandleTypeDef *)huart);
    if (c == 0)
    {
        return 2;
    }

    c->handler = handler;
    Bsp_Uart_Rearm(c);

    return 0;
}

void Bsp_Uart_SetRxByteHandler(Bsp_Uart_RxByteHandler handler)
{
    Bsp_Uart_Channel *c = Bsp_Uart_New(BOARD_HUART_HOST);

    if (c != 0)
    {
        c->handler = handler;
    }
}

void Bsp_Uart_RxStart(void)
{
    u8 i;

    (void)Bsp_Uart_New(BOARD_HUART_HOST);  /* 保证主机通道一定在表里 */

    for (i = 0; i < s_ch_count; i++)
    {
        Bsp_Uart_Rearm(&s_ch[i]);
    }
}

void Bsp_Uart_Service(void)
{
    u8 i;

    for (i = 0; i < s_ch_count; i++)
    {
        if (s_ch[i].armed == 0)
        {
            Bsp_Uart_Rearm(&s_ch[i]);
        }
    }
}

u8 Bsp_Uart_IsArmed(void)
{
    Bsp_Uart_Channel *c = Bsp_Uart_Find(BOARD_HUART_HOST);

    return (u8)((c != 0) ? c->armed : 0);
}

/* ---------------------------------------------------------------------------
 * 发送：阻塞等 TC，但最多等 CFG_UART_TX_TIMEOUT_MS
 *   超时 = 放弃这几个字节并计数。日志丢几个字没人看，
 *   主循环卡死（掉线保护失效、车一直转）是不能接受的。
 * ------------------------------------------------------------------------ */
void Bsp_Uart_SendByte(u8 byte)
{
    if (HAL_UART_Transmit(BOARD_HUART_HOST, &byte, 1, CFG_UART_TX_TIMEOUT_MS) != HAL_OK)
    {
        s_tx_drop++;
    }
}

void Bsp_Uart_SendBuf(const u8 *buf, u16 len)
{
    if ((buf == 0) || (len == 0))
    {
        return;
    }
    if (HAL_UART_Transmit(BOARD_HUART_HOST, (uint8_t *)buf, len, CFG_UART_TX_TIMEOUT_MS) != HAL_OK)
    {
        s_tx_drop++;
    }
}

u32 Bsp_Uart_GetTxDropCount(void)
{
    return s_tx_drop;
}

u32 Bsp_Uart_GetRxTotal(void)
{
    return s_rx_total;
}

u32 Bsp_Uart_GetRxErrorCount(void)
{
    return s_rx_err;
}

/* ---------------------------------------------------------------------------
 * HAL 回调（弱符号，全工程只能有这一处强定义）
 *   一次收完 1 字节 → 按 Instance 找到对应通道 → 交给它的回调 → 重新武装
 * ------------------------------------------------------------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Bsp_Uart_Channel *c = Bsp_Uart_Find(huart);

    if (c == 0)
    {
        return;                            /* 没注册过的串口：直接忽略 */
    }

    s_rx_total++;                          /* 诊断：收到的字节总数 */

    if (c->handler != 0)
    {
        c->handler(c->rx_byte);
    }

    Bsp_Uart_Rearm(c);
}

/* ---------------------------------------------------------------------------
 * 串口错误回调 —— 没有这个函数，串口会"静默失聪"
 *
 * 现象：上位机某一帧发太快/线路上有干扰 → 触发 ORE（溢出）或 FE/NE 错误
 *       → HAL 在错误里把接收**中止了**，而我们的 RxCpltCallback 再也不会被调用
 *       → 从此这一路串口一个字节都收不到，直到复位。
 *       用户看到的就是"调着调着突然发什么指令都没反应"。
 *
 * 处理：清错误标志 + 清 ErrorCode + 中止残留的接收状态 + 重新武装。
 *       先 AbortReceive 再武装，是因为错误路径下 HAL 可能仍停在 BUSY：
 *       不先清状态直接 Receive_IT 会返回 HAL_BUSY，等于"恢复了但没完全恢复"。
 * ------------------------------------------------------------------------ */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Bsp_Uart_Channel *c = Bsp_Uart_Find(huart);

    s_rx_err++;                            /* 诊断：线路错误次数 */

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (c == 0)
    {
        return;
    }

    c->armed = 0;                          /* 先标记未武装，下面若失败交给主循环 */
    (void)HAL_UART_AbortReceive(huart);    /* 把 BUSY/半截接收状态清干净 */
    Bsp_Uart_Rearm(c);
}
