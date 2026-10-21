#include "bsp_rc.h"
#include "mx_board.h"
#include "cfg_rc.h"
#include "osal.h"

/* ============================================================================
 * 遥控接收机（PPM 单线）输入捕获
 *
 * ---------------------------------------------------------------------------
 * 为什么这个定时器不在 CubeMX 的 .ioc 里（故意如此）
 *
 *   TIM1/TIM2 走 .ioc 生成，是因为它们要给电机发脉冲，配置多、要改时用图形化
 *   更稳。但本工程是多人并行开发：另一个人在改运动控制程序（app_init.c /
 *   motion.c / main.c 都动）。此时**只要有人再点一次 CubeMX 的 GENERATE，
 *   他手写的 USER CODE 之外的部分就有被覆盖的风险**。
 *
 *   所以 TIM3 在这里手工配置：实例/引脚/AF/中断号全部来自 mx_board.h 的
 *   BOARD_RC_* 宏，一行都不碰 Core/ 下的生成文件，两边物理上不冲突。
 *   代价是多 30 行配置代码；等运动控制稳定后再搬进 .ioc，删掉下面这段即可。
 *
 * ---------------------------------------------------------------------------
 * 解码原理（CPPM / PPM，一根线带 6 个通道）
 *
 *   线上是"每个通道一个脉冲、脉冲间隔里塞信息"。本实现**只抓上升沿**，
 *   用相邻两个上升沿之间的时间（下称"槽宽"）当作通道值：
 *
 *     ┌──┐        ┌┐    ┌────┐        ┌┐
 *     │  │        ││    │    │        ││
 *   ──┘  └────────┘└────┘    └────────┘└──
 *     |<--CH1槽-->|<-CH2槽->|  ...  |<--同步头-->|
 *       ~1500µs     ~1000µs            >2500µs
 *
 *   槽宽 > CFG_RC_SYNC_US(2500) 的那个沿就是帧同步头，它后面才是 CH1。
 *   这样即使从上电的半帧中间接管，也最多错一帧就能自动对齐，不需要任何
 *   "先等静默 500ms"之类的初始化技巧。
 *
 *   为什么不用 HAL 的 PWM 输入模式(PWMInput)或从机复位模式？
 *     本工程的主循环里全是阻塞动作（Motion_Drive 等），从机复位模式虽然能
 *     省中断，但一旦漏掉一个沿就整帧错乱且无自愈；而 45Hz × 6 = 270 次/秒
 *     中断对 168MHz 来说是噪声级别的负载。简单可靠优先。
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * 状态（全部 volatile：中断里写，主循环里读）
 * ------------------------------------------------------------------------ */

/* 我们自己建的句柄 —— 不是 MX_TIM3_Init() 生成的那个（那个不存在） */
static TIM_HandleTypeDef s_htim_rc;

static volatile u16 s_prev;       /* 上一个上升沿的计数值（1 µs 计数，16 位自动回绕）*/
static volatile u8  s_started;    /* 已收到第一个沿（第一个差值是垃圾，丢掉）*/
static volatile u8  s_idx;        /* 当前帧已收到几个通道沿 */

static volatile u16 s_buf[CFG_RC_CH_NUM];   /* 正在拼的半帧 */
static volatile u16 s_pub[CFG_RC_CH_NUM];   /* 拼完的整帧（发布区）*/

/* 发布区读写保护：奇数表示 ISR 正在写。主循环重试几次即可，不用关中断。*/
static volatile u8  s_pub_seq;
static volatile u8  s_pub_ok;

static volatile u32 s_edge_ms;    /* 最近一次捕获沿的时刻 */
static volatile u32 s_edge_cnt;
static volatile u32 s_frame_cnt;

static u8 s_ready;

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------ */
void Bsp_Rc_Init(void)
{
    GPIO_InitTypeDef   gpio;
    TIM_IC_InitTypeDef ic;

    s_ready    = 0u;
    s_started  = 0u;
    s_idx      = 0u;
    s_pub_seq  = 0u;
    s_pub_ok   = 0u;
    s_prev     = 0u;
    s_edge_cnt = 0u;
    s_frame_cnt = 0u;
    s_edge_ms  = Osal_GetTickMs();

    /* 1) 时钟：GPIO 与 TIM3（都在 APB1 域） */
    BOARD_RC_GPIO_CLK_ENABLE();
    BOARD_RC_TIM_CLK_ENABLE();

    /* 2) 引脚 → 复用功能
     *    Pull = PULLDOWN 的原因：接收机没插时线悬空，上拉会把空闲电平拉高、
     *    在上升沿捕获下持续误触发；下拉让"没接线"表现为"一个沿都没有"，
     *    从 Bsp_Rc_GetEdgeCount()==0 就能一眼看出来。 */
    gpio.Pin       = BOARD_RC_GPIO_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = BOARD_RC_GPIO_PULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = BOARD_RC_GPIO_AF;
    HAL_GPIO_Init(BOARD_RC_GPIO_PORT, &gpio);

    /* 3) 时基：1 µs 计数，ARR 满 16 位（65.5 ms，一帧 22.5 ms 装得下）*/
    s_htim_rc.Instance               = BOARD_RC_TIM_INSTANCE;
    s_htim_rc.Init.Prescaler         = (u32)CFG_RC_TIM_PSC;
    s_htim_rc.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_rc.Init.Period            = 0xFFFFu;
    s_htim_rc.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_rc.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_IC_Init(&s_htim_rc) != HAL_OK)
    {
        return;                     /* 失败就保持"无信号"，上层会打印 RC nosig */
    }

    /* 4) 通道 1 → 输入捕获，直接映射到 TI1 */
    ic.ICPolarity  = BOARD_RC_IC_POLARITY;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = (u32)CFG_RC_IC_FILTER;

    if (HAL_TIM_IC_ConfigChannel(&s_htim_rc, &ic, BOARD_RC_TIM_CHANNEL) != HAL_OK)
    {
        return;
    }

    /* 5) 先开 NVIC 再启动捕获：反过来的话第一个沿可能在中断使能前就丢了 */
    HAL_NVIC_SetPriority(BOARD_RC_TIM_IRQn, CFG_RC_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(BOARD_RC_TIM_IRQn);

    if (HAL_TIM_IC_Start_IT(&s_htim_rc, BOARD_RC_TIM_CHANNEL) != HAL_OK)
    {
        return;
    }

    s_ready = 1u;
}

/* ---------------------------------------------------------------------------
 * 中断入口（startup_stm32f407xx.s 里是弱定义，这里给出强定义接管）
 *
 * ⚠️ TIM3_IRQHandler 在 .ioc 里没被启用，所以 CubeMX 不会生成它；
 *    如果将来把 TIM3 搬进 .ioc，**必须删掉这里的强定义**，
 *    否则链接期报 "multiple definition"。
 * ------------------------------------------------------------------------ */
void BOARD_RC_IRQ_HANDLER(void)
{
    HAL_TIM_IRQHandler(&s_htim_rc);
}

/* ---------------------------------------------------------------------------
 * 捕获回调：全工程唯一一个 —— 所以必须先确认是自己的定时器
 *
 * 这个函数是 HAL 的 __weak 默认实现被我们覆盖。以后若有第二个定时器也要
 * 做输入捕获，它会进到这里来，靠下面这个 Instance 判断分流（README §11 X2）。
 *
 * 中断里只做减法 + 抄数组，不打印、不调 Motion_*、不做任何除法。
 * 唯一的函数调用是 Osal_GetTickMs()（就是读一个 volatile 变量）。
 * ------------------------------------------------------------------------ */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    u16 now;
    u16 d;
    u8  i;

    if (htim->Instance != BOARD_RC_TIM_INSTANCE)
    {
        return;                     /* 不是遥控那一路，不理 */
    }

    /* 直接读 CCR1，比 HAL_TIM_ReadCapturedValue() 少一层通道判断 */
    now = (u16)htim->Instance->CCR1;

    s_edge_cnt++;
    s_edge_ms = Osal_GetTickMs();

    if (s_started == 0u)
    {
        /* 第一个沿没有"上一个沿"可比，它的差值一定是垃圾，丢掉 */
        s_started = 1u;
        s_prev    = now;
        s_idx     = 0u;
        return;
    }

    /* 16 位自然回绕：65.5 ms 内差值就是真实毫秒微秒数，不会歧义 */
    d     = (u16)(now - s_prev);
    s_prev = now;

    if (d >= (u16)CFG_RC_SYNC_US)
    {
        /* 帧同步头：丢掉半帧，从下一个沿重新数 CH1 */
        s_idx = 0u;
        return;
    }

    if (d < (u16)CFG_RC_MIN_PULSE_US)
    {
        /* 毛刺（小于最小槽宽），不计入通道，也不算同步头 */
        return;
    }

    if (s_idx < (u8)CFG_RC_CH_NUM)
    {
        s_buf[s_idx] = d;
        s_idx++;
    }

    if (s_idx >= (u8)CFG_RC_CH_NUM)
    {
        /* 整帧凑齐 → 发布。seq 奇数表示"正在写"，读侧看到奇数就重试。 */
        s_pub_seq++;
        for (i = 0u; i < (u8)CFG_RC_CH_NUM; i++)
        {
            s_pub[i] = s_buf[i];
        }
        s_pub_ok = 1u;
        s_frame_cnt++;
        s_pub_seq++;
        s_idx = 0u;
    }
}

/* ---------------------------------------------------------------------------
 * 对外查询
 * ------------------------------------------------------------------------ */
u32 Bsp_Rc_GetAgeMs(void)
{
    return Osal_GetTickMs() - s_edge_ms;
}

u32 Bsp_Rc_GetEdgeCount(void)
{
    return s_edge_cnt;
}

u32 Bsp_Rc_GetFrameCount(void)
{
    return s_frame_cnt;
}

u8 Bsp_Rc_IsReady(void)
{
    return s_ready;
}

u8 Bsp_Rc_HasSignal(void)
{
    if (s_ready == 0u)
    {
        return 0u;
    }
    if (s_pub_ok == 0u)
    {
        return 0u;                  /* 还没解出过一帧完整的 */
    }
    if (Bsp_Rc_GetAgeMs() > (u32)CFG_RC_LOST_MS)
    {
        return 0u;                  /* 有帧但已经不新鲜了：线掉了/发射机关了 */
    }
    return 1u;
}

u8 Bsp_Rc_GetChannels(u16 *us, u8 n)
{
    u8  i;
    u8  attempt;
    u8  seq1;
    u8  seq2;
    u8  ok = 0u;

    if ((us == 0) || (n == 0u))
    {
        return 0u;
    }
    if (Bsp_Rc_HasSignal() == 0u)
    {
        return 0u;
    }
    if (n > (u8)CFG_RC_CH_NUM)
    {
        n = (u8)CFG_RC_CH_NUM;
    }

    /* 抄 12 字节期间 ISR 可能正好在写。用序号判断，重试最多 3 次；
     * 一帧 22 ms 才写一次、写一次不到 1 µs，撞上基本不会发生。 */
    for (attempt = 0u; attempt < 3u; attempt++)
    {
        seq1 = s_pub_seq;
        if ((seq1 & 1u) != 0u)
        {
            continue;               /* ISR 正在写 */
        }

        for (i = 0u; i < n; i++)
        {
            us[i] = s_pub[i];
        }

        seq2 = s_pub_seq;
        if (seq1 == seq2)
        {
            ok = 1u;
            break;
        }
    }

    return ok;
}
