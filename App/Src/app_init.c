#include "app.h"
#include "motion.h"
#include "protocol.h"
#include "log.h"
#include "bsp_uart.h"
#include "cfg_motion.h"
#include "app_rc.h"
#include "cfg_debug.h"   /* 调试开关 CFG_DEBUG_AUTOTEST（默认 0，正常固件不生成代码） */

/* ============================================================================
 * App 层 — 集成点（原 Core/Src/main.c 的 USER CODE 段）
 *
 * ⚠️ 这是全工程的**唯一集成点**：Bsp 的字节回调在这里接到 Service 的协议
 *    解析器上；收到什么命令、调哪个动作，也只在这里决定。
 *    多人协作时，这个文件只能一个人拥有。
 *
 * 本文件不允许 include 任何 MX 外设头文件（tim / usart / gpio 那三个），
 * 也不允许直接调 HAL 的延时函数 —— 一律走 Bsp / Service / Osal 的接口。
 *
 * ---------------------------------------------------------------------------
 * 命令集（帧必须以 0x0D 0x0A 结尾；串口 115200 8N1）
 *
 *   E 或 e 或 ESC   急停：立即停脉冲（不等 S 曲线）
 *                   ⚠️ 单字节就行，不需要回车。这是唯一"不用回车"的命令。
 *                   生效路径有两条：正在跑阻塞动作时由动作内部的等待循环
 *                   每 8ms 检查一次；主循环空闲时由 App_Loop() 第一件事兜底。
 *                   两条合起来，最迟一个主循环周期（<1ms）就断脉冲。
 *   S                平滑停：走 S 曲线降到 0
 *   ?
 *   H 或 h           打印状态 / 帮助
 *   R                打印遥控接收机 6 个通道的 µs 值（阶段 A：只读，不驱动）
 *   F<n>             直行：两侧 <n> Hz，正=前进 负=后退 0=停     例 F2500 / F-1200
 *   L<a>,<b>         差速：左 <a> Hz、右 <b> Hz                  例 L1000,2500
 *   A<n>             改加速度 <n> Hz/s                           例 A1000
 *   D                跑原来的六段演示（13 秒，中途可按 E 打断）
 *
 *   例：想让车以 1500Hz 前进 → 发 "F1500" + 回车
 *
 * ⚠️ 掉线保护：收到 F/L 之后若 CFG_CMD_TIMEOUT_MS(默认 3 秒) 内没有新的
 *    运动命令，Motion_Tick() 会自动停车。也就是说**要持续走就必须持续发**，
 *    这正好是"拔了串口线车不会一直跑"的保证。
 * ============================================================================ */

/* 急停快通道的现场记录：中断里置位，主循环里打印 */
static volatile u8 s_estop_hit = 0;

/* ---------------------------------------------------------------------------
 * 调试自动动作（CFG_DEBUG_AUTOTEST，见 Config/Inc/cfg_debug.h）
 *
 * 存在的唯一理由：STM32Cube 的 GDB 适配器要求求值必须带"选中的栈帧"，
 * 拿不到 frameId 时 `call` 一律失败。这条路不需要调试器参与 ——
 * 改编号、重编译、F5，上电就等于收到一条命令。
 *
 * 默认 0（关闭）时整段代码不参与编译，正常固件零开销。
 * ------------------------------------------------------------------------ */
#if (CFG_DEBUG_AUTOTEST != 0)
static void App_DebugAutoTest(void)
{
    LOG("AUTOTEST %d start\r\n", (int)CFG_DEBUG_AUTOTEST);

#if   (CFG_DEBUG_AUTOTEST == 1)
    Motion_Drive(1200, 0);              /* 加速到顶并保持：**不会自己停** */
#elif (CFG_DEBUG_AUTOTEST == 2)
    Motion_Drive(1200, 3000);           /* 加速 + 保持 3 s */
    Motion_Brake();                     /* 然后平滑停到 0 */
#elif (CFG_DEBUG_AUTOTEST == 3)
    Motion_Steer(600, -600, 3000);      /* 原地左转 3 s（差速 200%） */
    Motion_Brake();
#elif (CFG_DEBUG_AUTOTEST == 4)
    Motion_Demo();                      /* 六段演示（末段自带停车） */
#elif (CFG_DEBUG_AUTOTEST == 5)
    Motion_Drive(2500, 0);              /* 限幅验证：应被夹到 1200，**不会自己停** */
#elif (CFG_DEBUG_AUTOTEST == 6)
    Motion_Drive(1200, 1000);           /* 正转 1 s */
    Motion_Drive(-1200, 2000);          /* 立刻反转 → 看换向时序 */
    Motion_Brake();                     /* 最后平滑停 */
#elif (CFG_DEBUG_AUTOTEST == 7)
    /* 看门狗验证：只武装看门狗 + 给目标速度，**自己不停车**，
     * 停车完全交给主循环里的 Motion_Tick()。3 秒后应当自己停；
     * 如果不退，说明掉线保护失效（README §11.2 的核心验收项）。 */
    Motion_NotifyAlive();
    Motion_Drive(1200, 0);
#else
#error "CFG_DEBUG_AUTOTEST 用了未定义的编号，请查 Config/Inc/cfg_debug.h"
#endif

    LOG("AUTOTEST %d done\r\n", (int)CFG_DEBUG_AUTOTEST);
}
#endif

/* ---------------------------------------------------------------------------
 * 字节级入口：注册给 Bsp_Uart_SetRxByteHandler()，在串口中断里执行
 *
 * 为什么急停要在这一层拦？
 *   协议解析要等 0x0D 0x0A 凑齐一帧才交给主循环；而运动动作是阻塞的，
 *   主循环那 1~5 秒里根本转不动。急停必须比协议更快，所以在这里直接
 *   置一个 volatile 标志——motion.c 的等待循环每 8ms 看一次，立刻就停。
 * ------------------------------------------------------------------------ */
static void App_RxByte(u8 byte)
{
    if ((byte == 'E') || (byte == 'e') || (byte == 0x1B))
    {
        Motion_RequestAbort();
        Protocol_Reset();          /* 丢掉半截帧，免得 ESC 污染下一条命令 */
        s_estop_hit = 1;
        return;
    }

    Protocol_FeedByte(byte);
}

/* ---------------------------------------------------------------------------
 * 极简整数解析：跳过空格/逗号，可选正负号，遇非数字停
 *   返回下一个待解析位置；没解析到数字返回 0。
 * ------------------------------------------------------------------------ */
static const char *App_SkipSep(const char *p)
{
    while ((*p == ' ') || (*p == '\t') || (*p == ','))
    {
        p++;
    }
    return p;
}

static const char *App_ParseS32(const char *p, s32 *out)
{
    s32 sign = 1;
    s32 val  = 0;
    u8  any  = 0;

    p = App_SkipSep(p);
    if (*p == '-')
    {
        sign = -1;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    while ((*p >= '0') && (*p <= '9'))
    {
        if (val < 1000000)
        {
            val = (val * 10) + (s32)(*p - '0');
        }
        p++;
        any = 1;
    }

    if (any == 0)
    {
        return 0;
    }

    *out = val * sign;
    return p;
}

/* ---------------------------------------------------------------------------
 * 一条完整命令的分发
 * ------------------------------------------------------------------------ */
static void App_ExecCmd(const char *cmd)
{
    const char *p = App_SkipSep(cmd);
    const char *q;
    s32 a = 0;
    s32 b = 0;

    if (*p == '\0')
    {
        return;                    /* 空帧（例如单独一个回车）：不理它 */
    }

    switch (*p)
    {
        /* ---- 急停（一帧以 E 开头时也会走到这里，和快通道双保险） ---- */
        case 'E':
        case 'e':
            Motion_RequestAbort();
            Motion_Stop();
            LOG("ESTOP\r\n");
            break;

        /* ---- 平滑停 ---- */
        case 'S':
        case 's':
            Motion_ClearAbort();
            Motion_Brake();
            LOG("STOP\r\n");
            break;

        /* ---- 状态 / 帮助 ---- */
        case '?':
            Motion_GetSpeed(&a, &b);
            LOG("SPD L=%d R=%d ACC=%d%s TXDROP=%u RXTOT=%u RXERR=%u\r\n",
                (int)a, (int)b, (int)Motion_GetAccel(),
                (Motion_Aborted() != 0) ? " ABORT" : "",
                (unsigned int)Bsp_Uart_GetTxDropCount(),
                (unsigned int)Bsp_Uart_GetRxTotal(),
                (unsigned int)Bsp_Uart_GetRxErrorCount());
            break;

        case 'H':
        case 'h':
            LOG("CMDS: E=estop S=stop ?=status R=rc\r\n");
            LOG("      F<n> drive, L<a>,<b> diff, A<n> accel, D=demo\r\n");
            break;

        /* ---- 遥控通道只读打印（阶段 A：只采集，不驱动电机） ----
         * 这一条是遥控接线的验收命令：摇杆动，6 个数字跟着动。
         * 输出格式：RC <ch1> <ch2> ... <ch6> age=<ms>
         * 没信号时：RC nosig age=<ms> edges=<捕获沿数> frames=<完整帧数> */
        case 'R':
        case 'r':
            App_Rc_Report();
            break;

        /* ---- 直行 ---- */
        case 'F':
        case 'f':
            q = App_ParseS32(p + 1, &a);
            if (q == 0)
            {
                LOG("ERR F needs a number\r\n");
                break;
            }
            Motion_ClearAbort();
            Motion_NotifyAlive();
            Motion_Drive(a, 0);        /* 0 = 不保持，停在目标速度上等后续命令 */
            if (Motion_Aborted() != 0)
            {
                LOG("ABORTED\r\n");
            }
            else
            {
                LOG("FWD %d\r\n", (int)a);
            }
            break;

        /* ---- 差速 / 转弯 ---- */
        case 'L':
        case 'l':
            q = App_ParseS32(p + 1, &a);
            if (q == 0)
            {
                LOG("ERR L needs two numbers\r\n");
                break;
            }
            q = App_ParseS32(q, &b);
            if (q == 0)
            {
                LOG("ERR L needs two numbers\r\n");
                break;
            }
            Motion_ClearAbort();
            Motion_NotifyAlive();
            Motion_Steer(a, b, 0);
            if (Motion_Aborted() != 0)
            {
                LOG("ABORTED\r\n");
            }
            else
            {
                LOG("DIFF L=%d R=%d\r\n", (int)a, (int)b);
            }
            break;

        /* ---- 改加速度 ---- */
        case 'A':
        case 'a':
            q = App_ParseS32(p + 1, &a);
            if ((q == 0) || (a <= 0))
            {
                LOG("ERR A needs a positive number\r\n");
                break;
            }
            Motion_SetAccel((u32)a);
            LOG("ACC %d\r\n", (int)Motion_GetAccel());
            break;

        /* ---- 六段演示（原来是上电自动跑，现在手动触发） ---- */
        case 'D':
        case 'd':
            Motion_ClearAbort();
            LOG("DEMO start (send E to abort)\r\n");
            Motion_Demo();
            if (Motion_Aborted() != 0)
            {
                LOG("DEMO aborted\r\n");
            }
            else
            {
                LOG("DEMO done\r\n");
            }
            break;

        default:
            LOG("ERR unknown: %s\r\n", cmd);
            break;
    }
}

void App_Init(void)
{
    /* 1) 先注册"收到字节交给谁"，再开中断接收（顺序不能反）。
     *    注册的是本文件的 App_RxByte（不是 Protocol_FeedByte），
     *    为了在协议层之前抢一个急停入口。 */
    Bsp_Uart_SetRxByteHandler(App_RxByte);
    Bsp_Uart_RxStart();

    /* 2) 四路电机初始化（打开 PWM 通道，保持时基停止） */
    Motion_Init();

    /* 2b) 遥控接收机采集（阶段 A：只采集，不动电机）。
     *     内部手工配置 TIM3_CH1，不依赖 CubeMX 生成的 tim.c。 */
    App_Rc_Init();

    /* 3) 上电不再自启动任何动作 —— 等上位机命令。
     *    需要演示请发 'D'。 */
    LOG("READY. '?' for status, 'H' for help\r\n");

    /* 4) 调试自动动作（默认关闭；见 Config/Inc/cfg_debug.h） */
#if (CFG_DEBUG_AUTOTEST != 0)
    App_DebugAutoTest();
#endif
}

void App_Loop(void)
{
    char frame[PROTOCOL_FRAME_MAX];
    u16  len = 0;

    /* ---- ① 急停兜底（必做，别删）-------------------------------------
     * 中断里的快通道只置 s_abort 标志，**它自己不碰硬件**。
     * 阻塞动作内部（Motion_RampTo / Motion_WaitMs）会检查这个标志并停车，
     * 但"车正在转、主循环空闲"这种最常见的情况下，没有任何人在看它 ——
     * 直到 Motion_Tick() 的 3 秒掉线超时才停。也就是说：
     * 发 F1000 之后立刻发 E，车还会再转 3 秒，而不是"立即停"。
     *
     * 这里补上：只要处于中止状态且还有速度，就立刻断开脉冲。
     * 主循环没有阻塞动作时一圈 < 1ms，所以急停延迟实际 < 1ms。
     * ---------------------------------------------------------------- */
    if ((Motion_Aborted() != 0) && (Motion_IsMoving() != 0))
    {
        Motion_Stop();
    }

    /* ---- ② 接收通道保活 ----------------------------------------------
     * 错误回调里若因 BUSY 没武装上，这里补一次；正常情况只是一个标志判断。 */
    Bsp_Uart_Service();

    /* ---- ③ 掉线保护：超时没有运动命令就自动停车 ---------------------- */
    Motion_Tick();

    /* ---- ③b 遥控采集（阶段 A：只读，不驱动电机）----------------------
     * 现在只做两件事：解帧 + 检测"信号来了/丢了"并打日志。
     * 阶段 B 才在这里做通道映射 → Motion_Move()。
     * ⚠️ 只加这一行，不要重排周围的代码：另一个人正在改本文件时，
     *    这样合并起来是一处干净的新增，不产生冲突。
     * -------------------------------------------------------------------- */
    App_Rc_Tick();          /* USER-ANCHOR: 遥控接入点（阶段 A） */

    /* ---- ④ 急停快通道的回执（中断里只置位，这里补一句日志）---------- */
    if (s_estop_hit != 0)
    {
        s_estop_hit = 0;
        LOG("ESTOP (hard)\r\n");
    }

    /* ---- ⑤ 串口收帧：Service 组好一帧，这里取走并执行 ---------------- */
    if (Protocol_TakeFrame(frame, (u16)sizeof(frame), &len) != 0)
    {
        App_ExecCmd(frame);
    }
}
