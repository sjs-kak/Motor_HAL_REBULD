#include "app_rc.h"
#include "cfg_rc.h"

/* 本文件属于 PWM/PPM 备选方案，默认（i-BUS）下整个文件编空。
 * 见 cfg_rc.h 的 CFG_RC_SOURCE_PWM 说明。 */
#if (CFG_RC_SOURCE_PWM != 0)

#include "bsp_rc.h"
#include "log.h"

/* ============================================================================
 * App 层 — 遥控业务（阶段 A）
 *
 * ---------------------------------------------------------------------------
 * 阶段 A 的边界（故意划在这里）
 *
 *   做：采集 6 个通道 → 判断信号新鲜度 → 串口打印。
 *   不做：任何 Motion_* 调用。本文件连 motion.h 都不 include。
 *
 *   结果：即使另一个人把 motion.c 改得编译不过、或者逻辑改错，
 *         遥控这条路依然能独立验证（"摇杆动 → 数字动"）。
 *         反过来也成立：遥控接线错了，不会影响车轮控制调试。
 *
 * ---------------------------------------------------------------------------
 * 阶段 B 待办（等运动控制稳定后再加，不要在阶段 A 提前写）
 *
 *   1) 删掉下面那个 #error 守卫，把 CFG_RC_DRIVE_ENABLE 改成 1
 *   2) 在 App_Rc_Tick() 里加映射 + 斜率限幅 + Motion_Move()：
 *        用 Motion_Move()，**不要用 Motion_Drive/Motion_Steer**——
 *        后两个是阻塞的，遥控需要持续输入，会被卡死在第一个斜坡里。
 *   3) 100 ms 无新帧 → Motion_Stop()（不要只依赖接收机 failsafe）
 *   4) 控制源仲裁（建议 CH5）：遥控优先/串口优先，否则串口发 F1000
 *        会被下一帧遥控盖掉，调试时会误判断串口坏了
 *   5) 遥控驱动时也要调 Motion_NotifyAlive()，否则 Motion_Tick() 的
 *        3 秒掉线保护会把车停掉；"断信号停车"改由上面第 3 条接管
 * ============================================================================ */

#if (CFG_RC_DRIVE_ENABLE != 0)
#error "阶段 A 只采集不驱动：驱动逻辑在阶段 B 才实现，请先把 CFG_RC_DRIVE_ENABLE 改回 0"
#endif

/* 上一圈的信号状态，用来"只在变化时打一行日志"，不然串口会被刷爆 */
static u8 s_have = 0u;

void App_Rc_Init(void)
{
    Bsp_Rc_Init();

    if (Bsp_Rc_IsReady() == 0u)
    {
        LOG("RC init FAIL\r\n");
    }
}

void App_Rc_Tick(void)
{
    u8 now = Bsp_Rc_HasSignal();

    if ((s_have == 0u) && (now != 0u))
    {
        s_have = 1u;
        LOG("RC signal OK\r\n");
    }
    else if ((s_have != 0u) && (now == 0u))
    {
        s_have = 0u;
        LOG("RC signal LOST\r\n");
    }
    else
    {
        /* 状态没变化：什么都不做（阶段 B 在这里做映射 + 驱动） */
    }
}

void App_Rc_Report(void)
{
    u16 us[CFG_RC_CH_NUM];
    u32 i;
    u32 age;

    if (Bsp_Rc_IsReady() == 0u)
    {
        LOG("RC init FAIL (TIM3)\r\n");
        return;
    }

    age = Bsp_Rc_GetAgeMs();

    if (Bsp_Rc_GetChannels(us, (u8)CFG_RC_CH_NUM) == 0u)
    {
        /* 区分两种"没信号"，这是现场排查的分水岭：
         *   edges=0  → 一个沿都没抓到：线/供电/接收机灯/IC 极性
         *   edges>0  → 抓到了但组不成帧：多半是 CFG_RC_SYNC_US 判据不对 */
        LOG("RC nosig age=%u edges=%u frames=%u\r\n",
            (unsigned int)age,
            (unsigned int)Bsp_Rc_GetEdgeCount(),
            (unsigned int)Bsp_Rc_GetFrameCount());
        return;
    }

    LOG("RC");
    for (i = 0u; i < (u32)CFG_RC_CH_NUM; i++)
    {
        LOG(" %u", (unsigned int)us[i]);
    }
    LOG(" age=%u\r\n", (unsigned int)age);
}

#endif /* CFG_RC_SOURCE_PWM */
