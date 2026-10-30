#include "motion.h"
#include "bsp_motor_pulse.h"
#include "osal.h"
#include "cfg_motion.h"
#include "cfg_motor.h"   /* CFG_PUL_MAX_HZ：遥控通道的硬件上限 */

/* ============================================================================
 * App 层 — 运动控制
 *
 * 分家规则（这是整个重构最核心的一刀）：
 *   - 这一层知道"左前轮/右后轮/左转/右转"   → 留在本文件（App）
 *   - 下层只知道"某一路通道 / PSC 分频 / EGR" → 在 Bsp/bsp_motor_pulse.c
 *
 * 本文件只允许通过 Bsp 的接口碰硬件：不许 include "tim.h"，不许 HAL_GPIO_WritePin。
 *
 * 速度模型：
 *   s_speed_left / s_speed_right 记录"当前已下达"的左右速度（Hz，有符号）。
 *   所有平滑动作都从这里当起点，所以调用者不需要传"从多少开始"。
 *
 * 安全模型（本文件新增的那一半）：
 *   ① s_abort       —— 急停请求，只能置位/清零，不能当量用；等待循环每 8ms 看一眼；
 *   ② s_alive_ms    —— 上位机最后一次运动命令的时间戳，Motion_Tick() 靠它判断掉线；
 *   ③ 换向必须先停   —— 一侧符号翻转时，先降到 0 停脉冲、等 DIR 建立时间，再反向；
 *   ④ 速度/时长上限 —— 所有入口统一夹到 CFG_MOTION_MAX_SPEED_HZ / CFG_MOTION_MAX_RUN_MS。
 *   这四条加起来，才可能"发一条指令之后车不会一直转下去"。
 * ============================================================================ */

/* 车体语义 → Bsp 通道（这层映射本身就是 App 层的职责） */
#define MOTION_CH_FL   MOTOR_CH_L1
#define MOTION_CH_RL   MOTOR_CH_L2
#define MOTION_CH_FR   MOTOR_CH_R1
#define MOTION_CH_RR   MOTOR_CH_R2

/* 左/右侧轮在车体上镜像安装，同一 DIR 电平下两侧转向相反 */
#define MOTION_DIR_LEFT_FWD    MOTOR_DIR_CCW
#define MOTION_DIR_LEFT_BWD    MOTOR_DIR_CW
#define MOTION_DIR_RIGHT_FWD   MOTOR_DIR_CW
#define MOTION_DIR_RIGHT_BWD   MOTOR_DIR_CCW

/* ---------------------------------------------------------------------------
 * S 曲线过渡表（101 点，0 ~ 1000 表示 0.0% ~ 100.0%）
 *   本质为 cosine S 曲线: y(t) = (1-cos(pi*t))/2
 *   加速度在两端为 0、中间最大，起点/终点速度无阶跃跳变，
 *   避免 37cm 小车斜坡加速时更容易失步的问题。
 * ------------------------------------------------------------------------ */
static const u16 s_scurve_table[CFG_SCURVE_POINTS] = {
      0,   0,   1,   2,   4,   6,   9,  12,  16,  20,
     24,  30,  35,  41,  48,  54,  62,  70,  78,  86,
     95, 105, 115, 125, 136, 146, 158, 169, 181, 194,
    206, 219, 232, 245, 259, 273, 287, 301, 316, 331,
    345, 361, 376, 391, 406, 422, 437, 453, 469, 484,
    500, 516, 531, 547, 563, 578, 594, 609, 624, 639,
    655, 669, 684, 699, 713, 727, 741, 755, 768, 781,
    794, 806, 819, 831, 842, 854, 864, 875, 885, 895,
    905, 914, 922, 930, 938, 946, 952, 959, 965, 970,
    976, 980, 984, 988, 991, 994, 996, 998, 999, 1000,
    1000
};

/* 当前已下达的左右速度（Hz，有符号）。改硬件的地方只有 Motion_Move()。 */
static s32 s_speed_left  = 0;
static s32 s_speed_right = 0;

/* 当前加速度（Hz/s）。S 曲线变速时长 = |速度差| / s_accel_hz_per_s。
 * 默认 2000 == "1000Hz 用 500ms"；运行时可调，见 Motion_SetAccel()。 */
static u32 s_accel_hz_per_s = CFG_SCURVE_ACCEL_HZ_PER_S;

/* ---------------------------------------------------------------------------
 * 非阻塞遥控通道的私有状态（见 motion.h 第 ⑤ 组）
 *
 *   s_target_*      遥控"想要"的速度。只由 Motion_SetTarget() 写。
 *   s_slew_ms       上次推进斜坡的时间戳。
 *   s_slew_fraction 上一步不足 1Hz 的余数（Hz×1000），累积到下一步。
 *
 * 为什么余数要累积：加速度 1000Hz/s、节拍 8ms → 每步只有 8Hz，(8×1000)/1000
 * 正好整除；但只要把加速度调成 100Hz/s，每步就只剩 0.8Hz，整数截断后
 * **永远到不了目标**（一直差最后 1Hz），而 1Hz 在低频段就是明显的台阶。
 * ------------------------------------------------------------------------ */
static s32 s_target_left    = 0;
static s32 s_target_right   = 0;
static u32 s_slew_ms        = 0;
static u32 s_slew_fraction  = 0;

/* 急停请求标志：只由 Motion_RequestAbort() 置位（可能在中断里），
 * 由 Motion_ClearAbort() 清零。等待循环每 8ms 读一次。 */
static volatile u8 s_abort = 0;

/* 上位机最后一次运动命令的时间戳（0 表示还没收到过任何运动命令） */
static volatile u32 s_alive_ms = 0;

/* ---------------------------------------------------------------------------
 * 内部工具
 * ------------------------------------------------------------------------ */

/* 统一速度限幅：以**硬件能出的最高频** CFG_PUL_MAX_HZ 为上限。
 *
 *   为什么不卡 CFG_MOTION_MAX_SPEED_HZ(1200)：那是调试期的保守软件档，
 *   只该作为"上位机无人值守时"的建议值，不该让摇杆推到底也只出 1200Hz。
 *   遥控映射出来的峰值（CFG_RC_MAX_SPEED_HZ）本来就按 2500 设计，
 *   一旦在这里被砍到 1200，手感会变成"摇杆后半程全无响应"。
 *
 *   注意两侧共用这一条限幅：阻塞命令（Motion_Drive/Steer）与遥控
 *   （Motion_SetTarget→Motion_SlewTick→Motion_Move）最终都落到 Motion_Move，
 *   同一套硬件就必须同一套上下限，否则两条路的行为会不一致。 */
static s32 Motion_Clamp(s32 v)
{
    if (v >  (s32)CFG_PUL_MAX_HZ) { return  (s32)CFG_PUL_MAX_HZ; }
    if (v < -(s32)CFG_PUL_MAX_HZ) { return -(s32)CFG_PUL_MAX_HZ; }
    return v;
}

/* from → to 是否要跨过零点换向（任一端为 0 不算换向） */
static u8 Motion_SignFlip(s32 from, s32 to)
{
    if ((from == 0) || (to == 0))
    {
        return 0;
    }
    return ((from > 0) != (to > 0)) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------
 * 可中断的延时：每 CFG_SCURVE_SAMPLE_MS 检查一次急停标志
 *   返回 1 = 被急停打断，0 = 正常等满
 * ------------------------------------------------------------------------ */
static u8 Motion_WaitMs(u32 ms)
{
    u32 t0;
    u32 elapsed;
    u32 remain;

    if (s_abort != 0)
    {
        return 1;
    }
    if (ms == 0)
    {
        return 0;
    }

    t0 = Osal_GetTickMs();
    for (;;)
    {
        elapsed = Osal_GetTickMs() - t0;
        if (elapsed >= ms)
        {
            break;
        }
        remain = ms - elapsed;
        Osal_DelayMs((remain < CFG_SCURVE_SAMPLE_MS) ? remain : CFG_SCURVE_SAMPLE_MS);

        if (s_abort != 0)
        {
            return 1;
        }
    }
    return 0;
}

void Motion_Init(void)
{
    /* 时钟、GPIO、AF 复用、时基、PWM 通道全部由 CubeMX 生成代码完成，
     * Bsp 只负责打开 4 个通道输出并保持两路定时器停止。 */
    Bsp_MotorPulse_Init();

    s_speed_left  = 0;
    s_speed_right = 0;
    s_abort       = 0;
    s_alive_ms    = 0;

    /* 非阻塞通道一并清零：上电时"目标 = 当前速度 = 0"，
     * 遥控状态机在拿到有效帧之前调 Motion_SlewTick() 也不会让车动。 */
    s_target_left   = 0;
    s_target_right  = 0;
    s_slew_fraction = 0;
    s_slew_ms       = Osal_GetTickMs();
}

/* ---------------------------------------------------------------------------
 * 底层：立即把左右速度写进硬件（不平滑）
 *   v > 0 前进，v < 0 后退，v == 0 该侧停止
 * ------------------------------------------------------------------------ */
void Motion_Move(s32 vLeft, s32 vRight)
{
    u32 fl;
    u32 fr;

    vLeft  = Motion_Clamp(vLeft);
    vRight = Motion_Clamp(vRight);

    fl = (vLeft  < 0) ? (u32)(-vLeft)  : (u32)vLeft;
    fr = (vRight < 0) ? (u32)(-vRight) : (u32)vRight;

    /* 先定方向（此时可能还在转，由 S 曲线保证过零前已减速） */
    Bsp_MotorPulse_SetDir(MOTION_CH_FL, (vLeft  < 0) ? MOTION_DIR_LEFT_BWD  : MOTION_DIR_LEFT_FWD);
    Bsp_MotorPulse_SetDir(MOTION_CH_RL, (vLeft  < 0) ? MOTION_DIR_LEFT_BWD  : MOTION_DIR_LEFT_FWD);
    Bsp_MotorPulse_SetDir(MOTION_CH_FR, (vRight < 0) ? MOTION_DIR_RIGHT_BWD : MOTION_DIR_RIGHT_FWD);
    Bsp_MotorPulse_SetDir(MOTION_CH_RR, (vRight < 0) ? MOTION_DIR_RIGHT_BWD : MOTION_DIR_RIGHT_FWD);

    /* 顺序：先 SetDir，再 SetFreq。
     *
     * ⚠️ 这里**不调用** Bsp_MotorPulse_StartAll()：SetFreq(非零) 自己会按
     *    "装载分频 → 打开 CCxE → 置 CEN" 的顺序把该侧启动起来，而 SetFreq(0)
     *    会把该侧三道闸全关。所以：
     *      · 一侧 0、一侧非零 → 停的那侧保持停，不会被顺带重启
     *      · StartAll() 不看 PSC，无条件让两侧按旧分频开跑，正是它该被避开的原因 */
    Bsp_MotorPulse_SetFreq(MOTION_CH_FL, fl);          /* 左侧（FL/RL 共用 TIM1） */
    Bsp_MotorPulse_SetFreq(MOTION_CH_FR, fr);          /* 右侧（FR/RR 共用 TIM2） */

    s_speed_left  = vLeft;
    s_speed_right = vRight;
}

/* ---------------------------------------------------------------------------
 * 立即停车：把两路脉冲频率直接打成 0（不等 S 曲线）
 *   急停用。停下后 s_speed_* 归零，下一次 Motion_Drive 从 0 起步。
 *   ⚠️ 只是停脉冲：驱动器仍通电锁轴（静态电流约峰值的 50%），
 *      要真正断电只能断驱动器 ENA 或断主电。
 * ------------------------------------------------------------------------ */
void Motion_Stop(void)
{
    Bsp_MotorPulse_SetFreq(MOTION_CH_FL, 0);
    Bsp_MotorPulse_SetFreq(MOTION_CH_FR, 0);

    s_speed_left  = 0;
    s_speed_right = 0;

    /* ⚠️ 遥控通道也要一起清目标值，否则会出现"停不下来"：
     *    RcControl_Tick() 只要摇杆还推着，每帧都会 Motion_SetTarget(非0)，
     *    而 Motion_SlewTick() 下一拍就会把车重新推起来 —— 中间这段"停"
     *    短到肉眼看不见。所以任何"停车"都必须同时把遥控目标清零。
     *    （原来的 Motion_Stop 只清当前速度，因为过去没有别的写者。） */
    s_target_left   = 0;
    s_target_right  = 0;
    s_slew_fraction = 0;
    s_slew_ms       = Osal_GetTickMs();
}

void Motion_RequestAbort(void)
{
    s_abort = 1;
}

void Motion_ClearAbort(void)
{
    s_abort = 0;
}

u8 Motion_Aborted(void)
{
    return s_abort;
}

void Motion_NotifyAlive(void)
{
    s_alive_ms = Osal_GetTickMs();
    if (s_alive_ms == 0)
    {
        s_alive_ms = 1;                    /* 0 保留为"从未收到命令" */
    }
}

u8 Motion_IsMoving(void)
{
    return ((s_speed_left != 0) || (s_speed_right != 0)) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------
 * 通信看门狗：主循环每圈调用
 *   收到过运动命令、而且现在确实在转、但超过 CFG_CMD_TIMEOUT_MS 没新命令
 *   → 直接停车。这样"串口线被拔掉/上位机崩了"不会留下一个一直转的车。
 * ------------------------------------------------------------------------ */
void Motion_Tick(void)
{
    /* ① 安全网优先于一切：两侧速度都是 0 就绝不该有脉冲。
     *
     *    ⚠️ 这一条**必须**放在 s_alive_ms 判断之前。曾经的顺序是先
     *    `if (s_alive_ms == 0) return;` —— 而看门狗停车时恰好会把 s_alive_ms
     *    清零，于是之后每圈都在第一行返回，安全网永远执行不到。
     *
     *    这里**不需要**任何额外动作：本工程的 SetFreq(0) 是三道闸一起关
     *    （CEN=0 + CCxE=0 + TIM1 MOE=0），停车之后引脚上一定没有脉冲；
     *    而"速度都是 0"只可能来自 Motion_Stop()/Motion_StopImmediate()/
     *    Motion_Move(0,0)，它们都已经走过一次 SetFreq(0)。
     *    所以这条分支只做看门狗复位，零寄存器写。 */
    if ((s_speed_left == 0) && (s_speed_right == 0))
    {
        s_alive_ms = 0;                    /* 已停稳，看门狗一并复位 */
        return;
    }

    if (s_alive_ms == 0)
    {
        /* ⚠️ 这里**不能**做任何事，包括 return —— 遥控（RcControl_Tick +
         * Motion_SlewTick）走的不是"命令看门狗"这条路，它从来不设 s_alive_ms。
         * 遥控的失效判定在 RcControl_Tick 里（CFG_RC_FRAME_TIMEOUT_MS=200ms，
         * 比这里的 CFG_CMD_TIMEOUT_MS=3000ms 严得多），由那一层负责停车。
         * 本函数只负责上位机命令的掉线保护。 */
    }
    else if ((Osal_GetTickMs() - s_alive_ms) > CFG_CMD_TIMEOUT_MS)
    {
        Motion_Stop();
        s_alive_ms = 0;
    }
}

/* ---------------------------------------------------------------------------
 * 内部：按 S 曲线从"当前速度"过渡到目标速度（阻塞）
 *
 *   变速时长由加速度推出：durationMs = max(|Δ左|, |Δ右|) × 1000 / 加速度
 *   取两侧中变化更大的一侧，保证哪一轮都不超过设定的加速度上限；
 *   两侧共用同一条进度曲线，所以差速转向时左右轮同时到位、轨迹不会歪。
 *
 *   进度按"真实经过时间"算（Osal_GetTickMs），不是数循环次数，
 *   所以 DelayMs 的抖动不会累积成时长误差。
 *
 *   每步检查一次急停标志：被急停就直接停车返回，不会把这一段走完。
 * ------------------------------------------------------------------------ */
static void Motion_RampTo(s32 vLeftTo, s32 vRightTo)
{
    /* 必须先快照：循环里 Motion_Move() 会改写 s_speed_*，不能拿它当起点 */
    const s32 leftFrom  = s_speed_left;
    const s32 rightFrom = s_speed_right;
    const s32 dLeft     = vLeftTo  - leftFrom;
    const s32 dRight    = vRightTo - rightFrom;
    u32 span, durationMs, t0, elapsed, idx;

    if ((dLeft == 0) && (dRight == 0))
    {
        return;                                        /* 已在目标速度，不空转 */
    }

    span = (u32)((dLeft  < 0) ? -dLeft  : dLeft);
    if ((u32)((dRight < 0) ? -dRight : dRight) > span)
    {
        span = (u32)((dRight < 0) ? -dRight : dRight);
    }

    durationMs = (span * 1000UL) / s_accel_hz_per_s;
    if (durationMs < CFG_SCURVE_MIN_MS)
    {
        durationMs = CFG_SCURVE_MIN_MS;
    }

    t0 = Osal_GetTickMs();
    for (;;)
    {
        if (s_abort != 0)                              /* 急停：立即打断这一坡 */
        {
            Motion_Stop();
            return;
        }

        elapsed = Osal_GetTickMs() - t0;
        if (elapsed >= durationMs)
        {
            break;
        }

        idx = (elapsed * (CFG_SCURVE_POINTS - 1)) / durationMs;
        Motion_Move(leftFrom  + ((vLeftTo  - leftFrom)  * (s32)s_scurve_table[idx]) / 1000,
                    rightFrom + ((vRightTo - rightFrom) * (s32)s_scurve_table[idx]) / 1000);

        Osal_DelayMs(CFG_SCURVE_SAMPLE_MS);
    }

    Motion_Move(vLeftTo, vRightTo);                    /* 精确落到目标值 */
}

/* ---------------------------------------------------------------------------
 * ① 转弯 / 差速：左轮 vLeft、右轮 vRight（Hz，有符号）
 *   vLeft > vRight → 右转     vRight > vLeft → 左转     相等 → 直行
 *   负数表示该轮反转，例如 Motion_Steer(-2500, 2500) 就是原地左转。
 *
 *   换向路径（这是本次改动最要紧的一条）：
 *     任一侧符号翻转时，先把两侧都降到 0 并停掉脉冲，
 *     等 CFG_MOTION_DIR_SETTLE_MS，再往目标方向加速。
 *     理由：DIR 在脉冲边沿还在走的时候翻转，驱动器会丢步/抖动，
 *     表现就是"空转 + 电流上升"。先停再反向是从根上掐断这条路径。
 * ------------------------------------------------------------------------ */
void Motion_Steer(s32 vLeft, s32 vRight, u32 holdMs)
{
    const s32 l = Motion_Clamp(vLeft);
    const s32 r = Motion_Clamp(vRight);

    if (holdMs > CFG_MOTION_MAX_RUN_MS)                /* 单段时长硬上限 */
    {
        holdMs = CFG_MOTION_MAX_RUN_MS;
    }

    if ((Motion_SignFlip(s_speed_left, l) != 0) || (Motion_SignFlip(s_speed_right, r) != 0))
    {
        Motion_RampTo(0, 0);                           /* 平滑降到 0 */
        Motion_Stop();                                 /* 确保脉冲真的停了 */
        (void)Motion_WaitMs(CFG_MOTION_DIR_SETTLE_MS); /* DIR 建立时间，可被急停打断 */

        if (s_abort != 0)
        {
            Motion_Stop();
            return;
        }
    }

    Motion_RampTo(l, r);                               /* 平滑到目标速度 */

    if (s_abort != 0)
    {
        Motion_Stop();
        return;
    }

    if (holdMs > 0)
    {
        (void)Motion_WaitMs(holdMs);                   /* 到达目标后保持（可被急停打断） */
    }
}

/* ---------------------------------------------------------------------------
 * ① 进退：speed > 0 前进、< 0 后退、== 0 停止，左右同速（直线）
 * ------------------------------------------------------------------------ */
void Motion_Drive(s32 speed, u32 holdMs)
{
    Motion_Steer(speed, speed, holdMs);
}

/* ---------------------------------------------------------------------------
 * ① 停车：两侧 S 曲线降到 0
 * ------------------------------------------------------------------------ */
void Motion_Brake(void)
{
    Motion_Steer(0, 0, 0);
}

/* ---------------------------------------------------------------------------
 * ② 底层：读回当前已下达的速度（Hz，有符号），便于调试时观察
 * ------------------------------------------------------------------------ */
void Motion_GetSpeed(s32 *vLeft, s32 *vRight)
{
    if (vLeft  != 0) { *vLeft  = s_speed_left;  }
    if (vRight != 0) { *vRight = s_speed_right; }
}

/* ---------------------------------------------------------------------------
 * ③ 加速度：修改"多快达到目标速度"，对所有后续 S 曲线变速立即生效
 *   hzPerSec = 2000  → 1000Hz 用 500ms（默认值）
 *   hzPerSec = 4000  → 变速快一倍（0 → 2500Hz 用 625ms）
 *   hzPerSec 传 0 是非法值，直接忽略，保持原加速度不变。
 *   只改运行期状态，断电复位后回到 CFG_SCURVE_ACCEL_HZ_PER_S。
 * ------------------------------------------------------------------------ */
void Motion_SetAccel(u32 hzPerSec)
{
    if (hzPerSec == 0)
    {
        return;
    }

    s_accel_hz_per_s = hzPerSec;
}

u32 Motion_GetAccel(void)
{
    return s_accel_hz_per_s;
}

/* ---------------------------------------------------------------------------
 * ④ 原来的上电六段演示（内容与原 Core/Src/main.c USER CODE 2 段一致）
 *
 *   改成串口命令 'D' 触发，不再上电自动跑 ——
 *   上电跑演示 + 断电极易形成"断电→复位→再跑→再断电"的正反馈，
 *   而且演示一跑就是 13 秒，这 13 秒里主循环根本不执行。
 *
 *   每段之间检查急停标志，发 'E' 可以随时中断整段演示。
 * ------------------------------------------------------------------------ */
void Motion_Demo(void)
{
    Motion_ClearAbort();

    Motion_Drive( 2500,          1000);   /* 1) 从零起步：直行加速到 2500Hz，保持 1s */
    if (s_abort != 0) { return; }

    Motion_Steer( 2500,  1000,   1000);   /* 2) 右转：左轮 2500Hz、右轮 1000Hz，保持 1s */
    if (s_abort != 0) { return; }

    Motion_Steer( 1000,  2500,   1000);   /* 3) 回正（反向等量）：左轮 1000Hz、右轮 2500Hz，保持 1s */
    if (s_abort != 0) { return; }

    Motion_Drive( 2500,          1000);   /* 4) 前进：回到两侧 2500Hz，保持 1s */
    if (s_abort != 0) { return; }

    Motion_Drive(-2500,          2000);   /* 5) 减速后退：先停再换向到 -2500Hz，保持 2s */
    if (s_abort != 0) { return; }

    Motion_Brake();                       /* 6) 停车：两侧 S 曲线降到 0 */
}

/* ===========================================================================
 * ⑤ 非阻塞遥控通道
 *
 *   与上面的阻塞动作**互不调用**：这一组从不 Delay、从不等待、不改方向，
 *   只按节拍把速度朝目标推一小步。主循环必须一直能转，遥控才有意义。
 * ============================================================================ */

/* 遥控通道的限幅也用 CFG_PUL_MAX_HZ（与上面的 Motion_Clamp 保持同一个上限）。
 *
 * 如果要把遥控速度调低，改 cfg_rc.h 的 CFG_RC_MAX_SPEED_HZ 即可，
 * 不要动这里 —— 这里只管"不许超过硬件能出的最高频"。 */
static s32 Motion_ClampTarget(s32 speed)
{
    if (speed >  (s32)CFG_PUL_MAX_HZ) { return  (s32)CFG_PUL_MAX_HZ; }
    if (speed < -(s32)CFG_PUL_MAX_HZ) { return -(s32)CFG_PUL_MAX_HZ; }
    return speed;
}

void Motion_SetTarget(s32 vLeft, s32 vRight)
{
    s_target_left  = Motion_ClampTarget(vLeft);
    s_target_right = Motion_ClampTarget(vRight);
}

/* 单侧朝目标走一步：
 *   ① 目标与当前异号 → 先把目标压成 0（本步最多走到 0，绝不在带速时换向）
 *   ② 剩下的按步长逼近，到不了就只走 step
 * 返回新的速度值。 */
static s32 Motion_SlewStep(s32 current, s32 target, s32 step)
{
    if (((current > 0) && (target < 0)) || ((current < 0) && (target > 0)))
    {
        target = 0;
    }

    if (current < target)
    {
        return ((target - current) <= step) ? target : (current + step);
    }
    if (current > target)
    {
        return ((current - target) <= step) ? target : (current - step);
    }

    return current;
}

void Motion_SlewTick(u32 nowMs)
{
    u32 elapsed = nowMs - s_slew_ms;
    u32 credit;
    u32 step;
    s32 left;
    s32 right;

    if (elapsed < CFG_SCURVE_SAMPLE_MS)
    {
        return;                            /* 节拍未到：这一圈什么都不做 */
    }

    s_slew_ms = nowMs;

    /* 主循环被别的阻塞动作拖住时，elapsed 可能很大。
     * 夹紧到 CFG_MOTION_MAX_TICK_MS，否则一步就能跳过上百 Hz，等于没斜坡。 */
    if (elapsed > CFG_MOTION_MAX_TICK_MS)
    {
        elapsed = CFG_MOTION_MAX_TICK_MS;
    }

    /* 本步允许的最大速度变化（Hz）：
     *   accel(Hz/s) × elapsed(ms) / 1000 + 上一步的余数
     * 用 64 位只是为了让 accel 取大值时不溢出；余数保留到下一步。 */
    {
        uint64_t tot = (uint64_t)s_accel_hz_per_s * (uint64_t)elapsed
                     + (uint64_t)s_slew_fraction;
        credit          = (u32)(tot / 1000ULL);
        s_slew_fraction = (u32)(tot % 1000ULL);
    }

    if (credit == 0U)
    {
        return;                            /* 还没攒够 1Hz，等下一拍 */
    }

    /* 单侧最快也要 CFG_PUL_MAX_HZ 以上没有意义；夹一下防 step 过大 */
    if (credit > (2U * (u32)CFG_PUL_MAX_HZ))
    {
        credit = 2U * (u32)CFG_PUL_MAX_HZ;
    }
    step = credit;

    left  = Motion_SlewStep(s_speed_left,  s_target_left,  (s32)step);
    right = Motion_SlewStep(s_speed_right, s_target_right, (s32)step);

    if ((left != s_speed_left) || (right != s_speed_right))
    {
        Motion_Move(left, right);          /* 唯一改写硬件的地方，内部按需启停每一侧 */
    }
}

void Motion_StopImmediate(void)
{
    /* 上面两次 SetFreq(0) 就已经把三道闸全关死了（CEN=0 + CCxE=0 + TIM1 MOE=0），
     * 引脚上不可能再有脉冲，不需要再补一次"兜底关"。 */
    Bsp_MotorPulse_SetFreq(MOTION_CH_FL, 0);
    Bsp_MotorPulse_SetFreq(MOTION_CH_FR, 0);

    s_speed_left    = 0;
    s_speed_right   = 0;
    s_target_left   = 0;
    s_target_right  = 0;
    s_slew_fraction = 0;
    s_slew_ms       = Osal_GetTickMs();
}

