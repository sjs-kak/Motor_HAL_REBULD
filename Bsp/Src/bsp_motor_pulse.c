#include "bsp_motor_pulse.h"
#include "mx_board.h"

/* ============================================================================
 * 四路步进脉冲输出（PUL）＋ 四路方向（DIR）
 *
 * 频率公式：  f = tim_clk / ((PSC + 1) * ARR)
 *   固定 ARR（CFG_PUL_ARR），靠改 PSC 调频。
 * ============================================================================ */

/* 诊断计数：见 bsp_motor_pulse.h 的说明 */
static volatile u32 s_start_cnt  = 0;
static volatile u32 s_stop_cnt   = 0;
static volatile u32 s_forced_cnt = 0;

/* 分清"哪一路通道属于哪一侧定时器" */
static TIM_HandleTypeDef *Bsp_ChTimer(MotorChannel ch)
{
    return (ch == MOTOR_CH_L1 || ch == MOTOR_CH_L2) ? BOARD_HTIM_LEFT
                                                    : BOARD_HTIM_RIGHT;
}

static u32 Bsp_ChTimerClk(MotorChannel ch)
{
    return (ch == MOTOR_CH_L1 || ch == MOTOR_CH_L2) ? CFG_TIM1_CLK_HZ
                                                    : CFG_TIM2_CLK_HZ;
}

static GPIO_TypeDef *Bsp_ChDirPort(MotorChannel ch)
{
    switch (ch)
    {
        case MOTOR_CH_L1: return DIR_LF_PORT;
        case MOTOR_CH_L2: return DIR_LR_PORT;
        case MOTOR_CH_R1: return DIR_RF_PORT;
        default:          return DIR_RR_PORT;
    }
}

static uint16_t Bsp_ChDirPin(MotorChannel ch)
{
    switch (ch)
    {
        case MOTOR_CH_L1: return DIR_LF_PIN;
        case MOTOR_CH_L2: return DIR_LR_PIN;
        case MOTOR_CH_R1: return DIR_RF_PIN;
        default:          return DIR_RR_PIN;
    }
}

/* ---------------------------------------------------------------------------
 * 改一路定时器的分频系数，使其输出 freq(Hz) 的脉冲
 * ------------------------------------------------------------------------ */
static void Bsp_SetTimerFreq(TIM_HandleTypeDef *htim, u32 tim_clk, u32 freq)
{
    u32 psc;

    if (freq == 0)                          /* 频率为 0 即为停止 */
    {
        s_stop_cnt++;
        HAL_TIM_Base_Stop(htim);
        return;
    }

    if (freq < CFG_PUL_MIN_HZ) freq = CFG_PUL_MIN_HZ;   /* 频率下限保护 */
    if (freq > CFG_PUL_MAX_HZ) freq = CFG_PUL_MAX_HZ;   /* 频率上限保护 */

    psc = (tim_clk / (freq * CFG_PUL_ARR)) - 1;
    __HAL_TIM_SET_PRESCALER(htim, psc);

    /* ⚠️ 只有定时器**停止着**的时候才能写 UG（更新事件）。
     *
     * UG 会把计数器清零并重新装载预分频器——如果定时器正在输出脉冲，
     * 这等于把当前这个脉冲强行截断/拉长，可能短到 <2.5µs。
     * M860D 手册要求脉冲高低电平各 ≥2.5µs，短脉冲驱动器不认 = 丢步，
     * 四轮丢步不齐就是"原地打滑 / 空转 / 电流上升"。
     *
     * 而运行中改 PSC 本来就会在**下一个更新事件自动生效**（PSC 有影子寄存器），
     * 根本不需要 UG 帮忙，最多晚一个脉冲周期。
     * S 曲线每 8ms 改一次频率，晚一个周期（400µs）完全无感。 */
    if ((htim->Instance->CR1 & TIM_CR1_CEN) == 0U)
    {
        htim->Instance->EGR = TIM_EGR_UG;
    }
}

void Bsp_MotorPulse_Init(void)
{
    HAL_TIM_PWM_Start(BOARD_HTIM_LEFT,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(BOARD_HTIM_LEFT,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(BOARD_HTIM_RIGHT, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(BOARD_HTIM_RIGHT, TIM_CHANNEL_2);

    HAL_TIM_Base_Stop(BOARD_HTIM_LEFT);
    HAL_TIM_Base_Stop(BOARD_HTIM_RIGHT);
}

void Bsp_MotorPulse_SetDir(MotorChannel ch, MotorDir dir)
{
    HAL_GPIO_WritePin(Bsp_ChDirPort(ch), Bsp_ChDirPin(ch),
                      (dir == MOTOR_DIR_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bsp_MotorPulse_SetFreq(MotorChannel ch, u32 hz)
{
    Bsp_SetTimerFreq(Bsp_ChTimer(ch), Bsp_ChTimerClk(ch), hz);
}

void Bsp_MotorPulse_StartAll(void)
{
    s_start_cnt++;

    /* TIM1 是高级定时器：除了 CEN，还要 BDTR.MOE=1 输出才会真正到引脚。
     * HAL_TIM_PWM_Start 当初已经置过 MOE，但 EnsureStopped 会把它关掉，
     * 所以每次启动都要重新打开。 */
    __HAL_TIM_MOE_ENABLE(BOARD_HTIM_LEFT);

    (void)HAL_TIM_Base_Start(BOARD_HTIM_LEFT);
    (void)HAL_TIM_Base_Start(BOARD_HTIM_RIGHT);
}

/* ---------------------------------------------------------------------------
 * 幂等兜底：两层，任何一层生效就没有脉冲
 *   第一层：清 CEN（对 TIM1 / TIM2 都有效）
 *   第二层：TIM1 关 MOE —— 高级定时器的主输出使能，为 0 时**无论 CEN、CCR、
 *           CCER 是什么状态，输出引脚都被硬件强制为无效电平**
 *
 * 实测背景：速度已清零、CEN 被这里反复清掉，但定时器又在同一圈里被重新使能
 * （s_forced_cnt 涨到 500 万仍没停）。第二层就是针对这种"CEN 被别人抢回去"
 * 的场合：不依赖 CEN 也能停。
 * ------------------------------------------------------------------------ */
void Bsp_MotorPulse_EnsureStopped(void)
{
    if ((BOARD_HTIM_LEFT->Instance->CR1 & TIM_CR1_CEN) != 0U)
    {
        s_forced_cnt++;
        BOARD_HTIM_LEFT->Instance->CR1 &= ~TIM_CR1_CEN;
    }

    if ((BOARD_HTIM_RIGHT->Instance->CR1 & TIM_CR1_CEN) != 0U)
    {
        s_forced_cnt++;
        BOARD_HTIM_RIGHT->Instance->CR1 &= ~TIM_CR1_CEN;
    }

    /* 第二层兜底：直接关主输出 */
    __HAL_TIM_MOE_DISABLE(BOARD_HTIM_LEFT);
}

__attribute__((used)) u32 Bsp_MotorPulse_GetForcedStopCount(void)
{
    return s_forced_cnt;
}
