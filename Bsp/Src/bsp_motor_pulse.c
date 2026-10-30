#include "bsp_motor_pulse.h"
#include "mx_board.h"

/* ============================================================================
 * 四路步进脉冲输出（PUL）＋ 四路方向（DIR）
 *
 * 频率公式：  f = tim_clk / ((PSC + 1) * ARR)
 *   固定 ARR（CFG_PUL_ARR），靠改 PSC 调频。
 * ============================================================================ */

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
        /* HAL_TIM_Base_Stop uses __HAL_TIM_DISABLE, which preserves CEN while
         * CCxE is set. We own this whole timer, so stop it unconditionally and
         * gate BOTH channels on the side. No residual pulse train is allowed. */
        CLEAR_BIT(htim->Instance->CR1, TIM_CR1_CEN);
        CLEAR_BIT(htim->Instance->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
        return;
    }

    if (freq < CFG_PUL_MIN_HZ) freq = CFG_PUL_MIN_HZ;   /* 频率下限保护 */
    if (freq > CFG_PUL_MAX_HZ) freq = CFG_PUL_MAX_HZ;   /* 频率上限保护 */

    psc = (tim_clk / (freq * CFG_PUL_ARR)) - 1;
    __HAL_TIM_SET_PRESCALER(htim, psc);
    if ((htim->Instance->CR1 & TIM_CR1_CEN) == 0U)
    {
        /* A stopped side loads its divider before exposing either output. */
        htim->Instance->EGR = TIM_EGR_UG;
        SET_BIT(htim->Instance->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
        __HAL_TIM_ENABLE(htim);
    }
    /* Running timer: PSC is buffered and loads at the next natural overflow
     * (RM0090). Do NOT generate UG here: it resets the counter phase. Small
     * RC changes every 8 ms could otherwise suppress low-frequency steps or
     * truncate pulses indefinitely. A stop still gates both channels above. */
}

void Bsp_MotorPulse_Init(void)
{
    /* CubeMX already configured PWM mode/CCR. Do NOT call PWM_Start here:
     * it would briefly start pulses at reset before RC has been armed. */
    Bsp_SetTimerFreq(BOARD_HTIM_LEFT, CFG_TIM1_CLK_HZ, 0);
    Bsp_SetTimerFreq(BOARD_HTIM_RIGHT, CFG_TIM2_CLK_HZ, 0);
    __HAL_TIM_MOE_ENABLE(BOARD_HTIM_LEFT);
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
    SET_BIT(BOARD_HTIM_LEFT->Instance->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    SET_BIT(BOARD_HTIM_RIGHT->Instance->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    __HAL_TIM_ENABLE(BOARD_HTIM_LEFT);
    __HAL_TIM_ENABLE(BOARD_HTIM_RIGHT);
}
