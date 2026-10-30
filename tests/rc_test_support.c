/* Host-only hardware boundary for the real parser/controller/motion sources. */
#include <stddef.h>
#include "osal.h"
#include "bsp_motor_pulse.h"
#include "mx_board.h"
#include "bsp_uart.h"
#include "ibus.h"
#include "protocol.h"

static u32 s_test_tick;
static u32 s_dir_mask;
static TIM_TypeDef s_left_registers;
static TIM_TypeDef s_right_registers;
TIM_HandleTypeDef test_htim_left = { &s_left_registers };
TIM_HandleTypeDef test_htim_right = { &s_right_registers };
GPIO_TypeDef test_gpio;
static USART_TypeDef s_host_uart = {1}, s_rc_uart = {3};
UART_HandleTypeDef test_huart_host = {&s_host_uart, 0U};
UART_HandleTypeDef test_huart_rc = {&s_rc_uart, 0U};
static u8 *s_host_rx;
static u8 *s_rc_rx;

/* RM0090 timer model: PSC is buffered until overflow or UG; UG also resets
 * counter phase. Count actual PWM falling edges, not just requested Hz. */
typedef struct { uint64_t phase; u32 active_psc; u32 falling_edges; } PulseModel;
static PulseModel s_pulse_model[2];

void Test_ResetPulseModel(void)
{
    u32 side;
    for (side = 0; side < 2; ++side)
    {
        TIM_TypeDef *tim = side == 0 ? &s_left_registers : &s_right_registers;
        s_pulse_model[side].phase = 0;
        s_pulse_model[side].active_psc = tim->PSC;
        s_pulse_model[side].falling_edges = 0;
        tim->EGR = 0;
    }
}

void Test_AdvancePulseUs(u32 side, u32 microseconds)
{
    TIM_TypeDef *tim = side == 0 ? &s_left_registers : &s_right_registers;
    PulseModel *model = &s_pulse_model[side];
    u32 clock = side == 0 ? CFG_TIM1_CLK_HZ : CFG_TIM2_CLK_HZ;
    uint64_t ticks = (uint64_t)(clock / 1000000U) * microseconds;
    if (tim->EGR & TIM_EGR_UG)
    {
        model->phase = 0;
        model->active_psc = tim->PSC;
        tim->EGR = 0; /* Hardware self-clears event-generation bits. */
    }
    if ((tim->CR1 & TIM_CR1_CEN) == 0 || (tim->CCER & TIM_CCER_CC1E) == 0) { return; }
    while (ticks != 0)
    {
        uint64_t period = ((uint64_t)model->active_psc + 1U) * CFG_PUL_ARR;
        uint64_t advance = period - model->phase;
        if (advance > ticks) { advance = ticks; }
        if (model->phase < period / 2U && model->phase + advance >= period / 2U)
        {
            ++model->falling_edges;
        }
        model->phase += advance;
        ticks -= advance;
        if (model->phase == period)
        {
            model->phase = 0;
            model->active_psc = tim->PSC;
        }
    }
}

u32 Test_GetPulseEdges(u32 side) { return s_pulse_model[side].falling_edges; }

void Test_SetTick(u32 value) { s_test_tick = value; }
u32 Test_GetFrequency(u32 channel)
{
    TIM_TypeDef *tim = channel < 2U ? &s_left_registers : &s_right_registers;
    u32 clock = channel < 2U ? CFG_TIM1_CLK_HZ : CFG_TIM2_CLK_HZ;
    if ((tim->CR1 & TIM_CR1_CEN) == 0U || (tim->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E)) == 0U) { return 0; }
    return clock / ((tim->PSC + 1U) * CFG_PUL_ARR);
}
u32 Test_GetCen(u32 side) { return (side == 0U ? s_left_registers.CR1 : s_right_registers.CR1) & TIM_CR1_CEN; }
u32 Test_GetCcer(u32 side) { return side == 0U ? s_left_registers.CCER : s_right_registers.CCER; }
u32 Osal_GetTickMs(void) { return s_test_tick; }
void Osal_DelayMs(u32 ms) { s_test_tick += ms; }
u32 Osal_EnterCritical(void) { return 0; }
void Osal_ExitCritical(u32 key) { (void)key; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, u16 pin, u32 value)
{
    (void)port;
    if (value == GPIO_PIN_SET) { s_dir_mask |= pin; }
    else { s_dir_mask &= ~(u32)pin; }
}
u32 Test_GetDir(u32 channel) { return (s_dir_mask >> channel) & 1U; }

int HAL_UART_Receive_IT(UART_HandleTypeDef *h, u8 *data, u16 size)
{
    (void)size;
    if (h == &test_huart_host) { s_host_rx = data; }
    else { s_rc_rx = data; }
    return 0;
}
int HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{
    if (h == &test_huart_host) { s_host_rx = 0; }
    else { s_rc_rx = 0; }
    return 0;
}
int HAL_UART_Transmit(UART_HandleTypeDef *h, u8 *data, u16 size, u32 timeout)
{
    (void)h; (void)data; (void)size; (void)timeout;
    return 0;
}
void Test_UartInit(void)
{
    Protocol_Reset();
    Bsp_Uart_SetRxByteHandler(Protocol_FeedByte);
    Bsp_Uart_RcSetRxByteHandler(Ibus_FeedByte);
    Bsp_Uart_RxStart();
    Bsp_Uart_RcRxStart();
}
void Test_UartInject(u32 port, u8 byte)
{
    u8 *dest = port == 1U ? s_host_rx : s_rc_rx;
    if (dest != 0)
    {
        *dest = byte;
        HAL_UART_RxCpltCallback(port == 1U ? &test_huart_host : &test_huart_rc);
    }
}
void Test_UartError(u32 port)
{
    HAL_UART_ErrorCallback(port == 1U ? &test_huart_host : &test_huart_rc);
}

/* No CRT is needed for the Windows native test DLL. */
void *memcpy(void *dest, const void *src, size_t count)
{
    unsigned char *d = dest;
    const unsigned char *s = src;
    size_t i;
    for (i = 0; i < count; ++i) { d[i] = s[i]; }
    return dest;
}
void *memset(void *dest, int value, size_t count)
{
    unsigned char *d = dest;
    size_t i;
    for (i = 0; i < count; ++i) { d[i] = (unsigned char)value; }
    return dest;
}
