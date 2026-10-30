#ifndef TEST_MX_BOARD_H
#define TEST_MX_BOARD_H

/* Minimal register model for the REAL bsp_motor_pulse.c, never firmware. */
#include "types.h"
typedef struct { volatile u32 CR1, CCER, PSC, EGR, BDTR; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { u32 unused; } GPIO_TypeDef;
extern TIM_HandleTypeDef test_htim_left, test_htim_right;
extern GPIO_TypeDef test_gpio;
#define BOARD_HTIM_LEFT  (&test_htim_left)
#define BOARD_HTIM_RIGHT (&test_htim_right)
#define DIR_LF_PORT (&test_gpio)
#define DIR_LR_PORT (&test_gpio)
#define DIR_RF_PORT (&test_gpio)
#define DIR_RR_PORT (&test_gpio)
#define DIR_LF_PIN 1U
#define DIR_LR_PIN 2U
#define DIR_RF_PIN 4U
#define DIR_RR_PIN 8U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
#define TIM_CR1_CEN 1U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E 16U
#define TIM_EGR_UG 1U
#define TIM_BDTR_MOE 32768U
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define __HAL_TIM_SET_PRESCALER(h, value) ((h)->Instance->PSC = (value))
#define __HAL_TIM_ENABLE(h) SET_BIT((h)->Instance->CR1, TIM_CR1_CEN)
#define __HAL_TIM_MOE_ENABLE(h) SET_BIT((h)->Instance->BDTR, TIM_BDTR_MOE)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, u16 pin, u32 value);

typedef struct { u32 id; } USART_TypeDef;
/* ErrorCode mirrors the one field bsp_uart.c touches in HAL_UART_ErrorCallback:
 * real HAL latches the failing flags there and its IRQ loop refuses to re-arm
 * while it is non-zero, so the model must have it to be exercised honestly. */
typedef struct { USART_TypeDef *Instance; u32 ErrorCode; } UART_HandleTypeDef;
#define HAL_OK                   0
#define HAL_UART_ERROR_NONE      0U
#define __HAL_UART_CLEAR_NEFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_FEFLAG(h) ((void)(h))
extern UART_HandleTypeDef test_huart_host, test_huart_rc;
#define BOARD_HUART_HOST (&test_huart_host)
#define BOARD_HUART_RC (&test_huart_rc)
#define BOARD_UART_HOST_INSTANCE (test_huart_host.Instance)
#define BOARD_UART_RC_INSTANCE (test_huart_rc.Instance)
#define HAL_MAX_DELAY 0xFFFFFFFFU
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
int HAL_UART_Receive_IT(UART_HandleTypeDef *h, u8 *data, u16 size);
int HAL_UART_AbortReceive(UART_HandleTypeDef *h);
int HAL_UART_Transmit(UART_HandleTypeDef *h, u8 *data, u16 size, u32 timeout);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);

#endif
