#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

/* USER CODE BEGIN EFP */

void Error_Handler(void);

/* Remote joystick USART1 interrupt handler function */

/* USER CODE END EFP */


/* Private defines -----------------------------------------------------------*/

/* USART2 - PC Serial Monitor */
#define USART_TX_Pin        GPIO_PIN_2
#define USART_TX_GPIO_Port  GPIOA

#define USART_RX_Pin        GPIO_PIN_3
#define USART_RX_GPIO_Port  GPIOA


/* SWD */
#define TMS_Pin             GPIO_PIN_13
#define TMS_GPIO_Port       GPIOA

#define TCK_Pin             GPIO_PIN_14
#define TCK_GPIO_Port       GPIOA

#define SWO_Pin             GPIO_PIN_3
#define SWO_GPIO_Port       GPIOB


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */