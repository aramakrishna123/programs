#include "main.h"

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};


if (huart->Instance == USART2)
{
    /* Enable USART2 clock */
    __HAL_RCC_USART2_CLK_ENABLE();

    /* Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * PA2 = USART2_TX
     * PA3 = USART2_RX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
if (huart->Instance == USART2)
{
__HAL_RCC_USART2_CLK_DISABLE();


    HAL_GPIO_DeInit(
        GPIOA,
        GPIO_PIN_2 | GPIO_PIN_3
    );
}


}
