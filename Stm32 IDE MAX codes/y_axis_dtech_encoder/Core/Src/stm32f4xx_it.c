/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f4xx_it.h"

/* =========================================================
   EXTERNAL FUNCTIONS
   ========================================================= */

extern UART_HandleTypeDef huart2;

/* Defined in main.c. It calls Remote_OnUartIRQ(). */
extern void Remote_USART1_IRQHandler(void);

/* =========================================================
   CORTEX-M4 EXCEPTION HANDLERS
   ========================================================= */

void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* =========================================================
   USART1: RS485 REMOTE JOYSTICK
   ========================================================= */

/*void USART1_IRQHandler(void)
{
    Remote_USART1_IRQHandler();
}*/

/* =========================================================
   USART2: LOCAL PC SERIAL MONITOR
   ========================================================= */

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}