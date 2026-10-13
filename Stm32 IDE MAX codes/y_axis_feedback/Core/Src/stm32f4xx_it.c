/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_it.h"


/* =========================================================
   CORTEX-M4 HANDLERS
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


/* =========================================================
   SYSTICK
   ========================================================= */

void SysTick_Handler(void)
{
    HAL_IncTick();
}


/* =========================================================
   ENCODER INTERRUPT
   PB8 = EXTI8
   PB9 = EXTI9
   ========================================================= */

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(
        GPIO_PIN_8
    );

    HAL_GPIO_EXTI_IRQHandler(
        GPIO_PIN_9
    );
}


/* =========================================================
   USER BUTTON
   ========================================================= */

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(
        USER_BUTTON_PIN
    );
}