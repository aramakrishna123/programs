/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"

#include <stdint.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END USER CODE BEGIN PM */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */


/* External variables --------------------------------------------------------*/

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;


/* USER CODE BEGIN EV */

/*
 * DTech RX byte handler implemented in main.c.
 */
extern void DTech_ReceiveByte(uint8_t byte);

/* USER CODE END EV */


/******************************************************************************/ 
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
    /* USER CODE END NonMaskableInt_IRQn 0 */

    while (1)
    {
    }

    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    /* USER CODE END NonMaskableInt_IRQn 1 */
}


/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */
    /* USER CODE END HardFault_IRQn 0 */

    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}


/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */
    /* USER CODE END MemoryManagement_IRQn 0 */

    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}


/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */
    /* USER CODE END BusFault_IRQn 0 */

    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}


/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */
    /* USER CODE END UsageFault_IRQn 0 */

    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}


/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */
    /* USER CODE END SVCall_IRQn 0 */

    /* USER CODE BEGIN SVCall_IRQn 1 */
    /* USER CODE END SVCall_IRQn 1 */
}


/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */
    /* USER CODE END DebugMonitor_IRQn 0 */

    /* USER CODE BEGIN DebugMonitor_IRQn 1 */
    /* USER CODE END DebugMonitor_IRQn 1 */
}


/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */
    /* USER CODE END PendSV_IRQn 0 */

    /* USER CODE BEGIN PendSV_IRQn 1 */
    /* USER CODE END PendSV_IRQn 1 */
}


/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */
    /* USER CODE END SysTick_IRQn 0 */

    HAL_IncTick();

    /* USER CODE BEGIN SysTick_IRQn 1 */
    /* USER CODE END SysTick_IRQn 1 */
}


/******************************************************************************/ 
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/


/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    /*
     * USART1 is the servo RS422 interface.
     *
     * Servo communication in main.c is blocking
     * and does not use HAL RX interrupt reception.
     *
     * Keep the HAL IRQ handler here because USART1
     * is generated/configured by CubeMX.
     */
    HAL_UART_IRQHandler(&huart1);
}


/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    uint32_t status;


    /*
     * Read USART3 status.
     *
     * USART3 is the DTech RS422 interface.
     */
    status = USART3->SR;


    /* ------------------------------------------------------
       UART ERROR HANDLING
       ------------------------------------------------------ */

    if (status &
        (USART_SR_ORE |
         USART_SR_NE  |
         USART_SR_FE))
    {
        /*
         * Reading DR clears the UART error condition.
         */
        volatile uint8_t dummy;

        dummy = (uint8_t)USART3->DR;

        (void)dummy;

        return;
    }


    /* ------------------------------------------------------
       RECEIVE DATA
       ------------------------------------------------------ */

    if (status & USART_SR_RXNE)
    {
        uint8_t byte;


        /*
         * Read received DTech byte.
         */
        byte =
            (uint8_t)USART3->DR;


        /*
         * Pass byte to main.c RX buffer.
         */
        DTech_ReceiveByte(
            byte
        );
    }
}


/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */
    /* USER CODE END EXTI15_10_IRQn 0 */

    HAL_GPIO_EXTI_IRQHandler(
        USER_BUTTON_PIN
    );

    /* USER CODE BEGIN EXTI15_10_IRQn 1 */
    /* USER CODE END EXTI15_10_IRQn 1 */
}


/* USER CODE BEGIN 1 */
/* USER CODE END 1 */