#include "main.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */
/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */


/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
    /* USER CODE BEGIN MspInit 0 */
    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(
        NVIC_PRIORITYGROUP_4
    );

    /* USER CODE BEGIN MspInit 1 */
    /* USER CODE END MspInit 1 */
}


/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* =====================================================
       USART1
       SERVO RS422
       ===================================================== */

    if (huart->Instance == USART1)
    {
        /* Peripheral clock */
        __HAL_RCC_USART1_CLK_ENABLE();

        /* GPIO clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /*
         * USART1 GPIO:
         *
         * PA9  = USART1_TX
         * PA10 = USART1_RX
         */
        GPIO_InitStruct.Pin =
            GPIO_PIN_9 |
            GPIO_PIN_10;

        GPIO_InitStruct.Mode =
            GPIO_MODE_AF_PP;

        GPIO_InitStruct.Pull =
            GPIO_NOPULL;

        GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_VERY_HIGH;

        GPIO_InitStruct.Alternate =
            GPIO_AF7_USART1;

        HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct
        );

        /*
         * USART1 interrupt.
         *
         * Servo communication in main.c is
         * currently blocking, but keeping the
         * IRQ enabled is acceptable.
         */
        HAL_NVIC_SetPriority(
            USART1_IRQn,
            0,
            0
        );

        HAL_NVIC_EnableIRQ(
            USART1_IRQn
        );
    }


    /* =====================================================
       USART2
       PC / USB SERIAL
       ===================================================== */

    else if (huart->Instance == USART2)
    {
        /* Peripheral clock */
        __HAL_RCC_USART2_CLK_ENABLE();

        /* GPIO clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /*
         * USART2 GPIO:
         *
         * PA2 = USART2_TX
         * PA3 = USART2_RX
         */
        GPIO_InitStruct.Pin =
            GPIO_PIN_2 |
            GPIO_PIN_3;

        GPIO_InitStruct.Mode =
            GPIO_MODE_AF_PP;

        GPIO_InitStruct.Pull =
            GPIO_NOPULL;

        GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_VERY_HIGH;

        GPIO_InitStruct.Alternate =
            GPIO_AF7_USART2;

        HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct
        );

        /*
         * No USART2 interrupt required.
         *
         * No hardware flow control:
         * PA1 is used separately as DTech DE/RE.
         */
    }


    /* =====================================================
       USART3
       DTECH RS422
       ===================================================== */

    else if (huart->Instance == USART3)
    {
        /* Peripheral clock */
        __HAL_RCC_USART3_CLK_ENABLE();

        /* GPIO clock */
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /*
         * USART3 GPIO:
         *
         * PC10 = USART3_TX
         * PC11 = USART3_RX
         */
        GPIO_InitStruct.Pin =
            GPIO_PIN_10 |
            GPIO_PIN_11;

        GPIO_InitStruct.Mode =
            GPIO_MODE_AF_PP;

        GPIO_InitStruct.Pull =
            GPIO_NOPULL;

        GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_VERY_HIGH;

        GPIO_InitStruct.Alternate =
            GPIO_AF7_USART3;

        HAL_GPIO_Init(
            GPIOC,
            &GPIO_InitStruct
        );

        /*
         * USART3 interrupt required for
         * DTech RX.
         */
        HAL_NVIC_SetPriority(
            USART3_IRQn,
            5,
            0
        );

        HAL_NVIC_EnableIRQ(
            USART3_IRQn
        );
    }
}


/**
  * @brief UART MSP De-Initialization
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    /* =====================================================
       USART1
       ===================================================== */

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();

        /*
         * PA9  = USART1_TX
         * PA10 = USART1_RX
         */
        HAL_GPIO_DeInit(
            GPIOA,
            GPIO_PIN_9 |
            GPIO_PIN_10
        );

        HAL_NVIC_DisableIRQ(
            USART1_IRQn
        );
    }


    /* =====================================================
       USART2
       ===================================================== */

    else if (huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_DISABLE();

        /*
         * PA2 = USART2_TX
         * PA3 = USART2_RX
         */
        HAL_GPIO_DeInit(
            GPIOA,
            GPIO_PIN_2 |
            GPIO_PIN_3
        );
    }


    /* =====================================================
       USART3
       ===================================================== */

    else if (huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();

        /*
         * PC10 = USART3_TX
         * PC11 = USART3_RX
         */
        HAL_GPIO_DeInit(
            GPIOC,
            GPIO_PIN_10 |
            GPIO_PIN_11
        );

        HAL_NVIC_DisableIRQ(
            USART3_IRQn
        );
    }
}