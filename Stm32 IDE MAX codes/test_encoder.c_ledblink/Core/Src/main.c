#include "main.h"
#include "encoder.h"
#include <stdio.h>
#include <stdint.h>

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


/* Redirect printf to USART2 */
int _write(int file, char *ptr, int len)
{
    (void)file;

    if (HAL_UART_Transmit(&huart2,
                          (uint8_t *)ptr,
                          (uint16_t)len,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }

    return len;
}


int main(void)
{
    uint32_t last_print_time = 0U;
    uint32_t last_led_time = 0U;

    int32_t position_um;
    int32_t position_mm;
    int32_t fraction_um;

    uint8_t A;
    uint8_t B;

    /* HAL initialization */
    HAL_Init();

    /* System clock */
    SystemClock_Config();

    /* GPIO initialization */
    MX_GPIO_Init();

    /* USART2 initialization */
    MX_USART2_UART_Init();

    /* Encoder initialization */
    Encoder_Init();

    while (1)
    {
        /*
         * IMPORTANT:
         * Encoder_Update() runs continuously.
         * No HAL_Delay() here.
         */
        Encoder_Update();


        /* -------------------------------------------------
           Blink onboard LED every 500 ms
           ------------------------------------------------- */
        if ((HAL_GetTick() - last_led_time) >= 500U)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

            last_led_time = HAL_GetTick();
        }


        /* -------------------------------------------------
           Print encoder values every 200 ms
           ------------------------------------------------- */
        if ((HAL_GetTick() - last_print_time) >= 200U)
        {
            /* Read encoder A */
            A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
                    ? 1U : 0U;

            /* Read encoder B */
            B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
                    ? 1U : 0U;


            /* Get position in micrometers */
            position_um = Encoder_GetPositionUM();

            /* Convert micrometers to millimeters */
            position_mm = position_um / 1000;

            fraction_um = position_um % 1000;

            /*
             * Make fractional part positive for negative positions
             */
            if (fraction_um < 0)
            {
                fraction_um = -fraction_um;
            }


            /* Print A, B and position */
            printf("A=%d  B=%d  Position=%ld.%03ld mm\r\n",
                   A,
                   B,
                   (long)position_mm,
                   (long)fraction_um);


            last_print_time = HAL_GetTick();
        }
    }
}


/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* Enable power interface clock */
    __HAL_RCC_PWR_CLK_ENABLE();


    /* Voltage scaling */
    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE3
    );


    /* HSI configuration */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    /* PLL configuration */
    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;

    RCC_OscInitStruct.PLL.PLLN = 336;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ = 2;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    /* CPU, AHB and APB clock configuration */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200;

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;


    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* -------------------------------------------------
       PA5 = Nucleo onboard LED
       ------------------------------------------------- */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_5,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        GPIO_PIN_5;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* -------------------------------------------------
       PB8 = Encoder A
       PB9 = Encoder B
       ------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_8 | GPIO_PIN_9;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );


    /*
     * PA2/PA3 for USART2 are configured by
     * HAL_UART_MspInit() in stm32f4xx_hal_msp.c.
     *
     * Do not create another HAL_UART_MspInit()
     * in this file.
     */
}


/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}