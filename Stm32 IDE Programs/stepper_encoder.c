//pb8 pb9 encoder pins

#include "main.h"
#include <stdio.h>

/* Private variables --------------------------------------------------------- */

UART_HandleTypeDef huart2;

/* Encoder variables */
volatile int32_t encoder_count = 0;
static uint8_t previous_state = 0U;


/* Private function prototypes ----------------------------------------------- */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void Error_Handler(void);


/* ============================================================================
   PRINTF -> USART2
   ============================================================================ */

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


/* ============================================================================
   ENCODER UPDATE
   PB6 = A
   PB7 = B
   ============================================================================ */

static void Encoder_Update(void)
{
    uint8_t A;
    uint8_t B;
    uint8_t current_state;

    /* Read encoder A */
    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
            ? 1U : 0U;

    /* Read encoder B */
    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            ? 1U : 0U;

    /* A = bit 1, B = bit 0 */
    current_state = (uint8_t)((A << 1U) | B);

    /*
     * Quadrature decoding
     *
     * Direction 1:
     * 00 -> 01 -> 11 -> 10 -> 00
     *
     * Direction 2:
     * 00 -> 10 -> 11 -> 01 -> 00
     */

    switch ((uint8_t)((previous_state << 2U) | current_state))
    {
        /* Direction 1 */
        case 0x01:
        case 0x07:
        case 0x0E:
        case 0x08:
            encoder_count++;
            break;

        /* Direction 2 */
        case 0x02:
        case 0x0B:
        case 0x0D:
        case 0x04:
            encoder_count--;
            break;

        default:
            break;
    }

    previous_state = current_state;
}


/* ============================================================================
   MAIN
   ============================================================================ */

int main(void)
{
    uint8_t A;
    uint8_t B;

    uint32_t last_print_time = 0U;

    /* HAL initialization */
    HAL_Init();

    /* System clock */
    SystemClock_Config();

    /* GPIO */
    MX_GPIO_Init();

    /* USART2 */
    MX_USART2_UART_Init();


    /* -----------------------------------------------------
       Read initial encoder state
       ----------------------------------------------------- */

    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
            ? 1U : 0U;

    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            ? 1U : 0U;

    previous_state = (uint8_t)((A << 1U) | B);


    /* -----------------------------------------------------
       Startup message
       ----------------------------------------------------- */

    printf("\r\n");
    printf("========================================\r\n");
    printf("H9730 ENCODER TEST\r\n");
    printf("STM32F446RE\r\n");
    printf("Encoder A = PB6\r\n");
    printf("Encoder B = PB7\r\n");
    printf("GPIO SOFTWARE QUADRATURE\r\n");
    printf("USART2 = 115200 baud\r\n");
    printf("========================================\r\n");


    /* -----------------------------------------------------
       Main loop
       ----------------------------------------------------- */

    while (1)
    {
        /*
         * Continuously read encoder.
         *
         * No HAL_Delay() here because we want to
         * sample PB6/PB7 as fast as possible.
         */
        Encoder_Update();


        /*
         * Print every 200 ms.
         */
        if ((HAL_GetTick() - last_print_time) >= 200U)
        {
            printf("Encoder Count = %ld\r\n",
                   (long)encoder_count);

            last_print_time = HAL_GetTick();
        }
    }
}


/* ============================================================================
   USART2 INITIALIZATION
   ============================================================================ */

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


/* ============================================================================
   GPIO INITIALIZATION
   ============================================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();


    /* -----------------------------------------------------
       PB6 = Encoder A
       PB7 = Encoder B
       ----------------------------------------------------- */

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
     * USART2 PA2/PA3 are normally initialized by
     * HAL_UART_MspInit() in stm32f4xx_hal_msp.c.
     *
     * Do NOT add another HAL_UART_MspInit() here.
     */
}


/* ============================================================================
   SYSTEM CLOCK
   ============================================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* Power clock */
    __HAL_RCC_PWR_CLK_ENABLE();


    /* Voltage scaling */
    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE3
    );


    /* HSI */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    /* PLL */
    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;

    RCC_OscInitStruct.PLL.PLLN = 336;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ = 2;

    RCC_OscInitStruct.PLL.PLLR = 2;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }


    /* CPU / AHB / APB clocks */
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


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================================
   ERROR HANDLER
   ============================================================================ */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}