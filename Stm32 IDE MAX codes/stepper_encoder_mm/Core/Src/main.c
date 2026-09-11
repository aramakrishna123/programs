//pb8 pb9 encoder pins


#include "main.h"
#include <stdio.h>

/* Private variables --------------------------------------------------------- */

UART_HandleTypeDef huart2;

/* Encoder */
volatile int32_t encoder_count = 0;
static uint8_t previous_state = 0U;

/* Encoder calibration */
#define ENCODER_COUNTS_PER_INCH    180.0f
#define MM_PER_INCH                25.4f
#define MM_PER_COUNT               (MM_PER_INCH / ENCODER_COUNTS_PER_INCH)

/* Private function prototypes ----------------------------------------------- */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void Error_Handler(void);


/* USER CODE BEGIN 0 */
#define ENCODER_LINES_PER_INCH    180L
#define QUADRATURE_MULTIPLIER     4L

#define ENCODER_COUNTS_PER_INCH \
        (ENCODER_LINES_PER_INCH * QUADRATURE_MULTIPLIER)

#define MM_PER_INCH_X1000         25400L
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


/* Read and decode quadrature encoder */
static void Encoder_Update(void)
{
    uint8_t A;
    uint8_t B;
    uint8_t current_state;

    /* PB8 = Encoder A */
    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;

    /* PB9 = Encoder B */
    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;

    /* A = bit 1, B = bit 0 */
    current_state = (uint8_t)((A << 1U) | B);

    /*
     * Quadrature decoding
     *
     * 00 -> 01 -> 11 -> 10 -> 00
     * one direction
     *
     * 00 -> 10 -> 11 -> 01 -> 00
     * opposite direction
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

/* USER CODE END 0 */


/* Main ---------------------------------------------------------------------- */

int main(void)
{
    uint8_t A;
    uint8_t B;
    uint32_t last_print_time = 0U;

    /* HAL */
    HAL_Init();

    /* Clock */
    SystemClock_Config();

    /* GPIO */
    MX_GPIO_Init();

    /* USART2 */
    MX_USART2_UART_Init();

    /* Get initial encoder state */
    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1U : 0U;
    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) ? 1U : 0U;

    previous_state = (uint8_t)((A << 1U) | B);

    /* Startup message */
    printf("\r\n");
    printf("========================================\r\n");
    printf("H9730 ENCODER TEST\r\n");
    printf("STM32F446RE\r\n");
    printf("Encoder A       = PB8\r\n");
    printf("Encoder B       = PB9\r\n");
    printf("Counts/Inch     = 180\r\n");
    printf("MM/Count        = %.6f\r\n", MM_PER_COUNT);
    printf("USART2 Baud     = 115200\r\n");
    printf("========================================\r\n");

  while (1)
{
    Encoder_Update();

if ((HAL_GetTick() - last_print_time) >= 200U)
{
    int32_t position_um;
    int32_t whole_mm;
    int32_t fraction_um;

    /*
     * 180 lines/inch × 4 quadrature = 720 counts/inch
     *
     * 1 inch = 25400 micrometers
     */
    position_um =
        (encoder_count * 25400L) / 720L;

    whole_mm = position_um / 1000L;
    fraction_um = position_um % 1000L;

    if (fraction_um < 0)
    {
        fraction_um = -fraction_um;
    }

    printf("A=%d  B=%d  Count=%ld  Position=%ld.%03ld mm\r\n",
           (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1 : 0,
           (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) ? 1 : 0,
           (long)encoder_count,
           (long)whole_mm,
           (long)fraction_um);

    last_print_time = HAL_GetTick();
}
}
}


/* System Clock Configuration ----------------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}


/* USART2 Initialization ---------------------------------------------------- */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}


/* GPIO Initialization ------------------------------------------------------ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* PB8 = Encoder A
       PB9 = Encoder B */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*
     * PA2/PA3 for USART2 are initialized by
     * HAL_UART_MspInit() in stm32f4xx_hal_msp.c.
     */
}


/* Error Handler ------------------------------------------------------------- */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}