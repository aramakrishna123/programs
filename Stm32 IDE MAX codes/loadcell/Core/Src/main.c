#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

UART_HandleTypeDef huart2;

/* HX711 */
#define HX711_DT_PORT    GPIOC
#define HX711_DT_PIN     GPIO_PIN_2

#define HX711_SCK_PORT   GPIOC
#define HX711_SCK_PIN    GPIO_PIN_3

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

static void UART_SendString(const char *str);
static int32_t HX711_Read(void);
static int32_t HX711_Read_Average(uint8_t samples);
static uint8_t HX711_Ready(void);


/* =========================================================
   UART
   ========================================================= */

static void UART_SendString(const char *str)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)str,
        strlen(str),
        HAL_MAX_DELAY
    );
}


/* =========================================================
   HX711 READY
   ========================================================= */

static uint8_t HX711_Ready(void)
{
    return (
        HAL_GPIO_ReadPin(
            HX711_DT_PORT,
            HX711_DT_PIN
        ) == GPIO_PIN_RESET
    );
}


/* =========================================================
   HX711 READ
   ========================================================= */

static int32_t HX711_Read(void)
{
    uint32_t data = 0;

    /* Wait until data is ready */

    while (!HX711_Ready())
    {
    }


    /* Read 24 bits */

    for (int i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(
            HX711_SCK_PORT,
            HX711_SCK_PIN,
            GPIO_PIN_SET
        );

        data <<= 1;

        if (HAL_GPIO_ReadPin(
                HX711_DT_PORT,
                HX711_DT_PIN) == GPIO_PIN_SET)
        {
            data |= 1;
        }

        HAL_GPIO_WritePin(
            HX711_SCK_PORT,
            HX711_SCK_PIN,
            GPIO_PIN_RESET
        );
    }


    /*
     * 25th pulse:
     * Channel A, Gain 128
     */

    HAL_GPIO_WritePin(
        HX711_SCK_PORT,
        HX711_SCK_PIN,
        GPIO_PIN_SET
    );

    HAL_GPIO_WritePin(
        HX711_SCK_PORT,
        HX711_SCK_PIN,
        GPIO_PIN_RESET
    );


    /* Convert 24-bit signed value to 32-bit */

    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}


/* =========================================================
   AVERAGE
   ========================================================= */

static int32_t HX711_Read_Average(uint8_t samples)
{
    int64_t sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        sum += HX711_Read();
    }

    return (int32_t)(sum / samples);
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_USART2_UART_Init();


    char msg[100];

    int32_t raw;
    int32_t tare;
    int32_t value;


    UART_SendString("\r\n");
    UART_SendString("================================\r\n");
    UART_SendString("     HX711 LOAD CELL TEST\r\n");
    UART_SendString("================================\r\n");


    UART_SendString("UART OK\r\n");

    UART_SendString("Checking HX711...\r\n");


    if (HX711_Ready())
    {
        UART_SendString("HX711 READY\r\n");
    }
    else
    {
        UART_SendString("HX711 NOT READY\r\n");
    }


    /* =====================================================
       TARE
       ===================================================== */

    UART_SendString("\r\n");
    UART_SendString("Remove all weight from load cell.\r\n");
    UART_SendString("Tare in 3 seconds...\r\n");

    HAL_Delay(3000);


    UART_SendString("Reading tare...\r\n");


    tare = HX711_Read_Average(10);


    sprintf(
        msg,
        "Tare = %ld\r\n",
        (long)tare
    );

    UART_SendString(msg);


    UART_SendString("--------------------------------\r\n");


    /* =====================================================
       CONTINUOUS LOAD CELL READING
       ===================================================== */

   while (1)
{
    raw = HX711_Read_Average(10);

    value = raw - tare;

    /* Remove small zero noise */

    if (value > -200 && value < 200)
    {
        value = 0;
    }

    sprintf(
        msg,
        "RAW = %ld   LOAD = %ld\r\n",
        (long)raw,
        (long)value
    );

    UART_SendString(msg);

    HAL_Delay(500);
}
}


/* =========================================================
   SYSTEM CLOCK
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;

    RCC_OscInitStruct.PLL.PLLN = 336;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ = 7;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


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


/* =========================================================
   USART2
   ========================================================= */

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


/* =========================================================
   GPIO
   ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();


    /* HX711 SCK = PC3 */

    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_3,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        GPIO_PIN_3;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /* HX711 DT = PC2 */

    GPIO_InitStruct.Pin =
        GPIO_PIN_2;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );
}


/* =========================================================
   ERROR HANDLER
   ========================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}