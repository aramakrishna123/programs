#include "main.h"
#include "isensor.h"
#include <stdio.h>
#include <stdint.h>


/* =========================================================
   HANDLES
   ========================================================= */

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

void Error_Handler(void);


/* =========================================================
   PRINTF -> USART2
   ========================================================= */

int _write(int file, char *ptr, int len)
{
    (void)file;

    if (HAL_UART_Transmit(
            &huart2,
            (uint8_t *)ptr,
            (uint16_t)len,
            HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }

    return len;
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    float current_A = 0.0f;
    float voltage_V = 0.0f;
    float power_W = 0.0f;

    uint8_t sensor_ok = 0U;

    /*
     * 0 = normal operation
     * 1 = current limit reached
     *
     * Once set to 1, it never returns to 0
     * until MCU reset.
     */
    uint8_t current_fault = 0U;

    uint32_t last_blink_time = 0U;
    uint8_t led_state = 0U;


    /* =====================================================
       HAL
       ===================================================== */

    HAL_Init();


    /* =====================================================
       CLOCK
       ===================================================== */

    SystemClock_Config();


    /* =====================================================
       GPIO
       ===================================================== */

    MX_GPIO_Init();


    /* =====================================================
       I2C1
       ===================================================== */

    MX_I2C1_Init();


    /* =====================================================
       USART2
       ===================================================== */

    MX_USART2_UART_Init();


    /* =====================================================
       INA219
       ===================================================== */

    printf("\r\n");
    printf("========================================\r\n");
    printf("INA219 + ONBOARD LED TEST\r\n");
    printf("========================================\r\n");


    if (INA219_IsConnected(&hi2c1))
    {
        printf("I2C device found at 0x40\r\n");

        if (INA219_Init(&hi2c1))
        {
            sensor_ok = 1U;

            printf("INA219 initialization OK\r\n");
        }
        else
        {
            printf("INA219 initialization FAILED\r\n");
        }
    }
    else
    {
        printf("INA219 NOT DETECTED\r\n");
    }


    printf(
        "Current limit = %.3f A\r\n",
        INA219_CURRENT_LIMIT_A
    );


    /* =====================================================
       MAIN LOOP
       ===================================================== */

    while (1)
    {
        /* -------------------------------------------------
           SENSOR COMMUNICATION CHECK
           ------------------------------------------------- */

        if (!sensor_ok)
        {
            /*
             * Sensor unavailable.
             *
             * LED OFF permanently.
             */
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_5,
                GPIO_PIN_RESET
            );

            continue;
        }


        /* -------------------------------------------------
           READ CURRENT
           ------------------------------------------------- */

        if (!INA219_ReadCurrent(
                &hi2c1,
                &current_A))
        {
            /*
             * Sensor communication failure.
             *
             * Turn LED OFF permanently.
             */
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_5,
                GPIO_PIN_RESET
            );

            current_fault = 1U;

            printf(
                "INA219 READ ERROR | LED = OFF\r\n"
            );

            continue;
        }


        /* -------------------------------------------------
           READ VOLTAGE AND POWER
           ------------------------------------------------- */

        INA219_ReadBusVoltage(
            &hi2c1,
            &voltage_V
        );

        INA219_ReadPower(
            &hi2c1,
            &power_W
        );


        /* -------------------------------------------------
           CURRENT LIMIT
           ------------------------------------------------- */

        if (!current_fault &&
            current_A >= INA219_CURRENT_LIMIT_A)
        {
            /*
             * Current reached limit.
             */
            current_fault = 1U;

            /*
             * LED OFF immediately.
             */
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_5,
                GPIO_PIN_RESET
            );

            printf(
                "\r\n*** CURRENT LIMIT REACHED ***\r\n"
            );
        }


        /* -------------------------------------------------
           LED OPERATION
           ------------------------------------------------- */

        if (current_fault)
        {
            /*
             * Fault:
             * LED remains OFF.
             */
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_5,
                GPIO_PIN_RESET
            );
        }
        else
        {
            /*
             * Normal:
             * Blink onboard LED every 500 ms.
             */
            if ((HAL_GetTick() - last_blink_time) >= 500U)
            {
                led_state = !led_state;

                HAL_GPIO_WritePin(
                    GPIOA,
                    GPIO_PIN_5,
                    led_state ?
                    GPIO_PIN_SET :
                    GPIO_PIN_RESET
                );

                last_blink_time =
                    HAL_GetTick();
            }
        }


        /* -------------------------------------------------
           SERIAL OUTPUT
           ------------------------------------------------- */

        if (current_fault)
        {
            printf(
                "Voltage = %.3f V | "
                "Current = %.3f A | "
                "Power = %.3f W | "
                "Limit = %.3f A | "
                "LED = OFF | FAULT LATCHED\r\n",
                voltage_V,
                current_A,
                power_W,
                INA219_CURRENT_LIMIT_A
            );
        }
        else
        {
            printf(
                "Voltage = %.3f V | "
                "Current = %.3f A | "
                "Power = %.3f W | "
                "Limit = %.3f A | "
                "LED = BLINK\r\n",
                voltage_V,
                current_A,
                power_W,
                INA219_CURRENT_LIMIT_A
            );
        }


        /*
         * Slow down serial output.
         */
        HAL_Delay(500U);
    }
}


/* =========================================================
   I2C1 INITIALIZATION
   ========================================================= */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed =
        100000U;

    hi2c1.Init.DutyCycle =
        I2C_DUTYCYCLE_2;

    hi2c1.Init.OwnAddress1 =
        0U;

    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c1.Init.OwnAddress2 =
        0U;

    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;


    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   USART2 INITIALIZATION
   ========================================================= */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate =
        115200U;

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
   GPIO INITIALIZATION
   ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* -----------------------------------------------------
       PA5 = NUCLEO-F446RE ONBOARD LED
       ----------------------------------------------------- */

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
}


/* =========================================================
   SYSTEM CLOCK
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* Power */
    __HAL_RCC_PWR_CLK_ENABLE();


    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    /*
     * HSI = 16 MHz
     *
     * PLLM = 16
     * PLLN = 360
     * PLLP = 2
     *
     * SYSCLK = 180 MHz
     */

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

    RCC_OscInitStruct.PLL.PLLM =
        16U;

    RCC_OscInitStruct.PLL.PLLN =
        360U;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ =
        7U;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    /* OverDrive */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }


    /* Clock configuration */

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
        RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
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