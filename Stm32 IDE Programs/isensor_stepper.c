// pb0,pb1 as gpio o/p
// usart2 and i2c1
// pul-,dir-,isensor gnd,dm556 gnd common to dc power supply gnd


#include "main.h"
#include <stdio.h>
#include <stdint.h>

/* =========================================================
   HANDLES
   ========================================================= */

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;


/* =========================================================
   INA219
   ========================================================= */

#define INA219_ADDRESS              (0x40U << 1U)

/* INA219 registers */
#define INA219_REG_CONFIG           0x00U
#define INA219_REG_SHUNT_VOLTAGE    0x01U
#define INA219_REG_CALIBRATION      0x05U

/*
 * Your 7Semi module:
 * assumed shunt = 0.1 ohm
 *
 * INA219 shunt voltage:
 * 1 bit = 10 uV
 *
 * Current = Vshunt / Rshunt
 */
#define INA219_SHUNT_RESISTOR_OHM   0.1f


/* =========================================================
   CURRENT LIMIT
   ========================================================= */

/*
 * TEST LIMIT
 *
 * Your measured current was approximately
 * 0.30 to 0.33 A.
 *
 * Therefore:
 *
 * < 0.50 A -> motor runs
 * > 0.50 A -> motor stops
 */
#define CURRENT_LIMIT_A             0.30f


/* =========================================================
   STEP TIMING
   ========================================================= */

#define STEP_HIGH_TIME_MS           5U
#define STEP_LOW_TIME_MS            4U


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

static uint8_t INA219_Init(void);
static uint8_t INA219_ReadCurrent(float *current_A);

static void I2C_Scan(void);
static void Stepper_Step(void);

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
   I2C SCAN
   ========================================================= */

static void I2C_Scan(void)
{
    uint8_t address;
    uint8_t found = 0U;

    printf("\r\n");
    printf("========================================\r\n");
    printf("I2C SCAN START\r\n");
    printf("========================================\r\n");

    for (address = 1U; address < 127U; address++)
    {
        if (HAL_I2C_IsDeviceReady(
                &hi2c1,
                (uint16_t)(address << 1U),
                2U,
                50U) == HAL_OK)
        {
            printf("I2C device found at 0x%02X\r\n", address);
            found = 1U;
        }
    }

    if (found == 0U)
    {
        printf("No I2C device found\r\n");
    }

    printf("========================================\r\n");
    printf("I2C SCAN COMPLETE\r\n");
    printf("========================================\r\n");
}


/* =========================================================
   INA219 INITIALIZATION
   ========================================================= */

static uint8_t INA219_Init(void)
{
    uint8_t data[3];

    uint16_t config = 0x399F;

    /*
     * Configure INA219:
     *
     * Bus range  = 32 V
     * PGA        = /8
     * Shunt ADC  = 12 bit
     * Bus ADC    = 12 bit
     * Continuous measurement
     */

    data[0] = INA219_REG_CONFIG;
    data[1] = (uint8_t)(config >> 8U);
    data[2] = (uint8_t)(config & 0xFFU);


    if (HAL_I2C_Master_Transmit(
            &hi2c1,
            INA219_ADDRESS,
            data,
            3U,
            100U) != HAL_OK)
    {
        return 0U;
    }


    /*
     * Wait for first conversion
     */
    HAL_Delay(10U);


    return 1U;
}


/* =========================================================
   READ CURRENT FROM SHUNT VOLTAGE
   ========================================================= */

static uint8_t INA219_ReadCurrent(float *current_A)
{
    uint8_t reg = INA219_REG_SHUNT_VOLTAGE;
    uint8_t data[2];

    int16_t raw_shunt;

    float shunt_voltage_V;


    if (current_A == NULL)
    {
        return 0U;
    }


    /* Select shunt-voltage register */
    if (HAL_I2C_Master_Transmit(
            &hi2c1,
            INA219_ADDRESS,
            &reg,
            1U,
            100U) != HAL_OK)
    {
        return 0U;
    }


    /* Read shunt-voltage register */
    if (HAL_I2C_Master_Receive(
            &hi2c1,
            INA219_ADDRESS,
            data,
            2U,
            100U) != HAL_OK)
    {
        return 0U;
    }


    /*
     * Convert received bytes to signed 16-bit value
     */
    raw_shunt =
        (int16_t)(((uint16_t)data[0] << 8U) | data[1]);


    /*
     * INA219 shunt voltage:
     *
     * 1 bit = 10 uV
     */
    shunt_voltage_V =
        (float)raw_shunt * 10.0e-6f;


    /*
     * Current = V / R
     */
    *current_A =
        shunt_voltage_V / INA219_SHUNT_RESISTOR_OHM;


    /*
     * Do not report a negative current for this
     * unidirectional supply-current application.
     */
    if (*current_A < 0.0f)
    {
        *current_A = -*current_A;
    }


    return 1U;
}


/* =========================================================
   STEPPER STEP
   ========================================================= */

static void Stepper_Step(void)
{
    /* STEP HIGH */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0,
        GPIO_PIN_SET
    );

    HAL_Delay(STEP_HIGH_TIME_MS);


    /* STEP LOW */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0,
        GPIO_PIN_RESET
    );

    HAL_Delay(STEP_LOW_TIME_MS);
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    float current_A = 0.0f;

    uint8_t ina219_ok = 0U;

    uint32_t last_print_time = 0U;


    /* HAL initialization */
    HAL_Init();


    /* System clock */
    SystemClock_Config();


    /* GPIO */
    MX_GPIO_Init();


    /* I2C1 */
    MX_I2C1_Init();


    /* USART2 */
    MX_USART2_UART_Init();


    /* -----------------------------------------------------
       Motor initial state
       ----------------------------------------------------- */

    /* STEP LOW */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0,
        GPIO_PIN_RESET
    );


    /*
     * DIR LOW = CLOCKWISE
     *
     * Confirmed by your previous motor test.
     */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_1,
        GPIO_PIN_RESET
    );


    /* Give DM556 time */
    HAL_Delay(500U);


    /* -----------------------------------------------------
       I2C scan
       ----------------------------------------------------- */

    I2C_Scan();


    /* -----------------------------------------------------
       INA219 initialization
       ----------------------------------------------------- */

    ina219_ok = INA219_Init();


    if (ina219_ok)
    {
        printf("INA219 initialization OK\r\n");
        printf("Current limit = %.3f A\r\n",
               CURRENT_LIMIT_A);
    }
    else
    {
        printf("INA219 initialization FAILED\r\n");
    }


    /* =====================================================
       MAIN LOOP
       ===================================================== */

    while (1)
    {
        /*
         * -----------------------------------------------
         * Check INA219 communication
         * -----------------------------------------------
         */
        if (ina219_ok == 0U)
        {
            /*
             * Sensor unavailable:
             * stop motor
             */
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_0,
                GPIO_PIN_RESET
            );


            /*
             * Try to reconnect
             */
            ina219_ok = INA219_Init();


            HAL_Delay(100U);

            continue;
        }


        /*
         * -----------------------------------------------
         * Read current
         * -----------------------------------------------
         */
        if (INA219_ReadCurrent(&current_A) == 0U)
        {
            /*
             * I2C read failure:
             * stop motor
             */
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_0,
                GPIO_PIN_RESET
            );


            ina219_ok = 0U;

            continue;
        }


        /*
         * -----------------------------------------------
         * CURRENT LIMIT
         * -----------------------------------------------
         */

        if (current_A <= CURRENT_LIMIT_A)
        {
            /*
             * Current is normal.
             *
             * MOTOR RUN
             */
            Stepper_Step();
        }
        else
        {
            /*
             * Current exceeded limit.
             *
             * MOTOR STOP
             */
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_0,
                GPIO_PIN_RESET
            );

            /*
             * Check again after 10 ms
             */
            HAL_Delay(10U);
        }


        /*
         * -----------------------------------------------
         * Print current every 500 ms
         * -----------------------------------------------
         */

        if ((HAL_GetTick() - last_print_time) >= 500U)
        {
            printf(
                "Current = %.3f A | Limit = %.3f A | ",
                current_A,
                CURRENT_LIMIT_A
            );


            if (current_A <= CURRENT_LIMIT_A)
            {
                printf("MOTOR = RUN\r\n");
            }
            else
            {
                printf("MOTOR = STOP\r\n");
            }


            last_print_time = HAL_GetTick();
        }
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
       PA5 = onboard LED
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


    /* -----------------------------------------------------
       PB0 = STEP
       PB1 = DIR
       ----------------------------------------------------- */

    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0 | GPIO_PIN_1,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        GPIO_PIN_0 | GPIO_PIN_1;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;


    HAL_GPIO_Init(
        GPIOB,
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


    /* Voltage scaling */
    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    /*
     * HSI = 16 MHz
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