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

#define INA219_REG_CONFIG           0x00U
#define INA219_REG_SHUNT_VOLTAGE    0x01U
#define INA219_REG_CURRENT          0x04U
#define INA219_REG_CALIBRATION      0x05U

/*
 * Assumed 7Semi INA219 shunt:
 * 0.1 ohm
 *
 * Shunt voltage LSB = 10 uV
 */
#define INA219_SHUNT_RESISTOR_OHM   0.1f

/* =========================================================
   CURRENT LIMIT
   ========================================================= */

/*
 * TEST LIMIT
 *
 * Current below 0.300 A  -> motor runs
 * Current >= 0.300 A     -> motor stops permanently
 */
#define CURRENT_LIMIT_A             0.420f


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

static uint8_t I2C_Device_Detected(void);

static uint8_t INA219_Init(void);
static uint8_t INA219_ReadCurrent(float *current_A);

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
   CHECK INA219 AT 0x40
   ========================================================= */

static uint8_t I2C_Device_Detected(void)
{
    if (HAL_I2C_IsDeviceReady(
            &hi2c1,
            INA219_ADDRESS,
            3U,
            100U) == HAL_OK)
    {
        return 1U;
    }

    return 0U;
}


/* =========================================================
   INA219 INITIALIZATION
   ========================================================= */

static uint8_t INA219_Init(void)
{
    uint8_t data[3];

    uint16_t config = 0x399F;

    /*
     * Write configuration.
     *
     * 32 V bus range
     * PGA /8
     * 12-bit shunt ADC
     * 12-bit bus ADC
     * Continuous conversion
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


    /* Convert to signed 16-bit value */
    raw_shunt =
        (int16_t)(((uint16_t)data[0] << 8U) |
                  data[1]);


    /*
     * INA219 shunt voltage:
     * 1 LSB = 10 uV
     */
    shunt_voltage_V =
        (float)raw_shunt * 10.0e-6f;


    /*
     * Current = V / R
     */
    *current_A =
        shunt_voltage_V /
        INA219_SHUNT_RESISTOR_OHM;


    /*
     * This is a unidirectional supply-current measurement.
     * Keep current positive.
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
    /*
     * STEP HIGH
     */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0,
        GPIO_PIN_SET
    );

    HAL_Delay(STEP_HIGH_TIME_MS);


    /*
     * STEP LOW
     */
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

    /*
     * 0 = normal operation
     * 1 = permanent overcurrent fault
     */
    uint8_t overcurrent_fault = 0U;

    uint32_t last_print_time = 0U;


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
       MOTOR INITIAL STATE
       ===================================================== */

    /*
     * STEP = LOW
     */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0,
        GPIO_PIN_RESET
    );


    /*
     * DIR = LOW
     *
     * LOW = CLOCKWISE
     * according to your confirmed motor test
     */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_1,
        GPIO_PIN_RESET
    );


    /*
     * Wait for DM556 direction recognition
     */
    HAL_Delay(500U);


    /* =====================================================
       I2C / INA219 DETECTION
       ===================================================== */

    printf("\r\n");
    printf("========================================\r\n");
    printf("INA219 + STEPPER CURRENT PROTECTION\r\n");
    printf("========================================\r\n");


    if (I2C_Device_Detected())
    {
        printf("I2C device found at 0x40\r\n");

        ina219_ok = INA219_Init();

        if (ina219_ok)
        {
            printf("INA219 initialization OK\r\n");
        }
        else
        {
            printf("INA219 initialization FAILED\r\n");
        }
    }
    else
    {
        printf("No INA219 found at 0x40\r\n");
        ina219_ok = 0U;
    }


    printf(
        "Current limit = %.3f A\r\n",
        CURRENT_LIMIT_A
    );


    /* =====================================================
       MAIN LOOP
       ===================================================== */

/* =====================================================
   MAIN LOOP
   ===================================================== */

while (1)
{
    /*
     * Always try to read current.
     */
    if (INA219_ReadCurrent(&current_A) == 0U)
    {
        printf("INA219 READ ERROR\r\n");

        /*
         * Stop motor permanently because sensor
         * communication has failed.
         */
        HAL_GPIO_WritePin(
            GPIOB,
            GPIO_PIN_0,
            GPIO_PIN_RESET
        );

        overcurrent_fault = 1U;

        HAL_Delay(500);

        continue;
    }


    /*
     * Print current continuously, whether motor is
     * running or already stopped.
     */
    printf(
        "Current = %.3f A | Limit = %.3f A | ",
        current_A,
        CURRENT_LIMIT_A
    );


    /*
     * Once fault has happened, NEVER restart motor.
     */
    if (overcurrent_fault)
    {
        HAL_GPIO_WritePin(
            GPIOB,
            GPIO_PIN_0,
            GPIO_PIN_RESET
        );

        printf("MOTOR = STOP | FAULT LATCHED\r\n");

        HAL_Delay(500);

        continue;
    }


    /*
     * Check current limit.
     */
    if (current_A >= CURRENT_LIMIT_A)
    {
        /*
         * Overcurrent detected.
         */
        HAL_GPIO_WritePin(
            GPIOB,
            GPIO_PIN_0,
            GPIO_PIN_RESET
        );

        overcurrent_fault = 1U;

        printf("MOTOR = STOP | OVERCURRENT FAULT\r\n");

        HAL_Delay(500);

        continue;
    }


    /*
     * Current is below limit.
     * Motor can rotate.
     */
    printf("MOTOR = RUN\r\n");

    Stepper_Step();
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


    /*
     * PB0 = STEP
     * PB1 = DIR
     */
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


    /* Power configuration */
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


    /* Enable OverDrive */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * CPU / AHB / APB
     */
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