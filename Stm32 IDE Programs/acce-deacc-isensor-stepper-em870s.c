#include "main.h"
#include "isensor.h"
#include <stdio.h>

/* =========================================================
   PERIPHERAL HANDLES
   ========================================================= */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;

/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

static void Motor_SetFrequency(uint32_t frequency_hz);
static void Motor_Start(uint32_t frequency_hz);
static void Motor_Stop(void);
static void Motor_Accelerate(void);
static void Motor_Decelerate(void);
static void Wait_For_Current_Clear(void);
static void Run_At_Max_Speed(void);
static void Print_Status(float current_A,
                         float voltage_V,
                         float power_W);

/* =========================================================
   MOTOR SETTINGS
   ========================================================= */
#define MOTOR_START_SPEED       100U
#define MOTOR_MAX_SPEED        2000U
#define SPEED_STEP               20U
#define RAMP_UPDATE_MS           20U

/* Motor stops when current reaches this value */
#define CURRENT_LIMIT_A        0.500f

/* Motor restarts when current drops below this value */
#define CURRENT_RESTART_A      0.450f

#define CURRENT_CHECK_MS         200U

/* PB1 direction
   RESET = CLOCKWISE in your existing setup */
#define CLOCKWISE_LEVEL GPIO_PIN_RESET

/* TIM3 counter frequency = 1 MHz */
#define TIMER_COUNTER_HZ 1000000UL

/* =========================================================
   MOTOR STATE
   ========================================================= */
volatile uint8_t motor_running = 0U;
volatile uint32_t motor_frequency = 0U;

/* =========================================================
   PRINTF -> USART2
   ========================================================= */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)ptr,
                      len,
                      HAL_MAX_DELAY);

    return len;
}

/* =========================================================
   TIM3 CALLBACK
   PB0 = STEP/PUL
   ========================================================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        if (motor_running)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB,
                              GPIO_PIN_0,
                              GPIO_PIN_RESET);
        }
    }
}

/* =========================================================
   SET STEP FREQUENCY
   ========================================================= */
static void Motor_SetFrequency(uint32_t frequency_hz)
{
    uint32_t half_period_us;

    if (frequency_hz == 0U)
    {
        return;
    }

    /*
       TIM3 counter = 1 MHz.

       PB0 toggles every interrupt.

       Therefore:
       interrupt frequency = 2 x step frequency
    */
    half_period_us =
        TIMER_COUNTER_HZ / (frequency_hz * 2U);

    if (half_period_us < 1U)
    {
        half_period_us = 1U;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim3,
                             half_period_us - 1U);

    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    motor_frequency = frequency_hz;
}

/* =========================================================
   START MOTOR
   ========================================================= */
static void Motor_Start(uint32_t frequency_hz)
{
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_1,
                      CLOCKWISE_LEVEL);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);

    Motor_SetFrequency(frequency_hz);

    motor_running = 1U;

    HAL_TIM_Base_Start_IT(&htim3);

    printf("MOTOR STARTED - CLOCKWISE\r\n");
}

/* =========================================================
   STOP MOTOR
   ========================================================= */
static void Motor_Stop(void)
{
    motor_running = 0U;

    HAL_TIM_Base_Stop_IT(&htim3);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);

    motor_frequency = 0U;
}

/* =========================================================
   ACCELERATION
   100 -> 2000 Hz
   ========================================================= */
static void Motor_Accelerate(void)
{
    uint32_t speed = MOTOR_START_SPEED;

    Motor_Start(speed);

    while (speed < MOTOR_MAX_SPEED)
    {
        float current_A = 0.0f;

        HAL_Delay(RAMP_UPDATE_MS);

        if (INA219_ReadCurrent(&current_A) != HAL_OK)
        {
            printf("INA219 CURRENT READ ERROR\r\n");
            printf("CONTROLLED DECELERATION\r\n");

            Motor_Decelerate();
            return;
        }

        printf("ACCEL  Speed = %lu Hz, Current = %.3f A\r\n",
               (unsigned long)speed,
               current_A);

        /* Current limit */
        if (current_A >= CURRENT_LIMIT_A)
        {
            printf("\r\n");
            printf("CURRENT LIMIT REACHED!\r\n");
            printf("Current = %.3f A\r\n", current_A);
            printf("Limit   = %.3f A\r\n", CURRENT_LIMIT_A);
            printf("CONTROLLED DECELERATION\r\n");

            Motor_Decelerate();
            return;
        }

        speed += SPEED_STEP;

        if (speed > MOTOR_MAX_SPEED)
        {
            speed = MOTOR_MAX_SPEED;
        }

        Motor_SetFrequency(speed);
    }

    printf("MOTOR REACHED MAX SPEED = %lu Hz\r\n",
           (unsigned long)MOTOR_MAX_SPEED);
}

/* =========================================================
   CONTROLLED DECELERATION
   CURRENT SPEED -> 100 -> STOP
   ========================================================= */
static void Motor_Decelerate(void)
{
    uint32_t speed = motor_frequency;

    if (!motor_running)
    {
        return;
    }

    printf("DECELERATION START\r\n");

    while (speed > MOTOR_START_SPEED)
    {
        HAL_Delay(RAMP_UPDATE_MS);

        if (speed > SPEED_STEP)
        {
            speed -= SPEED_STEP;
        }
        else
        {
            speed = MOTOR_START_SPEED;
        }

        if (speed < MOTOR_START_SPEED)
        {
            speed = MOTOR_START_SPEED;
        }

        Motor_SetFrequency(speed);

        printf("DECEL  Speed = %lu Hz\r\n",
               (unsigned long)speed);
    }

    HAL_Delay(RAMP_UPDATE_MS);

    Motor_Stop();

    printf("MOTOR STOPPED\r\n");

    printf("WAITING FOR CURRENT TO FALL BELOW %.3f A\r\n",
           CURRENT_RESTART_A);
}

/* =========================================================
   WAIT FOR CURRENT TO FALL BELOW RESTART THRESHOLD
   ========================================================= */
static void Wait_For_Current_Clear(void)
{
    float current_A = 0.0f;

    while (1)
    {
        if (INA219_ReadCurrent(&current_A) != HAL_OK)
        {
            printf("INA219 CURRENT READ ERROR WHILE WAITING\r\n");

            HAL_Delay(CURRENT_CHECK_MS);

            continue;
        }

        printf("MOTOR STOPPED | Current = %.3f A | "
               "Restart threshold = %.3f A\r\n",
               current_A,
               CURRENT_RESTART_A);

        if (current_A < CURRENT_RESTART_A)
        {
            printf("\r\n");
            printf("CURRENT CLEARED\r\n");
            printf("Current = %.3f A\r\n", current_A);
            printf("AUTOMATIC RESTART\r\n");
            printf("\r\n");

            return;
        }

        HAL_Delay(CURRENT_CHECK_MS);
    }
}

/* =========================================================
   RUN AT MAX SPEED
   ========================================================= */
static void Run_At_Max_Speed(void)
{
    float current_A = 0.0f;
    float voltage_V = 0.0f;
    float power_W = 0.0f;

    uint32_t last_print_time = 0U;

    printf("MOTOR RUNNING CONTINUOUSLY\r\n");

    last_print_time = HAL_GetTick();

    while (motor_running)
    {
        if (INA219_ReadCurrent(&current_A) != HAL_OK)
        {
            printf("INA219 CURRENT READ FAILED\r\n");
            printf("CONTROLLED DECELERATION\r\n");

            Motor_Decelerate();
            return;
        }

        /* Current limit */
        if (current_A >= CURRENT_LIMIT_A)
        {
            printf("\r\n");
            printf("========================================\r\n");
            printf("CURRENT LIMIT REACHED\r\n");
            printf("Current = %.3f A\r\n", current_A);
            printf("Limit   = %.3f A\r\n", CURRENT_LIMIT_A);
            printf("========================================\r\n");
            printf("CONTROLLED DECELERATION\r\n");

            Motor_Decelerate();

            return;
        }

        /* Status every 500 ms */
        if ((HAL_GetTick() - last_print_time) >= 500U)
        {
            last_print_time = HAL_GetTick();

            if (INA219_ReadBusVoltage(&voltage_V)
                != HAL_OK)
            {
                voltage_V = 0.0f;
            }

            if (INA219_ReadPower(&power_W)
                != HAL_OK)
            {
                power_W = 0.0f;
            }

            Print_Status(current_A,
                         voltage_V,
                         power_W);
        }

        HAL_Delay(RAMP_UPDATE_MS);
    }
}

/* =========================================================
   STATUS
   ========================================================= */
static void Print_Status(float current_A,
                         float voltage_V,
                         float power_W)
{
    printf("Current = %.3f A | "
           "Voltage = %.3f V | "
           "Power = %.3f W | "
           "Speed = %lu Hz\r\n",
           current_A,
           voltage_V,
           power_W,
           (unsigned long)motor_frequency);
}

/* =========================================================
   MAIN
   ========================================================= */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();

    HAL_Delay(500U);

    printf("\r\n");
    printf("========================================\r\n");
    printf("EM870S + INA219 CURRENT CONTROL\r\n");
    printf("========================================\r\n");

    printf("Current limit = %.3f A\r\n",
           CURRENT_LIMIT_A);

    printf("Restart threshold = %.3f A\r\n",
           CURRENT_RESTART_A);

    printf("Start speed = %lu Hz\r\n",
           (unsigned long)MOTOR_START_SPEED);

    printf("Max speed = %lu Hz\r\n",
           (unsigned long)MOTOR_MAX_SPEED);

    printf("Speed step = %lu Hz\r\n",
           (unsigned long)SPEED_STEP);

    /* =====================================================
       INA219 CHECK
       ===================================================== */
    printf("Checking INA219 at 0x40...\r\n");

    if (!INA219_IsConnected())
    {
        printf("No INA219 found at 0x40\r\n");
        printf("INA219 NOT READY\r\n");

        Motor_Stop();

        printf("MOTOR = STOP\r\n");

        while (1)
        {
            HAL_Delay(1000U);
        }
    }

    printf("INA219 FOUND at 0x40\r\n");

    if (INA219_Init() != HAL_OK)
    {
        printf("INA219 INITIALIZATION FAILED\r\n");

        Motor_Stop();

        printf("MOTOR = STOP\r\n");

        while (1)
        {
            HAL_Delay(1000U);
        }
    }

    printf("INA219 INITIALIZED\r\n");

    /* =====================================================
       AUTOMATIC RUN / STOP / RESTART
       ===================================================== */
    while (1)
    {
        /*
           Start from 100 Hz and accelerate.
        */
        Motor_Accelerate();

        /*
           If current limit was reached during acceleration,
           Motor_Accelerate() has already stopped the motor.
        */
        if (!motor_running)
        {
            Wait_For_Current_Clear();
            continue;
        }

        /*
           Motor reached 2000 Hz.
           Run continuously until current limit.
        */
        Run_At_Max_Speed();

        /*
           Run_At_Max_Speed() returns after motor has
           decelerated and stopped because of the current limit.

           Wait until current falls below restart threshold.
        */
        if (!motor_running)
        {
            Wait_For_Current_Clear();
        }

        /*
           Loop starts again:
           100 Hz -> acceleration -> 2000 Hz.
        */
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
        PWR_REGULATOR_VOLTAGE_SCALE1);

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
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
        RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_5)
        != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   I2C1
   PB6 = SCL
   PB7 = SDA
   ========================================================= */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed = 100000U;
    hi2c1.Init.DutyCycle =
        I2C_DUTYCYCLE_2;

    hi2c1.Init.OwnAddress1 = 0U;

    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c1.Init.OwnAddress2 = 0U;

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
   USART2
   PA2 = TX
   PA3 = RX
   ========================================================= */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200U;

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
   TIM3
   90 MHz / 90 = 1 MHz
   ========================================================= */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};

    htim3.Instance = TIM3;

    htim3.Init.Prescaler = 89U;

    htim3.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim3.Init.Period = 4999U;

    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV2;

    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(
            &htim3,
            &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   GPIO
   PB0 = PUL+
   PB1 = DIR+
   ========================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0 | GPIO_PIN_1,
                      GPIO_PIN_RESET);

    GPIO_InitStruct.Pin =
        GPIO_PIN_0 | GPIO_PIN_1;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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