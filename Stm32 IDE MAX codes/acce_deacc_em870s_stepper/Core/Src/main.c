// pbo..pul+...pb1...dir+


#include "main.h"
#include <stdint.h>

/* =========================================================
   TIMER HANDLE
   ========================================================= */

TIM_HandleTypeDef htim3;


/* =========================================================
   MOTOR PINS
   ========================================================= */

#define STEP_PORT       GPIOB
#define STEP_PIN        GPIO_PIN_0

#define DIR_PORT        GPIOB
#define DIR_PIN         GPIO_PIN_1


/* =========================================================
   MOTOR DIRECTION
   ========================================================= */

/*
 * LOW = CLOCKWISE
 *
 * This follows your previously tested direction.
 */
#define CLOCKWISE_LEVEL     GPIO_PIN_RESET


/* =========================================================
   SPEED SETTINGS
   ========================================================= */

/*
 * STEP frequency in steps/second.
 *
 * Start slowly so acceleration is easy to observe.
 */
#define START_SPEED         100U

/*
 * Maximum STEP frequency.
 */
#define MAX_SPEED           2000U


/* =========================================================
   ACCELERATION / DECELERATION
   ========================================================= */

/*
 * Every RAMP_UPDATE_MS:
 *
 * acceleration:
 * speed = speed + SPEED_STEP
 *
 * deceleration:
 * speed = speed - SPEED_STEP
 */
#define SPEED_STEP          20U
#define RAMP_UPDATE_MS      20U


/* =========================================================
   MAXIMUM SPEED HOLD TIME
   ========================================================= */

#define MAX_SPEED_HOLD_MS   2000U


/* =========================================================
   STOPPED HOLD TIME
   ========================================================= */

#define STOP_HOLD_MS        1000U


/* =========================================================
   TIMER COUNTER
   ========================================================= */

/*
 * STM32F446RE:
 *
 * SYSCLK = 180 MHz
 * APB1 = 45 MHz
 * TIM3 timer clock = 90 MHz
 *
 * Prescaler = 89
 *
 * 90 MHz / 90 = 1 MHz
 *
 * Therefore:
 *
 * 1 timer count = 1 us
 */
#define TIMER_COUNTER_HZ    1000000UL


/* =========================================================
   MOTOR STATE
   ========================================================= */

typedef enum
{
    MOTOR_STOPPED = 0,
    MOTOR_ACCELERATING,
    MOTOR_RUNNING,
    MOTOR_DECELERATING

} MotorState_t;


/* =========================================================
   GLOBAL VARIABLES
   ========================================================= */

volatile MotorState_t motor_state = MOTOR_STOPPED;

volatile uint32_t motor_speed = 0U;

uint32_t last_ramp_update = 0U;

uint32_t max_speed_start_time = 0U;

uint32_t stop_start_time = 0U;


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

static void Motor_SetFrequency(uint32_t frequency);
static void Motor_Start(void);
static void Motor_Accelerate(void);
static void Motor_Decelerate(void);


/* =========================================================
   TIMER INTERRUPT CALLBACK
   ========================================================= */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        /*
         * Toggle PB0.
         *
         * PB0 generates STEP/PUL.
         */
        HAL_GPIO_TogglePin(
            STEP_PORT,
            STEP_PIN
        );
    }
}


/* =========================================================
   SET STEP FREQUENCY
   ========================================================= */

static void Motor_SetFrequency(uint32_t frequency)
{
    uint32_t half_period_us;


    /*
     * Frequency = 0 means STOP.
     */
    if (frequency == 0U)
    {
        HAL_TIM_Base_Stop_IT(&htim3);

        HAL_GPIO_WritePin(
            STEP_PORT,
            STEP_PIN,
            GPIO_PIN_RESET
        );

        motor_speed = 0U;

        motor_state = MOTOR_STOPPED;

        return;
    }


    /*
     * Timer interrupt toggles STEP.
     *
     * Therefore:
     *
     * interrupt frequency = 2 x STEP frequency
     *
     * half period:
     *
     * 1,000,000 / (2 x frequency)
     */
    half_period_us =
        TIMER_COUNTER_HZ /
        (2U * frequency);


    /*
     * ARR is zero-based.
     */
    if (half_period_us > 0U)
    {
        half_period_us--;
    }


    /*
     * TIM3 is 16-bit.
     */
    if (half_period_us > 65535U)
    {
        half_period_us = 65535U;
    }


    /*
     * Set new timer period.
     */
    __HAL_TIM_SET_AUTORELOAD(
        &htim3,
        half_period_us
    );


    /*
     * Reset timer counter.
     */
    __HAL_TIM_SET_COUNTER(
        &htim3,
        0U
    );


    /*
     * Store current speed.
     */
    motor_speed = frequency;


    /*
     * Start timer if not already running.
     */
    if ((htim3.Instance->CR1 & TIM_CR1_CEN) == 0U)
    {
        HAL_TIM_Base_Start_IT(&htim3);
    }
}


/* =========================================================
   START MOTOR
   ========================================================= */

static void Motor_Start(void)
{
    /*
     * Set clockwise direction.
     */
    HAL_GPIO_WritePin(
        DIR_PORT,
        DIR_PIN,
        CLOCKWISE_LEVEL
    );


    /*
     * STEP LOW before starting.
     */
    HAL_GPIO_WritePin(
        STEP_PORT,
        STEP_PIN,
        GPIO_PIN_RESET
    );


    /*
     * Allow DIR to settle.
     */
    HAL_Delay(10U);


    /*
     * Start at low speed.
     */
    motor_speed = START_SPEED;

    motor_state = MOTOR_ACCELERATING;

    last_ramp_update = HAL_GetTick();


    /*
     * Begin STEP pulses.
     */
    Motor_SetFrequency(
        START_SPEED
    );
}


/* =========================================================
   ACCELERATION
   ========================================================= */

static void Motor_Accelerate(void)
{
    uint32_t now = HAL_GetTick();


    /*
     * Wait for next acceleration step.
     */
    if ((now - last_ramp_update) < RAMP_UPDATE_MS)
    {
        return;
    }


    last_ramp_update = now;


    /*
     * Increase speed.
     */
    if (motor_speed < MAX_SPEED)
    {
        if ((motor_speed + SPEED_STEP) >= MAX_SPEED)
        {
            motor_speed = MAX_SPEED;

            Motor_SetFrequency(
                MAX_SPEED
            );

            motor_state = MOTOR_RUNNING;

            max_speed_start_time = HAL_GetTick();
        }
        else
        {
            motor_speed += SPEED_STEP;

            Motor_SetFrequency(
                motor_speed
            );
        }
    }
    else
    {
        motor_speed = MAX_SPEED;

        motor_state = MOTOR_RUNNING;

        max_speed_start_time = HAL_GetTick();
    }
}


/* =========================================================
   DECELERATION
   ========================================================= */

static void Motor_Decelerate(void)
{
    uint32_t now = HAL_GetTick();


    /*
     * Wait for next deceleration step.
     */
    if ((now - last_ramp_update) < RAMP_UPDATE_MS)
    {
        return;
    }


    last_ramp_update = now;


    /*
     * Decrease speed.
     */
    if (motor_speed > START_SPEED)
    {
        if (motor_speed > SPEED_STEP)
        {
            motor_speed -= SPEED_STEP;
        }
        else
        {
            motor_speed = START_SPEED;
        }


        Motor_SetFrequency(
            motor_speed
        );
    }
    else
    {
        /*
         * Completely stop STEP pulses.
         */
        motor_speed = 0U;


        HAL_TIM_Base_Stop_IT(
            &htim3
        );


        HAL_GPIO_WritePin(
            STEP_PORT,
            STEP_PIN,
            GPIO_PIN_RESET
        );


        motor_state = MOTOR_STOPPED;


        stop_start_time = HAL_GetTick();
    }
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    HAL_Init();


    SystemClock_Config();


    MX_GPIO_Init();


    MX_TIM3_Init();


    /*
     * -----------------------------------------------------
     * MOTOR INITIAL STATE
     * -----------------------------------------------------
     */

    /*
     * CLOCKWISE
     */
    HAL_GPIO_WritePin(
        DIR_PORT,
        DIR_PIN,
        CLOCKWISE_LEVEL
    );


    /*
     * STEP LOW
     */
    HAL_GPIO_WritePin(
        STEP_PORT,
        STEP_PIN,
        GPIO_PIN_RESET
    );


    motor_speed = 0U;

    motor_state = MOTOR_STOPPED;


    /*
     * -----------------------------------------------------
     * AUTOMATIC START
     * -----------------------------------------------------
     */

    Motor_Start();


    /*
     * -----------------------------------------------------
     * MAIN LOOP
     * -----------------------------------------------------
     */

    while (1)
    {
        /*
         * -------------------------------------------------
         * ACCELERATING
         * -------------------------------------------------
         */

        if (motor_state == MOTOR_ACCELERATING)
        {
            Motor_Accelerate();
        }


        /*
         * -------------------------------------------------
         * MAXIMUM SPEED
         * -------------------------------------------------
         */

        else if (motor_state == MOTOR_RUNNING)
        {
            /*
             * Hold maximum speed for 2 seconds.
             */
            if ((HAL_GetTick() - max_speed_start_time)
                    >= MAX_SPEED_HOLD_MS)
            {
                motor_state = MOTOR_DECELERATING;

                last_ramp_update = HAL_GetTick();
            }
        }


        /*
         * -------------------------------------------------
         * DECELERATING
         * -------------------------------------------------
         */

        else if (motor_state == MOTOR_DECELERATING)
        {
            Motor_Decelerate();
        }


        /*
         * -------------------------------------------------
         * STOPPED
         * -------------------------------------------------
         */

        else if (motor_state == MOTOR_STOPPED)
        {
            /*
             * Stay stopped for 1 second.
             */
            if ((HAL_GetTick() - stop_start_time)
                    >= STOP_HOLD_MS)
            {
                /*
                 * Start another cycle.
                 */
                Motor_Start();
            }
        }


        /*
         * Small CPU delay.
         */
        HAL_Delay(1U);
    }
}


/* =========================================================
   GPIO INITIALIZATION
   ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /*
     * Enable GPIOB clock.
     */
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /*
     * PB0 = STEP
     * PB1 = DIR
     *
     * Initial state LOW.
     */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0 | GPIO_PIN_1,
        GPIO_PIN_RESET
    );


    /*
     * Configure PB0/PB1 as outputs.
     */
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
   TIM3 INITIALIZATION
   ========================================================= */

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};


    /*
     * TIM3 timer clock:
     *
     * 90 MHz
     *
     * Prescaler:
     *
     * 89
     *
     * Counter clock:
     *
     * 90 MHz / 90
     * = 1 MHz
     */
    htim3.Instance = TIM3;


    htim3.Init.Prescaler =
        89U;


    htim3.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    /*
     * Initial 100 Hz STEP:
     *
     * half period:
     *
     * 1,000,000 / (2 x 100)
     * = 5000 us
     *
     * ARR = 4999
     */
    htim3.Init.Period =
        4999U;


    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (HAL_TIM_Base_Init(
            &htim3) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Internal clock.
     */
    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;


    if (HAL_TIM_ConfigClockSource(
            &htim3,
            &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Timer is initially stopped.
     */
    HAL_TIM_Base_Stop_IT(
        &htim3
    );
}


/* =========================================================
   SYSTEM CLOCK
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* -----------------------------------------------------
       Power
       ----------------------------------------------------- */

    __HAL_RCC_PWR_CLK_ENABLE();


    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    /* -----------------------------------------------------
       HSI + PLL
       ----------------------------------------------------- */

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


    /* -----------------------------------------------------
       OverDrive
       ----------------------------------------------------- */

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }


    /* -----------------------------------------------------
       Clock tree
       ----------------------------------------------------- */

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