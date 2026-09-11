#include "main.h"

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);


/* =========================================================
   MOTOR SETTINGS
   ========================================================= */

/*
 * DM556 = 400 pulses/revolution
 *
 * STEP timing:
 * 5 ms HIGH + 4 ms LOW
 *
 * Approx. frequency = 111 Hz
 */
#define STEPS_PER_REV    400


/* =========================================================
   STEP FUNCTION
   ========================================================= */

/*
 * Returns:
 *
 * 1 = step generated
 * 0 = limit switch pressed
 */
uint8_t Stepper_Step(void)
{
    /* -----------------------------------------------------
       Check limit switch
       PC0 LOW = switch pressed
       ----------------------------------------------------- */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        /* Stop STEP output */
        HAL_GPIO_WritePin(GPIOB,
                          GPIO_PIN_0,
                          GPIO_PIN_RESET);

        return 0;
    }


    /* -----------------------------------------------------
       STEP HIGH
       ----------------------------------------------------- */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_SET);

    HAL_Delay(5);


    /* -----------------------------------------------------
       Check limit switch again while STEP is HIGH
       ----------------------------------------------------- */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        HAL_GPIO_WritePin(GPIOB,
                          GPIO_PIN_0,
                          GPIO_PIN_RESET);

        return 0;
    }


    /* -----------------------------------------------------
       STEP LOW
       ----------------------------------------------------- */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);

    HAL_Delay(4);

    return 1;
}


/* =========================================================
   ROTATE MOTOR
   ========================================================= */

/*
 * direction:
 *
 * GPIO_PIN_RESET = Clockwise
 * GPIO_PIN_SET   = Anticlockwise
 *
 * This matches your current motor wiring/test.
 */
uint8_t Stepper_Rotate(uint32_t steps,
                       GPIO_PinState direction)
{
    /* Set direction */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_1,
                      direction);

    /* Allow DM556 DIR signal to settle */
    HAL_Delay(20);


    /* Generate STEP pulses */
    for (uint32_t i = 0; i < steps; i++)
    {
        /*
         * If limit is pressed,
         * stop immediately.
         */
        if (Stepper_Step() == 0)
        {
            return 0;
        }
    }

    return 1;
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    /* Reset peripherals and initialize Flash and SysTick */
    HAL_Init();


    /* Configure system clock */
    SystemClock_Config();


    /* Initialize GPIO */
    MX_GPIO_Init();


    /* -----------------------------------------------------
       Initial states
       ----------------------------------------------------- */

    /* STEP LOW */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);


    /* DIR = LOW */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_1,
                      GPIO_PIN_RESET);


    /* Give DM556 time to initialize */
    HAL_Delay(500);


    /* =====================================================
       MAIN LOOP
       ===================================================== */

    while (1)
    {
        /*
         * -------------------------------------------------
         * CLOCKWISE
         * -------------------------------------------------
         */

        /*
         * If limit switch is already pressed,
         * do not start the motor.
         */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
            == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(GPIOB,
                              GPIO_PIN_0,
                              GPIO_PIN_RESET);

            HAL_Delay(100);
            continue;
        }


        /*
         * One complete revolution clockwise
         */
        Stepper_Rotate(STEPS_PER_REV,
                       GPIO_PIN_RESET);


        /*
         * Stop for 1 second
         */
        HAL_Delay(1000);


        /*
         * -------------------------------------------------
         * ANTICLOCKWISE
         * -------------------------------------------------
         */

        /*
         * Only run reverse if limit is released.
         */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
            == GPIO_PIN_RESET)
        {
            /*
             * Limit still pressed.
             * Keep motor stopped.
             */
            HAL_GPIO_WritePin(GPIOB,
                              GPIO_PIN_0,
                              GPIO_PIN_RESET);

            HAL_Delay(100);
            continue;
        }


        /*
         * One complete revolution anticlockwise
         */
        Stepper_Rotate(STEPS_PER_REV,
                       GPIO_PIN_SET);


        /*
         * Stop for 1 second
         */
        HAL_Delay(1000);
    }
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


    /* HSI + PLL configuration */
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


    /* Enable OverDrive */
    if (HAL_PWREx_EnableOverDrive()
        != HAL_OK)
    {
        Error_Handler();
    }


    /* CPU, AHB and APB clocks */
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
            FLASH_LATENCY_5)
        != HAL_OK)
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


    /* Enable GPIOB and GPIOC clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();


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


    /* -----------------------------------------------------
       PC0 = LIMIT SWITCH
       ----------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_0;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    /*
     * Internal pull-up:
     *
     * Switch released → HIGH
     * Switch pressed  → LOW
     */
    GPIO_InitStruct.Pull =
        GPIO_PULLUP;


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