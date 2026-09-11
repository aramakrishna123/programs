// pb8,pb9 encoder...pa0 joystick....pc0 limit switch...




#include "main.h"
#include <stdio.h>
#include <stdint.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;


/* =========================================================
   ENCODER SETTINGS
   PB8 = Encoder A
   PB9 = Encoder B
   ========================================================= */

volatile int32_t encoder_count = 0;

static uint8_t previous_state = 0U;

#define ENCODER_SQUARES_PER_INCH    180
#define ENCODER_COUNTS_PER_SQUARE   4
#define ENCODER_COUNTS_PER_INCH \
    (ENCODER_SQUARES_PER_INCH * ENCODER_COUNTS_PER_SQUARE)


/* =========================================================
   JOYSTICK SETTINGS
   PA0 = VRx
   ========================================================= */

#define JOYSTICK_DEADZONE    500

static uint16_t joystick_center = 2048;


/* =========================================================
   STEPPER SETTINGS
   PB0 = STEP / PUL
   PB1 = DIR
   ========================================================= */

#define MIN_STEP_DELAY_MS    2
#define MAX_STEP_DELAY_MS    20


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

void Error_Handler(void);

static uint16_t Joystick_Read(void);
static void Joystick_Calibrate(void);

static void Encoder_Update(void);
static void Encoder_PrintPosition(void);

static void Stepper_Step(uint32_t delay_ms);


/* =========================================================
   UART PRINTF REDIRECTION
   ========================================================= */

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


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    uint16_t joystick;
    int32_t difference;
    uint32_t step_delay;
    uint32_t last_encoder_print = 0U;

    uint8_t A;
    uint8_t B;


    /* =====================================================
       HAL INITIALIZATION
       ===================================================== */

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();


    /* =====================================================
       INITIAL ENCODER STATE

       A B:
       00 = state 0
       01 = state 1
       11 = state 3
       10 = state 2
       ===================================================== */

    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
            ? 1U
            : 0U;

    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            ? 1U
            : 0U;

    previous_state = (uint8_t)((A << 1U) | B);


    /* =====================================================
       MOTOR OFF AT START
       ===================================================== */

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_1,
                      GPIO_PIN_RESET);


    /* =====================================================
       START MESSAGE
       ===================================================== */

    printf("\r\n");
    printf("============================================\r\n");
    printf(" JOYSTICK + STEPPER + ENCODER SYSTEM\r\n");
    printf("============================================\r\n");

    printf("VRx        = PA0\r\n");
    printf("STEP/PUL   = PB0\r\n");
    printf("DIR        = PB1\r\n");
    printf("Encoder A  = PB8\r\n");
    printf("Encoder B  = PB9\r\n");
    printf("Limit      = PC0\r\n");
    printf("--------------------------------------------\r\n");

    printf("Keep joystick CENTERED...\r\n");
    printf("Calibrating joystick center...\r\n");


    /* =====================================================
       JOYSTICK CENTER CALIBRATION
       ===================================================== */

    Joystick_Calibrate();


    printf("Joystick center = %u\r\n",
           joystick_center);

    printf("Deadzone = +/- %u\r\n",
           JOYSTICK_DEADZONE);

    printf("Motor ready.\r\n");
    printf("Encoder monitoring continuously.\r\n");

    printf("============================================\r\n");


    HAL_Delay(500);


    /* =====================================================
       MAIN LOOP
       ===================================================== */

    while (1)
    {
        /* =================================================
           1. ALWAYS READ ENCODER

           Encoder operation is independent of joystick.
           ================================================= */

        Encoder_Update();


        /* =================================================
           2. READ JOYSTICK
           ================================================= */

        joystick = Joystick_Read();


        /* =================================================
           3. CALCULATE DISTANCE FROM CENTER
           ================================================= */

        difference = (int32_t)joystick -
                     (int32_t)joystick_center;


        /* =================================================
           4. LIMIT SWITCH

           PC0 LOW = limit switch pressed
           ================================================= */

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)
            == GPIO_PIN_RESET)
        {
            /* Force STEP LOW */

            HAL_GPIO_WritePin(GPIOB,
                              GPIO_PIN_0,
                              GPIO_PIN_RESET);

            /*
             * Motor stopped.
             *
             * Encoder continues to be monitored because
             * Encoder_Update() is at the top of every loop.
             */

            HAL_Delay(1);
        }


        /* =================================================
           5. JOYSTICK CENTER = MOTOR STOP
           ================================================= */

        else if ((difference > -JOYSTICK_DEADZONE) &&
                 (difference < JOYSTICK_DEADZONE))
        {
            /* Force STEP LOW */

            HAL_GPIO_WritePin(GPIOB,
                              GPIO_PIN_0,
                              GPIO_PIN_RESET);

            /*
             * Motor stopped.
             *
             * Encoder still runs.
             */

            HAL_Delay(1);
        }


        /* =================================================
           6. JOYSTICK LEFT = ANTICLOCKWISE
           ================================================= */

        else if (difference <= -JOYSTICK_DEADZONE)
        {
            uint32_t magnitude;


            /*
             * Convert negative difference to positive
             * magnitude.
             */

            magnitude = (uint32_t)(-difference);


            /*
             * Limit magnitude.
             */

            if (magnitude > (uint32_t)joystick_center)
            {
                magnitude = (uint32_t)joystick_center;
            }


            /*
             * PB1 SET = ANTICLOCKWISE
             */

            HAL_GPIO_WritePin(GPIOB,
                              GPIO_PIN_1,
                              GPIO_PIN_SET);


            /*
             * More joystick movement
             * = faster motor.
             */

            step_delay =
                MAX_STEP_DELAY_MS -
                (
                    (magnitude *
                     (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS))
                    / joystick_center
                );


            if (step_delay < MIN_STEP_DELAY_MS)
            {
                step_delay = MIN_STEP_DELAY_MS;
            }


            if (step_delay > MAX_STEP_DELAY_MS)
            {
                step_delay = MAX_STEP_DELAY_MS;
            }


            Stepper_Step(step_delay);
        }


        /* =================================================
           7. JOYSTICK RIGHT = CLOCKWISE
           ================================================= */

        else
        {
            uint32_t magnitude;
            uint32_t joystick_range;


            /*
             * Positive joystick distance from center.
             */

            magnitude = (uint32_t)difference;


            /*
             * Maximum positive joystick range.
             */

            joystick_range =
                4095U - (uint32_t)joystick_center;


            /*
             * Limit magnitude.
             */

            if (magnitude > joystick_range)
            {
                magnitude = joystick_range;
            }


            /*
             * PB1 RESET = CLOCKWISE
             */

            HAL_GPIO_WritePin(GPIOB,
                              GPIO_PIN_1,
                              GPIO_PIN_RESET);


            /*
             * More joystick movement
             * = faster motor.
             */

            step_delay =
                MAX_STEP_DELAY_MS -
                (
                    (magnitude *
                     (MAX_STEP_DELAY_MS - MIN_STEP_DELAY_MS))
                    / joystick_range
                );


            if (step_delay < MIN_STEP_DELAY_MS)
            {
                step_delay = MIN_STEP_DELAY_MS;
            }


            if (step_delay > MAX_STEP_DELAY_MS)
            {
                step_delay = MAX_STEP_DELAY_MS;
            }


            Stepper_Step(step_delay);
        }


        /* =================================================
           8. ALWAYS PRINT ENCODER

           Every 200 ms.

           Motor moving:
               encoder values change.

           Motor stopped:
               last encoder values remain and are printed.

           Motor moves again:
               new values are read and displayed.
           ================================================= */

        if ((HAL_GetTick() - last_encoder_print) >= 200U)
        {
            last_encoder_print = HAL_GetTick();

            Encoder_PrintPosition();


            /*
             * Joystick diagnostic value.
             */

              }
    }
}


/* =========================================================
   JOYSTICK READ
   ========================================================= */

static uint16_t Joystick_Read(void)
{
    uint16_t value = 0U;


    HAL_ADC_Start(&hadc1);


    if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
    {
        value = HAL_ADC_GetValue(&hadc1);
    }


    HAL_ADC_Stop(&hadc1);


    return value;
}


/* =========================================================
   JOYSTICK CALIBRATION
   ========================================================= */

static void Joystick_Calibrate(void)
{
    uint32_t total = 0U;

    uint16_t value;

    int i;


    /*
     * Take 100 samples while joystick is centered.
     */

    for (i = 0; i < 100; i++)
    {
        value = Joystick_Read();

        total += (uint32_t)value;

        HAL_Delay(10);
    }


    /*
     * Average = actual center.
     */

    joystick_center =
        (uint16_t)(total / 100U);
}


/* =========================================================
   STEPPER STEP
   ========================================================= */

static void Stepper_Step(uint32_t delay_ms)
{
    /*
     * STEP HIGH
     */

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_SET);


    /*
     * STEP pulse width
     */

    HAL_Delay(1);


    /*
     * STEP LOW
     */

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);


    /*
     * Delay before next step.
     */

    HAL_Delay(delay_ms);
}


/* =========================================================
   ENCODER UPDATE
   ========================================================= */

static void Encoder_Update(void)
{
    uint8_t A;
    uint8_t B;
    uint8_t current_state;


    /* -----------------------------------------------------
       Read encoder A
       ----------------------------------------------------- */

    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
            ? 1U
            : 0U;


    /* -----------------------------------------------------
       Read encoder B
       ----------------------------------------------------- */

    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            ? 1U
            : 0U;


    /* -----------------------------------------------------
       Make 2-bit state

       A B

       0 0 = 00
       0 1 = 01
       1 1 = 11
       1 0 = 10
       ----------------------------------------------------- */

    current_state =
        (uint8_t)((A << 1U) | B);


    /* -----------------------------------------------------
       Quadrature decoding

       Positive direction:
       00 -> 01 -> 11 -> 10 -> 00

       Negative direction:
       00 -> 10 -> 11 -> 01 -> 00
       ----------------------------------------------------- */

    switch ((uint8_t)
            ((previous_state << 2U) | current_state))
    {
        /* Positive direction */

        case 0x01U:
        case 0x07U:
        case 0x0EU:
        case 0x08U:

            encoder_count++;
            break;


        /* Negative direction */

        case 0x02U:
        case 0x0BU:
        case 0x0DU:
        case 0x04U:

            encoder_count--;
            break;


        default:

            /*
             * No movement or invalid transition.
             */

            break;
    }


    /*
     * Save current state.
     */

    previous_state = current_state;
}


/* =========================================================
   ENCODER PRINT POSITION
   ========================================================= */

static void Encoder_PrintPosition(void)
{
    uint8_t A;
    uint8_t B;

    int32_t count;

    int64_t position_um;

    int64_t abs_position_um;

    int64_t whole_mm;

    int64_t decimal_mm;


    /* -----------------------------------------------------
       Read current encoder A
       ----------------------------------------------------- */

    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
            ? 1U
            : 0U;


    /* -----------------------------------------------------
       Read current encoder B
       ----------------------------------------------------- */

    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            ? 1U
            : 0U;


    /* -----------------------------------------------------
       Read current count
       ----------------------------------------------------- */

    count = encoder_count;


    /* -----------------------------------------------------
       Position calculation

       180 squares / inch
       4 counts / square

       = 720 counts / inch

       1 inch = 25.4 mm = 25400 micrometers
       ----------------------------------------------------- */

    position_um =
        ((int64_t)count * 25400LL)
        / ENCODER_COUNTS_PER_INCH;


    /* -----------------------------------------------------
       Negative position
       ----------------------------------------------------- */

    if (position_um < 0)
    {
        abs_position_um = -position_um;

        whole_mm = abs_position_um / 1000LL;

        decimal_mm = abs_position_um % 1000LL;


        printf("A=%u B=%u Count=%ld Position=-%ld.%03ld mm\r\n",
               (unsigned int)A,
               (unsigned int)B,
               (long)count,
               (long)whole_mm,
               (long)decimal_mm);
    }


    /* -----------------------------------------------------
       Positive position
       ----------------------------------------------------- */

    else
    {
        whole_mm = position_um / 1000LL;

        decimal_mm = position_um % 1000LL;


        printf("A=%u B=%u Count=%ld Position=%ld.%03ld mm\r\n",
               (unsigned int)A,
               (unsigned int)B,
               (long)count,
               (long)whole_mm,
               (long)decimal_mm);
    }
}


/* =========================================================
   GPIO INITIALIZATION
   ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* -----------------------------------------------------
       Enable GPIO clocks
       ----------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();


    /* =====================================================
       PB0 = STEP
       PB1 = DIR
       ===================================================== */

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


    HAL_GPIO_Init(GPIOB,
                  &GPIO_InitStruct);


    /* =====================================================
       PB8 = ENCODER A
       PB9 = ENCODER B
       ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_8 | GPIO_PIN_9;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(GPIOB,
                  &GPIO_InitStruct);


    /* =====================================================
       PC0 = LIMIT SWITCH

       Pull-up enabled.

       Not pressed = HIGH
       Pressed     = LOW
       ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_0;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;


    HAL_GPIO_Init(GPIOC,
                  &GPIO_InitStruct);
}


/* =========================================================
   ADC1 INITIALIZATION
   ========================================================= */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};


    hadc1.Instance =
        ADC1;


    hadc1.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;


    hadc1.Init.Resolution =
        ADC_RESOLUTION_12B;


    hadc1.Init.ScanConvMode =
        DISABLE;


    hadc1.Init.ContinuousConvMode =
        DISABLE;


    hadc1.Init.DiscontinuousConvMode =
        DISABLE;


    hadc1.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;


    hadc1.Init.ExternalTrigConv =
        ADC_SOFTWARE_START;


    hadc1.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;


    hadc1.Init.NbrOfConversion =
        1;


    hadc1.Init.DMAContinuousRequests =
        DISABLE;


    hadc1.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;


    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }


    /* -----------------------------------------------------
       PA0 = ADC1_IN0
       ----------------------------------------------------- */

    sConfig.Channel =
        ADC_CHANNEL_0;


    sConfig.Rank =
        1;


    sConfig.SamplingTime =
        ADC_SAMPLETIME_84CYCLES;


    if (HAL_ADC_ConfigChannel(&hadc1,
                              &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   USART2 INITIALIZATION
   ========================================================= */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance =
        USART2;


    huart2.Init.BaudRate =
        115200;


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
   SYSTEM CLOCK
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* -----------------------------------------------------
       Enable power control clock
       ----------------------------------------------------- */

    __HAL_RCC_PWR_CLK_ENABLE();


    /* -----------------------------------------------------
       Voltage scaling
       ----------------------------------------------------- */

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1);


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
        16;


    RCC_OscInitStruct.PLL.PLLN =
        360;


    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;


    RCC_OscInitStruct.PLL.PLLQ =
        7;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }


    /* -----------------------------------------------------
       Enable OverDrive
       ----------------------------------------------------- */

    if (HAL_PWREx_EnableOverDrive()
        != HAL_OK)
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


    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_5)
        != HAL_OK)
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