/* =========================================================
   LEADSHINE EM870S CONTROL SYSTEM
   STM32F446RE

   JOYSTICK + STEPPER MOTOR + ENCODER
   + INA219 CURRENT SENSOR + LIMIT SWITCH

   Z-AXIS
   =========================================================
   CONNECTIONS
   =========================================================

   JOYSTICK (HW-504)
       VRX  -> PA0
       VCC  -> 3.3V
       GND  -> STM32 GND

   STEPPER DRIVER (LEADSHINE EM870S)
       PUL+  -> PB0
       DIR+  -> PB1
       PUL-  -> STM32 GND
       DIR-  -> STM32 GND

   STEPPER MOTOR
       Motor wires -> EM870S motor output terminals
       PB0 -> PUL+
       PB1 -> DIR

   ENCODER
       A    -> PB8
       B    -> PB9
       VCC  -> 3.3V
       GND  -> STM32 GND

   LIMIT SWITCH
       Signal          -> PC0
       Other terminal  -> STM32 GND

   INA219 CURRENT SENSOR
       SCL  -> PB6
       SDA  -> PB7
       VCC  -> 3.3V
       GND  -> STM32 GND

   UART / SERIAL MONITOR
       USART2 TX -> PA2
       USART2 RX -> PA3
       Baud Rate -> 115200

   =========================================================
*/

#include "main.h"
#include "encoder.h"
#include "isensor.h"

#include <stdio.h>

/* 1 = axis driven by the wireless value on USART2 RX, 0 = original local joystick on PA0 */
#define REMOTE_JOYSTICK 1
#include "remote_joystick.h"
#include <stdint.h>


/* =========================================================
   PERIPHERAL HANDLES
   ========================================================= */

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;


/* =========================================================
   JOYSTICK SETTINGS
   ========================================================= */

#define JOYSTICK_DEADZONE          1900U

#define JOYSTICK_MIN_HZ            100U
#define JOYSTICK_MAX_HZ            2000U

#define JOYSTICK_FILTER_SAMPLES    8U
#define JOYSTICK_CENTER_SAMPLES    100U

#define JOYSTICK_CENTER_MIN        1900U
#define JOYSTICK_CENTER_MAX        2700U

#define JOYSTICK_MAX_VARIATION     250U


/* =========================================================
   CURRENT PROTECTION
   ========================================================= */

/*
 * Change only this value for the current limit.
 *
 * 0.700f = 700 mA
 * 0.800f = 800 mA
 * 0.900f = 900 mA
 * 1.000f = 1000 mA
 */

#define CURRENT_LIMIT_A            0.500f

/*
 * Current must fall BELOW this value before
 * an overcurrent fault can clear.
 */

#define CURRENT_RESTART_A          0.450f


/*
 * Number of consecutive over-limit measurements
 * required before declaring a fault.
 *
 * CURRENT_CHECK_MS = 20 ms
 * 3 samples ≈ 60 ms
 */

#define CURRENT_TRIP_SAMPLES       3U

static uint8_t current_fault = 0U;

static uint8_t current_over_limit_count = 0U;


/* =========================================================
   SPEED RAMP
   ========================================================= */

#define SPEED_RAMP_STEP_HZ         25U
#define SPEED_UPDATE_MS            10U


/* =========================================================
   CURRENT MONITOR
   ========================================================= */

#define CURRENT_CHECK_MS           20U
#define CURRENT_PRINT_MS           200U


/* =========================================================
   DIRECTION SETUP
   ========================================================= */

#define DIR_SETUP_US               10U
#define DIR_CHANGE_STOP_MS         2U


/* =========================================================
   TIM1 SETTINGS
   =========================================================

   System clock = 180 MHz

   TIM1 clock = 180 MHz

   Prescaler = 179

   180 MHz / 180 = 1 MHz

   Therefore:
   1 timer count = 1 us

   PB0 = TIM1_CH2N
   ========================================================= */

#define TIM1_TIMER_CLOCK_HZ        180000000U
#define TIM1_TIMER_TICK_HZ         1000000U
#define TIM1_PRESCALER             179U


/* =========================================================
   JOYSTICK CENTER
   ========================================================= */

static uint16_t joystick_center = 2048U;


/* =========================================================
   MOTOR DIRECTION
   =========================================================

   1  = CLOCKWISE
   -1 = ANTICLOCKWISE
   0  = STOPPED / UNDEFINED
   ========================================================= */

static int8_t active_direction = 0;


/* =========================================================
   MOTOR SPEED
   ========================================================= */

static uint32_t current_speed_hz = 0U;

static uint32_t target_speed_hz = 0U;


/* =========================================================
   MOTOR RUNNING STATE
   =========================================================

   0 = MOTOR STOPPED
   1 = MOTOR ROTATING

   This variable is the single source for the
   motor status displayed on UART.
   ========================================================= */

static uint8_t motor_running = 0U;


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

static uint16_t Joystick_Read(void);

static uint16_t Joystick_Read_Average(
    uint16_t samples);

static uint8_t Joystick_Calibrate(void);

static uint8_t Joystick_IsCentered(
    int32_t difference);

static uint32_t Joystick_To_Speed(
    uint16_t joystick);

static void DWT_Init(void);

static void DWT_Delay_us(
    uint32_t us);

static void Stepper_Stop(void);

static void Stepper_SetDirection_Clockwise(void);

static void Stepper_SetDirection_Anticlockwise(void);

static void Stepper_ApplySpeed(
    uint32_t speed_hz);

static void Stepper_RampTo(
    uint32_t target_hz);

static void TIM1_Pin_AlternateFunction(void);

static void TIM1_Pin_OutputLow(void);

void Error_Handler(void);


/* =========================================================
   JOYSTICK SOURCE
   (local PA0 ADC or remote USART2)
   ========================================================= */

static uint8_t Joystick_SetupCenter(void)
{
#if REMOTE_JOYSTICK
    joystick_center = REMOTE_CENTER;
    Remote_Init();
    printf("Joystick source = REMOTE (USART2)\r\n");
    return 1U;
#else
    return Joystick_Calibrate();
#endif
}

static uint16_t Joystick_Sample(void)
{
#if REMOTE_JOYSTICK
    return Remote_ReadOrCenter();
#else
    return Joystick_Read_Average(JOYSTICK_FILTER_SAMPLES);
#endif
}


/* =========================================================
   USART2 RX INTERRUPT (remote joystick)
   ========================================================= */

#if REMOTE_JOYSTICK
void USART2_IRQHandler(void)
{
    Remote_OnUartIRQ();
}
#endif


/* =========================================================
   UART PRINTF
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
    uint16_t joystick;

    int32_t difference;

    uint32_t now;

    uint32_t last_speed_update = 0U;

    uint32_t last_current_check = 0U;

    uint32_t last_current_print = 0U;

    float current_A = 0.0f;

    float voltage_V = 0.0f;

    float power_W = 0.0f;

    uint8_t current_read_ok = 0U;

    uint8_t limit_pressed;


/* =========================================================
   HAL INITIALIZATION
   ========================================================= */

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_ADC1_Init();

    MX_USART2_UART_Init();

    MX_I2C1_Init();

    MX_TIM1_Init();

    DWT_Init();

    Encoder_Init();


/* =========================================================
   MOTOR OFF AT START
   ========================================================= */

    Stepper_Stop();


/* =========================================================
   START MESSAGE
   ========================================================= */

    printf("\r\n");

    printf("============================================\r\n");
    printf(" LEADSHINE EM870S CONTROL SYSTEM\r\n");
    printf(" Z-AXIS\r\n");
    printf("============================================\r\n");

    printf("Joystick VRX  = PA0\r\n");
    printf("STEP/PUL      = PB0 TIM1_CH2N\r\n");
    printf("DIR           = PB1\r\n");
    printf("Encoder A     = PB8\r\n");
    printf("Encoder B     = PB9\r\n");
    printf("Limit Switch  = PC0\r\n");
    printf("INA219 SCL    = PB6\r\n");
    printf("INA219 SDA    = PB7\r\n");
    printf("UART          = USART2 115200\r\n");

    printf("--------------------------------------------\r\n");

    printf("RIGHT = CLOCKWISE\r\n");
    printf("LEFT  = ANTICLOCKWISE\r\n");
    printf("CENTER = STOP\r\n");

    printf("Current Limit = %.3f A\r\n",
           CURRENT_LIMIT_A);

    printf("Restart Below = %.3f A\r\n",
           CURRENT_RESTART_A);

    printf("Trip Samples  = %u\r\n",
           CURRENT_TRIP_SAMPLES);

    printf("Speed Range   = %u - %u Hz\r\n",
           JOYSTICK_MIN_HZ,
           JOYSTICK_MAX_HZ);

    printf("Speed Ramp    = %u Hz / %u ms\r\n",
           SPEED_RAMP_STEP_HZ,
           SPEED_UPDATE_MS);

    printf("--------------------------------------------\r\n");


/* =========================================================
   INA219 CHECK
   ========================================================= */

    printf("Checking INA219...\r\n");

    if (INA219_IsConnected(&hi2c1))
    {
        printf("INA219 FOUND at 0x40\r\n");

        if (INA219_Init(&hi2c1))
        {
            printf("INA219 initialized\r\n");
        }
        else
        {
            printf("INA219 initialization FAILED\r\n");
        }
    }
    else
    {
        printf("INA219 NOT FOUND\r\n");
    }


/* =========================================================
   JOYSTICK CALIBRATION
   ========================================================= */

    printf("--------------------------------------------\r\n");

    printf("Keep joystick CENTERED\r\n");

    printf("Calibrating joystick...\r\n");


    if (!Joystick_SetupCenter())
    {
        Stepper_Stop();

        printf("\r\n");
        printf("============================================\r\n");
        printf("JOYSTICK SIGNAL INVALID / UNSTABLE\r\n");
        printf("CHECK VRX -> PA0\r\n");
        printf("CHECK VCC -> 3.3V\r\n");
        printf("CHECK GND -> STM32 GND\r\n");
        printf("MOTOR DISABLED\r\n");
        printf("============================================\r\n");


        while (1)
        {
            uint16_t test_value;

            test_value =
                Joystick_Read();

            printf("PA0 ADC = %u\r\n",
                   test_value);

            HAL_Delay(1000);
        }
    }


    printf("Joystick center = %u\r\n",
           joystick_center);

    printf("Deadzone = +/- %u\r\n",
           JOYSTICK_DEADZONE);

    printf("Motor ready\r\n");

    printf("Encoder monitoring continuously\r\n");

    printf("============================================\r\n");

    HAL_Delay(500);


/* =========================================================
   MAIN CONTROL LOOP
   ========================================================= */

    while (1)
    {
        now = HAL_GetTick();


/* =========================================================
   1. ENCODER UPDATE
   ========================================================= */

        Encoder_Update();


/* =========================================================
   2. READ JOYSTICK
   ========================================================= */

        joystick =
            Joystick_Sample();


        difference =
            (int32_t)joystick -
            (int32_t)joystick_center;


/* =========================================================
   3. LIMIT SWITCH STATUS
   ========================================================= */

        limit_pressed =
            (HAL_GPIO_ReadPin(
                GPIOC,
                GPIO_PIN_0)
             == GPIO_PIN_RESET)
            ? 1U
            : 0U;


/* =========================================================
   4. CURRENT SENSOR CHECK
   ========================================================= */

        if ((now - last_current_check) >=
            CURRENT_CHECK_MS)
        {
            last_current_check = now;


            if (INA219_ReadCurrent(
                    &hi2c1,
                    &current_A))
            {
                current_read_ok = 1U;


/* ---------------------------------------------------------
   CURRENT OVER LIMIT
   --------------------------------------------------------- */

                if ((!current_fault) &&
                    motor_running &&
                    (current_A >=
                     CURRENT_LIMIT_A))
                {
                    if (current_over_limit_count <
                        CURRENT_TRIP_SAMPLES)
                    {
                        current_over_limit_count++;
                    }


                    if (current_over_limit_count >=
                        CURRENT_TRIP_SAMPLES)
                    {
                        current_fault = 1U;

                        current_over_limit_count = 0U;

                        Stepper_Stop();

                        active_direction = 0;


                        printf("\r\n");
                        printf("========================================\r\n");
                        printf("CURRENT LIMIT REACHED\r\n");

                        printf("Current = %.3f A\r\n",
                               current_A);

                        printf("Limit   = %.3f A\r\n",
                               CURRENT_LIMIT_A);

                        printf("MOTOR OFF\r\n");

                        printf("CURRENT FAULT ACTIVE\r\n");

                        printf("WAITING FOR CURRENT < %.3f A\r\n",
                               CURRENT_RESTART_A);

                        printf("========================================\r\n");
                    }
                }
                else
                {
                    if (current_A < CURRENT_LIMIT_A)
                    {
                        current_over_limit_count = 0U;
                    }
                }


/* ---------------------------------------------------------
   CURRENT FAULT CLEAR
   --------------------------------------------------------- */

                if (current_fault &&
                    (current_A <
                     CURRENT_RESTART_A))
                {
                    current_fault = 0U;

                    current_over_limit_count = 0U;

                    Stepper_Stop();

                    active_direction = 0;


                    printf("\r\n");
                    printf("CURRENT CLEARED\r\n");

                    printf("Current = %.3f A\r\n",
                           current_A);

                    printf("JOYSTICK COMMAND WILL RESUME MOTOR\r\n");

                    printf("\r\n");
                }
            }
            else
            {
                current_read_ok = 0U;
            }
        }


/* =========================================================
   5. DETERMINE TARGET SPEED
   ========================================================= */

        target_speed_hz = 0U;


/* ---------------------------------------------------------
   LIMIT SWITCH
   --------------------------------------------------------- */

        if (limit_pressed)
        {
            target_speed_hz = 0U;

            if (motor_running)
            {
                Stepper_Stop();
            }

            active_direction = 0;
        }


/* ---------------------------------------------------------
   CURRENT FAULT
   --------------------------------------------------------- */

        else if (current_fault)
        {
            target_speed_hz = 0U;

            if (motor_running)
            {
                Stepper_Stop();
            }
        }


/* ---------------------------------------------------------
   JOYSTICK CENTER
   --------------------------------------------------------- */

        else if (Joystick_IsCentered(
                     difference))
        {
            target_speed_hz = 0U;
        }


/* ---------------------------------------------------------
   JOYSTICK LEFT
   --------------------------------------------------------- */

        else if (difference <=
                 -(int32_t)JOYSTICK_DEADZONE)
        {
            /*
             * LEFT = ANTICLOCKWISE
             */

            if ((active_direction == 1) &&
                motor_running)
            {
                Stepper_Stop();

                active_direction = 0;

                HAL_Delay(
                    DIR_CHANGE_STOP_MS);
            }


            if (active_direction != -1)
            {
                Stepper_SetDirection_Anticlockwise();

                active_direction = -1;
            }


            target_speed_hz =
                Joystick_To_Speed(
                    joystick);
        }


/* ---------------------------------------------------------
   JOYSTICK RIGHT
   --------------------------------------------------------- */

        else
        {
            /*
             * RIGHT = CLOCKWISE
             */

            if ((active_direction == -1) &&
                motor_running)
            {
                Stepper_Stop();

                active_direction = 0;

                HAL_Delay(
                    DIR_CHANGE_STOP_MS);
            }


            if (active_direction != 1)
            {
                Stepper_SetDirection_Clockwise();

                active_direction = 1;
            }


            target_speed_hz =
                Joystick_To_Speed(
                    joystick);
        }


/* =========================================================
   6. SPEED RAMP
   ========================================================= */

        if ((now - last_speed_update) >=
            SPEED_UPDATE_MS)
        {
            last_speed_update = now;


            if (current_fault ||
                limit_pressed)
            {
                Stepper_Stop();
            }
            else
            {
                Stepper_RampTo(
                    target_speed_hz);
            }
        }


/* =========================================================
   7. COMBINED STATUS PRINT
   ========================================================= */

        if ((now - last_current_print) >=
            CURRENT_PRINT_MS)
        {
            last_current_print = now;


            if (INA219_ReadCurrent(
                    &hi2c1,
                    &current_A))
            {
                current_read_ok = 1U;
            }
            else
            {
                current_read_ok = 0U;

                current_A = 0.0f;
            }


            if (!INA219_ReadBusVoltage(
                    &hi2c1,
                    &voltage_V))
            {
                voltage_V = 0.0f;
            }


            if (!INA219_ReadPower(
                    &hi2c1,
                    &power_W))
            {
                power_W = 0.0f;
            }


            {
                int32_t encoder_count =
                    Encoder_GetCount();

                float encoder_position =
                    Encoder_GetPositionMM();

                int8_t encoder_direction =
                    Encoder_GetDirection();


                GPIO_PinState enc_a =
                    HAL_GPIO_ReadPin(
                        GPIOB,
                        GPIO_PIN_8);

                GPIO_PinState enc_b =
                    HAL_GPIO_ReadPin(
                        GPIOB,
                        GPIO_PIN_9);


                const char *encoder_dir;

                const char *encoder_status;

                const char *motor_direction;


/* ---------------------------------------------------------
   ENCODER DIRECTION
   --------------------------------------------------------- */

                if (encoder_direction > 0)
                {
                    encoder_dir = "CW";
                }
                else if (encoder_direction < 0)
                {
                    encoder_dir = "CCW";
                }
                else
                {
                    encoder_dir = "STOP";
                }


/* ---------------------------------------------------------
   ENCODER ROTATION STATUS
   --------------------------------------------------------- */

                if (encoder_direction != 0)
                {
                    encoder_status = "ROTATING";
                }
                else
                {
                    encoder_status = "NOT_ROTATING";
                }


/* ---------------------------------------------------------
   MOTOR DIRECTION
   --------------------------------------------------------- */

                if (motor_running &&
                    (active_direction > 0))
                {
                    motor_direction = "CLOCKWISE";
                }
                else if (motor_running &&
                         (active_direction < 0))
                {
                    motor_direction = "ANTICLOCKWISE";
                }
                else
                {
                    motor_direction = "STOP";
                }


/* ---------------------------------------------------------
   COMBINED DISPLAY
   --------------------------------------------------------- */

                printf(
                    "CURRENT=%.3f A | "
                    "VOLTAGE=%.3f V | "
                    "POWER=%.3f W |\r\n"

                    "SENSOR=%s | "
                    "ENC_A=%d | "
                    "ENC_B=%d |\r\n"

                    "ENC_COUNT=%ld | "
                    "ENC_DIR=%s | "
                    "POS=%.2fmm |\r\n"

                    "ENCODER=%s | "
                    "LIMIT=%s |\r\n"

                    "MOTOR=%s | "
                    "SPEED=%lu Hz | "
                    "DIRECTION=%s\r\n",

                    current_A,
                    voltage_V,
                    power_W,

                    current_read_ok
                        ? "CONNECTED"
                        : "NOT_CONNECTED",

                    (int)enc_a,
                    (int)enc_b,

                    (long)encoder_count,

                    encoder_dir,

                    encoder_position,

                    encoder_status,

                    limit_pressed
                        ? "PRESSED"
                        : "RELEASED",

                    motor_running
                        ? "ROTATING"
                        : "STOPPED",

                    (unsigned long)
                        current_speed_hz,

                    motor_direction
                );
            }
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


    if (HAL_ADC_PollForConversion(
            &hadc1,
            10U) == HAL_OK)
    {
        value =
            (uint16_t)
            HAL_ADC_GetValue(&hadc1);
    }


    HAL_ADC_Stop(&hadc1);


    return value;
}


/* =========================================================
   JOYSTICK AVERAGE
   ========================================================= */

static uint16_t Joystick_Read_Average(
    uint16_t samples)
{
    uint32_t total = 0U;

    uint16_t value;

    uint16_t i;


    if (samples == 0U)
    {
        return 0U;
    }


    for (i = 0U;
         i < samples;
         i++)
    {
        value =
            Joystick_Read();

        total += value;
    }


    return (uint16_t)
           (total / samples);
}


/* =========================================================
   JOYSTICK CENTER TEST
   ========================================================= */

static uint8_t Joystick_IsCentered(
    int32_t difference)
{
    if ((difference >
         -(int32_t)JOYSTICK_DEADZONE) &&
        (difference <
         (int32_t)JOYSTICK_DEADZONE))
    {
        return 1U;
    }


    return 0U;
}


/* =========================================================
   JOYSTICK CALIBRATION
   ========================================================= */

static uint8_t Joystick_Calibrate(void)
{
    uint32_t total = 0U;

    uint16_t minimum = 4095U;

    uint16_t maximum = 0U;

    uint16_t value;

    uint16_t i;

    uint16_t variation;


    for (i = 0U;
         i < JOYSTICK_CENTER_SAMPLES;
         i++)
    {
        value =
            Joystick_Read_Average(
                JOYSTICK_FILTER_SAMPLES);


        total += value;


        if (value < minimum)
        {
            minimum = value;
        }


        if (value > maximum)
        {
            maximum = value;
        }


        HAL_Delay(5);
    }


    joystick_center =
        (uint16_t)
        (
            total /
            JOYSTICK_CENTER_SAMPLES
        );


    variation =
        maximum -
        minimum;


    printf("Joystick MIN = %u\r\n",
           minimum);

    printf("Joystick MAX = %u\r\n",
           maximum);

    printf("Joystick CENTER = %u\r\n",
           joystick_center);

    printf("Joystick variation = %u\r\n",
           variation);


    if (variation >
        JOYSTICK_MAX_VARIATION)
    {
        return 0U;
    }


    if ((joystick_center <
         JOYSTICK_CENTER_MIN) ||
        (joystick_center >
         JOYSTICK_CENTER_MAX))
    {
        return 0U;
    }


    return 1U;
}


/* =========================================================
   JOYSTICK -> SPEED
   ========================================================= */

static uint32_t Joystick_To_Speed(
    uint16_t joystick)
{
    int32_t difference;

    uint32_t magnitude;

    uint32_t range;

    uint32_t speed;


    difference =
        (int32_t)joystick -
        (int32_t)joystick_center;


/* ---------------------------------------------------------
   LEFT
   --------------------------------------------------------- */

    if (difference < 0)
    {
        magnitude =
            (uint32_t)(-difference);

        range =
            (uint32_t)joystick_center;


        if (magnitude > range)
        {
            magnitude = range;
        }
    }


/* ---------------------------------------------------------
   RIGHT
   --------------------------------------------------------- */

    else
    {
        magnitude =
            (uint32_t)difference;

        range =
            4095U -
            (uint32_t)joystick_center;


        if (magnitude > range)
        {
            magnitude = range;
        }
    }


    if (range == 0U)
    {
        return JOYSTICK_MIN_HZ;
    }


    speed =
        JOYSTICK_MIN_HZ +
        (
            magnitude *
            (JOYSTICK_MAX_HZ -
             JOYSTICK_MIN_HZ)
            /
            range
        );


    if (speed >
        JOYSTICK_MAX_HZ)
    {
        speed =
            JOYSTICK_MAX_HZ;
    }


    if (speed <
        JOYSTICK_MIN_HZ)
    {
        speed =
            JOYSTICK_MIN_HZ;
    }


    return speed;
}


/* =========================================================
   TIM1 PB0 = ALTERNATE FUNCTION
   PB0 AF1 = TIM1_CH2N
   ========================================================= */

static void TIM1_Pin_AlternateFunction(void)
{
    GPIOB->MODER &=
        ~(GPIO_MODER_MODER0_Msk);

    GPIOB->MODER |=
        (2U <<
         GPIO_MODER_MODER0_Pos);


    GPIOB->AFR[0] &=
        ~(GPIO_AFRL_AFSEL0_Msk);

    GPIOB->AFR[0] |=
        (1U <<
         GPIO_AFRL_AFSEL0_Pos);


    GPIOB->PUPDR &=
        ~(GPIO_PUPDR_PUPD0_Msk);


    GPIOB->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED0_Msk);

    GPIOB->OSPEEDR |=
        (3U <<
         GPIO_OSPEEDR_OSPEED0_Pos);
}


/* =========================================================
   TIM1 PB0 = GPIO LOW
   ========================================================= */

static void TIM1_Pin_OutputLow(void)
{
    GPIOB->MODER &=
        ~(GPIO_MODER_MODER0_Msk);

    GPIOB->MODER |=
        (1U <<
         GPIO_MODER_MODER0_Pos);


    GPIOB->OTYPER &=
        ~(GPIO_OTYPER_OT0);


    GPIOB->PUPDR &=
        ~(GPIO_PUPDR_PUPD0_Msk);


    GPIOB->BSRR =
        ((uint32_t)
         GPIO_PIN_0
         << 16U);
}


/* =========================================================
   STEPPER STOP
   ========================================================= */

static void Stepper_Stop(void)
{
    /*
     * Stop timer.
     */

    TIM1->CR1 &=
        ~TIM_CR1_CEN;


    /*
     * Disable complementary CH2.
     */

    TIM1->CCER &=
        ~TIM_CCER_CC2NE;


    /*
     * Disable main output.
     */

    TIM1->BDTR &=
        ~TIM_BDTR_MOE;


    /*
     * Force PB0 LOW.
     */

    TIM1_Pin_OutputLow();


    /*
     * Reset motor state.
     */

    current_speed_hz = 0U;

    motor_running = 0U;
}


/* =========================================================
   CLOCKWISE
   PB1 HIGH
   ========================================================= */

static void Stepper_SetDirection_Clockwise(void)
{
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_1,
        GPIO_PIN_SET);


    DWT_Delay_us(
        DIR_SETUP_US);
}


/* =========================================================
   ANTICLOCKWISE
   PB1 LOW
   ========================================================= */

static void Stepper_SetDirection_Anticlockwise(void)
{
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_1,
        GPIO_PIN_RESET);


    DWT_Delay_us(
        DIR_SETUP_US);
}


/* =========================================================
   APPLY TIM1 SPEED
   ========================================================= */

static void Stepper_ApplySpeed(
    uint32_t speed_hz)
{
    uint32_t period_us;

    uint32_t arr;

    uint32_t compare;


    if (speed_hz == 0U)
    {
        Stepper_Stop();

        return;
    }


    if (speed_hz >
        JOYSTICK_MAX_HZ)
    {
        speed_hz =
            JOYSTICK_MAX_HZ;
    }


    if (speed_hz < 1U)
    {
        speed_hz = 1U;
    }


    period_us =
        TIM1_TIMER_TICK_HZ /
        speed_hz;


    if (period_us < 2U)
    {
        period_us = 2U;
    }


    arr =
        period_us - 1U;


    compare =
        period_us / 2U;


    if (compare == 0U)
    {
        compare = 1U;
    }


    TIM1_Pin_AlternateFunction();


    TIM1->ARR =
        arr;


    TIM1->CCR2 =
        compare;


    TIM1->CCMR1 &=
        ~(
            TIM_CCMR1_CC2S_Msk |
            TIM_CCMR1_OC2M_Msk
          );


    TIM1->CCMR1 |=
        (
            (6U <<
             TIM_CCMR1_OC2M_Pos) |
            TIM_CCMR1_OC2PE
        );


    TIM1->CCER |=
        TIM_CCER_CC2NE;


    TIM1->CCER &=
        ~TIM_CCER_CC2NP;


    TIM1->BDTR |=
        TIM_BDTR_MOE;


    TIM1->EGR =
        TIM_EGR_UG;


    TIM1->CNT =
        0U;


    TIM1->CR1 |=
        TIM_CR1_CEN;


    /*
     * Motor is now physically commanded to rotate.
     */

    current_speed_hz =
        speed_hz;

    motor_running =
        1U;
}


/* =========================================================
   SPEED RAMP
   ========================================================= */

static void Stepper_RampTo(
    uint32_t target_hz)
{
    uint32_t new_speed;


/* ---------------------------------------------------------
   STOP REQUEST
   --------------------------------------------------------- */

    if (target_hz == 0U)
    {
        if (current_speed_hz > 0U)
        {
            if (current_speed_hz >
                SPEED_RAMP_STEP_HZ)
            {
                new_speed =
                    current_speed_hz -
                    SPEED_RAMP_STEP_HZ;
            }
            else
            {
                new_speed = 0U;
            }


            if (new_speed == 0U)
            {
                Stepper_Stop();
            }
            else
            {
                Stepper_ApplySpeed(
                    new_speed);
            }
        }
        else
        {
            Stepper_Stop();
        }

        return;
    }


/* ---------------------------------------------------------
   ACCELERATE
   --------------------------------------------------------- */

    if (current_speed_hz <
        target_hz)
    {
        new_speed =
            current_speed_hz +
            SPEED_RAMP_STEP_HZ;


        if (new_speed >
            target_hz)
        {
            new_speed =
                target_hz;
        }
    }


/* ---------------------------------------------------------
   DECELERATE
   --------------------------------------------------------- */

    else if (current_speed_hz >
             target_hz)
    {
        if (current_speed_hz >
            SPEED_RAMP_STEP_HZ)
        {
            new_speed =
                current_speed_hz -
                SPEED_RAMP_STEP_HZ;
        }
        else
        {
            new_speed = 0U;
        }


        if (new_speed <
            target_hz)
        {
            new_speed =
                target_hz;
        }
    }


/* ---------------------------------------------------------
   ALREADY AT TARGET
   --------------------------------------------------------- */

    else
    {
        new_speed =
            current_speed_hz;
    }


    if (new_speed == 0U)
    {
        Stepper_Stop();
    }
    else
    {
        Stepper_ApplySpeed(
            new_speed);
    }
}


/* =========================================================
   DWT INITIALIZATION
   ========================================================= */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |=
        CoreDebug_DEMCR_TRCENA_Msk;


    DWT->CYCCNT = 0U;


    DWT->CTRL |=
        DWT_CTRL_CYCCNTENA_Msk;
}


/* =========================================================
   DWT MICROSECOND DELAY
   ========================================================= */

static void DWT_Delay_us(
    uint32_t us)
{
    uint32_t start;

    uint32_t cycles;


    start =
        DWT->CYCCNT;


    cycles =
        (SystemCoreClock /
         1000000U) *
        us;


    while ((DWT->CYCCNT - start) <
           cycles)
    {
        /* wait */
    }
}


/* =========================================================
   GPIO INITIALIZATION
   ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
        {0};


/* ---------------------------------------------------------
   GPIO CLOCKS
   --------------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();


/* ---------------------------------------------------------
   PB0 = STEP
   --------------------------------------------------------- */

    TIM1_Pin_OutputLow();


/* ---------------------------------------------------------
   PB1 = DIR
   --------------------------------------------------------- */

    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_1,
        GPIO_PIN_RESET);


    GPIO_InitStruct.Pin =
        GPIO_PIN_1;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;


    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct);


/* ---------------------------------------------------------
   PB8 = Encoder A
   PB9 = Encoder B
   --------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_8 |
        GPIO_PIN_9;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct);


/* ---------------------------------------------------------
   PC0 = LIMIT SWITCH
   Released = HIGH
   Pressed  = LOW
   --------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_0;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;


    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct);
}


/* =========================================================
   ADC1
   PA0 = ADC1_IN0
   ========================================================= */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig =
        {0};


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
        1U;

    hadc1.Init.DMAContinuousRequests =
        DISABLE;

    hadc1.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;


    if (HAL_ADC_Init(&hadc1)
        != HAL_OK)
    {
        Error_Handler();
    }


    sConfig.Channel =
        ADC_CHANNEL_0;

    sConfig.Rank =
        1;

    sConfig.SamplingTime =
        ADC_SAMPLETIME_84CYCLES;


    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   USART2
   PA2 = TX
   PA3 = RX
   115200 8-N-1
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


    if (HAL_UART_Init(&huart2)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   I2C1
   PB6 = SCL
   PB7 = SDA
   100 kHz
   ========================================================= */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance =
        I2C1;

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


    if (HAL_I2C_Init(&hi2c1)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   TIM1 INITIALIZATION
   PB0 = TIM1_CH2N

   Direct register configuration.
   No HAL TIM required.
   ========================================================= */

static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();


    TIM1->CR1 = 0U;


    TIM1->PSC =
        TIM1_PRESCALER;


    TIM1->ARR =
        9999U;


    TIM1->CCR2 =
        5000U;


    TIM1->CR1 |=
        TIM_CR1_ARPE;


    TIM1->CCMR1 =
        (
            (6U <<
             TIM_CCMR1_OC2M_Pos) |
            TIM_CCMR1_OC2PE
        );


    TIM1->CCER = 0U;


    TIM1->BDTR = 0U;


    TIM1->EGR =
        TIM_EGR_UG;


    TIM1->CR1 &=
        ~TIM_CR1_CEN;


    TIM1_Pin_OutputLow();
}


/* =========================================================
   SYSTEM CLOCK
   HSI = 16 MHz
   SYSCLK = 180 MHz
   APB1 = 45 MHz
   APB2 = 90 MHz
   TIM1 = 180 MHz
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
        {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct =
        {0};


/* ---------------------------------------------------------
   POWER CLOCK
   --------------------------------------------------------- */

    __HAL_RCC_PWR_CLK_ENABLE();


/* ---------------------------------------------------------
   VOLTAGE SCALE
   --------------------------------------------------------- */

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1);


/* ---------------------------------------------------------
   HSI
   --------------------------------------------------------- */

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


/* ---------------------------------------------------------
   PLL
   --------------------------------------------------------- */

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
            &RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }


/* ---------------------------------------------------------
   OVERDRIVE
   --------------------------------------------------------- */

    if (HAL_PWREx_EnableOverDrive()
        != HAL_OK)
    {
        Error_Handler();
    }


/* ---------------------------------------------------------
   CLOCK TREE
   --------------------------------------------------------- */

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
   ERROR HANDLER
   ========================================================= */

void Error_Handler(void)
{
    __disable_irq();


    while (1)
    {
    }
}
