// pa0 = joystick
// pa8 = max485 de+/re
// pc0 = limit switch

#include "main.h"
#include "encoder.h"

#include <stdint.h>
#include <stdio.h>

/* =========================================================
   HANDLES
   ========================================================= */
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* =========================================================
   SERVO SETTINGS
   ========================================================= */
#define SERVO_ID                    1U

#define SERVO_MODE_REG              33U
#define SERVO_TORQUE_REG            40U
#define SERVO_ACCELERATION_REG      41U
#define SERVO_GOAL_SPEED_L_REG      46U
#define SERVO_LOCK_REG              55U

/* Present current */
#define SERVO_PRESENT_CURRENT_REG   69U

#define SERVO_INST_WRITE            0x03U
#define SERVO_INST_READ             0x02U

/* =========================================================
   SERVO DIRECTION
   =========================================================

   Positive speed  -> 0x03xx -> CLOCKWISE
   Negative speed  -> 0x83xx -> ANTICLOCKWISE
   ========================================================= */
#define SERVO_DIRECTION_INVERT      0U

/* =========================================================
   JOYSTICK
   ========================================================= */
#define JOYSTICK_ADC_CHANNEL        ADC_CHANNEL_0

#define ADC_AVG_SAMPLES             8U
#define CENTER_SAMPLES              100U

#define JOYSTICK_DEADZONE           300U

#define MIN_SPEED                   300
#define MAX_SPEED                   1000

#define SPEED_STEP                  50

/* Actual observed joystick limits */
#define JOYSTICK_UP_MAX_ADC         3815
#define JOYSTICK_DOWN_MIN_ADC       3

/* =========================================================
   LIMIT SWITCH
   =========================================================

   PC0:
   LOW  = PRESSED
   HIGH = RELEASED

   Pressed = motor STOP in either direction.
   ========================================================= */
#define LIMIT_PORT                  GPIOC
#define LIMIT_PIN                   GPIO_PIN_0

/* =========================================================
   TIMING
   ========================================================= */
#define LOOP_DELAY_MS               20U
#define PRINT_INTERVAL_MS           200U
#define STOP_BEFORE_REVERSE_MS      100U

/* =========================================================
   CURRENT CONVERSION
   =========================================================

   1 raw current unit = 6.5 mA
   ========================================================= */
#define CURRENT_MA_PER_RAW          6.5f

/* =========================================================
   DIRECTION ENUM
   ========================================================= */
typedef enum
{
    DIR_STOP = 0,
    DIR_CLOCKWISE,
    DIR_ANTICLOCKWISE
} Direction_t;

/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

/* Servo write */
static void Servo_WriteReg8(
    uint8_t reg,
    uint8_t value
);

static void Servo_WriteReg16(
    uint8_t reg,
    uint16_t value
);

/* Servo read */
static HAL_StatusTypeDef Servo_ReadReg16(
    uint8_t reg,
    uint16_t *value
);

/* Servo */
static uint16_t Servo_GetRawSpeedValue(
    int16_t speed
);

static void Servo_SetSpeed(
    int16_t speed
);

static void Servo_Init(void);

/* Current */
static HAL_StatusTypeDef Servo_ReadCurrent(
    uint16_t *current_raw,
    float *current_mA,
    float *current_A
);

/* Joystick */
static uint16_t Joystick_ReadAverage(void);

static uint16_t Joystick_CalibrateCenter(void);

static int16_t Joystick_GetTargetSpeed(
    uint16_t adc,
    uint16_t center,
    Direction_t *requested_direction
);

/* Limit */
static int IsLimitPressed(void);

/* Strings */
static const char *Direction_ToString(
    Direction_t direction
);

static const char *EncoderDirection_ToString(
    int8_t direction
);

/* printf */
int _write(
    int file,
    char *ptr,
    int len
);

/* =========================================================
   MAIN
   ========================================================= */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();

    /* =====================================================
       ENCODER
       ===================================================== */
    Encoder_Init();
    Encoder_Reset();

    printf("\r\n");
    printf("=============================================\r\n");
    printf(" STM32F446RE X-AXIS SERVO CONTROLLER\r\n");
    printf("=============================================\r\n");
    printf("Joystick       = PA0\r\n");
    printf("MAX485 DE+/RE  = PA8\r\n");
    printf("Servo TX       = PA9\r\n");
    printf("Servo RX       = PA10\r\n");
    printf("Encoder A      = PB8\r\n");
    printf("Encoder B      = PB9\r\n");
    printf("Limit Switch   = PC0\r\n");
    printf("Current FB     = Servo Register 69/70\r\n");
    printf("=============================================\r\n");

    /* =====================================================
       SERVO INITIALIZATION
       ===================================================== */
    Servo_Init();

    HAL_Delay(500);

    /* =====================================================
       JOYSTICK CENTER
       ===================================================== */
    uint16_t joystick_center =
        Joystick_CalibrateCenter();

    printf(
        "Joystick center = %u\r\n",
        joystick_center
    );

    printf("Controller ready\r\n");
    printf("\r\n");

    /* =====================================================
       CONTROL VARIABLES
       ===================================================== */
    int16_t current_speed = 0;
    int16_t target_speed = 0;
    int16_t applied_target_speed = 0;

    uint32_t last_print_time =
        HAL_GetTick();

    uint32_t reverse_stop_time = 0;

    uint8_t reversing = 0U;

    Direction_t requested_direction =
        DIR_STOP;

    /* =====================================================
       SERVO CURRENT VARIABLES
       ===================================================== */
    uint16_t servo_current_raw = 0U;

    float servo_current_mA = 0.0f;
    float servo_current_A = 0.0f;

    HAL_StatusTypeDef current_status =
        HAL_ERROR;

    /* =====================================================
       MAIN LOOP
       ===================================================== */
    while (1)
    {
        /* -------------------------------------------------
           READ JOYSTICK
           ------------------------------------------------- */
        uint16_t adc_value =
            Joystick_ReadAverage();

        /* -------------------------------------------------
           JOYSTICK TARGET
           ------------------------------------------------- */
        target_speed =
            Joystick_GetTargetSpeed(
                adc_value,
                joystick_center,
                &requested_direction
            );

        /* -------------------------------------------------
           LIMIT SWITCH

           PRESSED:
               stop motor in BOTH directions

           RELEASED:
               normal joystick operation

           Nothing else is changed.
           ------------------------------------------------- */
        int limit_pressed =
            IsLimitPressed();

        if (limit_pressed)
        {
            target_speed = 0;
            current_speed = 0;
            applied_target_speed = 0;
            reversing = 0U;

            Servo_SetSpeed(0);
        }
        else
        {
            /* -------------------------------------------------
               STOP BEFORE REVERSAL
               ------------------------------------------------- */
            if ((target_speed != 0) &&
                (current_speed != 0) &&
                ((target_speed > 0) !=
                 (current_speed > 0)))
            {
                target_speed = 0;

                if (!reversing)
                {
                    reversing = 1U;

                    reverse_stop_time =
                        HAL_GetTick();
                }
            }

            /* -------------------------------------------------
               REVERSAL DELAY
               ------------------------------------------------- */
            if (reversing)
            {
                if (current_speed == 0)
                {
                    if ((HAL_GetTick() -
                         reverse_stop_time) >=
                        STOP_BEFORE_REVERSE_MS)
                    {
                        reversing = 0U;
                    }
                }
            }

            if (reversing)
            {
                applied_target_speed = 0;
            }
            else
            {
                applied_target_speed =
                    target_speed;
            }

            /* -------------------------------------------------
               SPEED RAMP
               ------------------------------------------------- */
            if (current_speed <
                applied_target_speed)
            {
                current_speed += SPEED_STEP;

                if (current_speed >
                    applied_target_speed)
                {
                    current_speed =
                        applied_target_speed;
                }
            }
            else if (current_speed >
                     applied_target_speed)
            {
                current_speed -= SPEED_STEP;

                if (current_speed <
                    applied_target_speed)
                {
                    current_speed =
                        applied_target_speed;
                }
            }

            /* -------------------------------------------------
               SEND SPEED
               ------------------------------------------------- */
            Servo_SetSpeed(current_speed);
        }

        /* =====================================================
           PRINT EVERY 200 ms
           ===================================================== */
        if ((HAL_GetTick() -
             last_print_time) >=
            PRINT_INTERVAL_MS)
        {
            last_print_time =
                HAL_GetTick();

            /* -------------------------------------------------
               READ CURRENT FROM SERVO
               ------------------------------------------------- */
            current_status =
                Servo_ReadCurrent(
                    &servo_current_raw,
                    &servo_current_mA,
                    &servo_current_A
                );

            /* -------------------------------------------------
               ENCODER
               ------------------------------------------------- */
            int32_t encoder_count =
                Encoder_GetCount();

            int8_t encoder_direction =
                Encoder_GetDirection();

            int32_t encoder_position_mm =
                Encoder_GetPositionMM();

            GPIO_PinState encoder_a =
                HAL_GPIO_ReadPin(
                    GPIOB,
                    GPIO_PIN_8
                );

            GPIO_PinState encoder_b =
                HAL_GPIO_ReadPin(
                    GPIOB,
                    GPIO_PIN_9
                );

            /* -------------------------------------------------
               JOYSTICK OFFSET
               ------------------------------------------------- */
            int32_t offset =
                (int32_t)adc_value -
                (int32_t)joystick_center;

            /* -------------------------------------------------
               RAW SPEED
               ------------------------------------------------- */
            uint16_t raw_speed =
                Servo_GetRawSpeedValue(
                    current_speed
                );

            /* -------------------------------------------------
               SERVO STATE
               ------------------------------------------------- */
            const char *servo_state;

            if (limit_pressed)
            {
                servo_state =
                    "LIMIT_STOP";
            }
            else if (current_speed == 0)
            {
                servo_state =
                    "STOPPED";
            }
            else
            {
                servo_state =
                    "ROTATING";
            }

            /* -------------------------------------------------
               PRINT
               ------------------------------------------------- */
            if (current_status == HAL_OK)
            {
                printf(
                    "ADC=%4u | "
                    "CENTER=%4u | "
                    "OFFSET=%5ld | "

                    "TARGET=%5d | "
                    "CURRENT=%5d | "

                    "REQ=%s | "
                    "RAW=0x%04X | "

                    "ENC_A=%d | "
                    "ENC_B=%d | "

                    "ENC_COUNT=%ld | "
                    "ENC_DIR=%s | "
                    "POS=%ldmm | "

                    "LIMIT=%s | "
                    "SERVO=%s | "

                    "SERVO_CURRENT_RAW=%u | "
                    "CURRENT=%.2f mA | "
                    "CURRENT=%.4f A\r\n",

                    adc_value,

                    joystick_center,

                    (long)offset,

                    target_speed,

                    current_speed,

                    Direction_ToString(
                        requested_direction
                    ),

                    raw_speed,

                    (encoder_a ==
                     GPIO_PIN_SET) ? 1 : 0,

                    (encoder_b ==
                     GPIO_PIN_SET) ? 1 : 0,

                    (long)encoder_count,

                    EncoderDirection_ToString(
                        encoder_direction
                    ),

                    (long)encoder_position_mm,

                    limit_pressed
                        ? "PRESSED"
                        : "RELEASED",

                    servo_state,

                    servo_current_raw,

                    (double)servo_current_mA,

                    (double)servo_current_A
                );
            }
            else
            {
                printf(
                    "ADC=%4u | "
                    "CENTER=%4u | "
                    "OFFSET=%5ld | "

                    "TARGET=%5d | "
                    "CURRENT=%5d | "

                    "REQ=%s | "
                    "RAW=0x%04X | "

                    "ENC_A=%d | "
                    "ENC_B=%d | "

                    "ENC_COUNT=%ld | "
                    "ENC_DIR=%s | "
                    "POS=%ldmm | "

                    "LIMIT=%s | "
                    "SERVO=%s | "

                    "SERVO_CURRENT=READ_ERROR\r\n",

                    adc_value,

                    joystick_center,

                    (long)offset,

                    target_speed,

                    current_speed,

                    Direction_ToString(
                        requested_direction
                    ),

                    raw_speed,

                    (encoder_a ==
                     GPIO_PIN_SET) ? 1 : 0,

                    (encoder_b ==
                     GPIO_PIN_SET) ? 1 : 0,

                    (long)encoder_count,

                    EncoderDirection_ToString(
                        encoder_direction
                    ),

                    (long)encoder_position_mm,

                    limit_pressed
                        ? "PRESSED"
                        : "RELEASED",

                    servo_state
                );
            }
        }

        HAL_Delay(LOOP_DELAY_MS);
    }
}

/* =========================================================
   SERVO INITIALIZATION
   ========================================================= */
static void Servo_Init(void)
{
    /* Torque OFF */
    Servo_WriteReg8(
        SERVO_TORQUE_REG,
        0U
    );

    HAL_Delay(50);

    /* Unlock EEPROM */
    Servo_WriteReg8(
        SERVO_LOCK_REG,
        0U
    );

    HAL_Delay(20);

    /* Wheel / velocity mode */
    Servo_WriteReg8(
        SERVO_MODE_REG,
        1U
    );

    HAL_Delay(20);

    /* Lock EEPROM */
    Servo_WriteReg8(
        SERVO_LOCK_REG,
        1U
    );

    HAL_Delay(20);

    /* Acceleration */
    Servo_WriteReg8(
        SERVO_ACCELERATION_REG,
        50U
    );

    HAL_Delay(20);

    /* Torque ON */
    Servo_WriteReg8(
        SERVO_TORQUE_REG,
        1U
    );

    HAL_Delay(20);

    /* Stop */
    Servo_SetSpeed(0);

    HAL_Delay(100);
}

/* =========================================================
   WRITE 8-BIT SERVO REGISTER
   ========================================================= */
static void Servo_WriteReg8(
    uint8_t reg,
    uint8_t value)
{
    uint8_t packet[8];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = SERVO_ID;
    packet[3] = 0x04;
    packet[4] = SERVO_INST_WRITE;
    packet[5] = reg;
    packet[6] = value;

    uint16_t checksum_sum =
        SERVO_ID +
        packet[3] +
        packet[4] +
        packet[5] +
        packet[6];

    packet[7] =
        (uint8_t)(~checksum_sum);

    /* MAX485 TX */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_SET
    );

    HAL_Delay(1);

    HAL_UART_Transmit(
        &huart1,
        packet,
        sizeof(packet),
        100
    );

    HAL_Delay(1);

    /* MAX485 RX */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );
}

/* =========================================================
   WRITE 16-BIT SERVO REGISTER
   ========================================================= */
static void Servo_WriteReg16(
    uint8_t reg,
    uint16_t value)
{
    uint8_t packet[9];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = SERVO_ID;
    packet[3] = 0x05;
    packet[4] = SERVO_INST_WRITE;
    packet[5] = reg;

    packet[6] =
        (uint8_t)(value & 0xFFU);

    packet[7] =
        (uint8_t)(
            (value >> 8U) & 0xFFU
        );

    uint16_t checksum_sum =
        SERVO_ID +
        packet[3] +
        packet[4] +
        packet[5] +
        packet[6] +
        packet[7];

    packet[8] =
        (uint8_t)(~checksum_sum);

    /* MAX485 TX */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_SET
    );

    HAL_Delay(1);

    HAL_UART_Transmit(
        &huart1,
        packet,
        sizeof(packet),
        100
    );

    HAL_Delay(1);

    /* MAX485 RX */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );
}

/* =========================================================
   READ 16-BIT SERVO REGISTER
   ========================================================= */
static HAL_StatusTypeDef Servo_ReadReg16(
    uint8_t reg,
    uint16_t *value)
{
    uint8_t tx_packet[8];
    uint8_t rx_packet[8];

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    /* -----------------------------------------------------
       Clear stale RX bytes
       ----------------------------------------------------- */
    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_RXNE))
    {
        (void)huart1.Instance->DR;
    }

    /* -----------------------------------------------------
       READ PACKET
       ----------------------------------------------------- */
    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = SERVO_ID;
    tx_packet[3] = 0x04;
    tx_packet[4] = SERVO_INST_READ;
    tx_packet[5] = reg;
    tx_packet[6] = 0x02;

    uint16_t checksum_sum =
        SERVO_ID +
        tx_packet[3] +
        tx_packet[4] +
        tx_packet[5] +
        tx_packet[6];

    tx_packet[7] =
        (uint8_t)(~checksum_sum);

    /* -----------------------------------------------------
       MAX485 TX
       ----------------------------------------------------- */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_SET
    );

    HAL_Delay(1);

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            &huart1,
            tx_packet,
            sizeof(tx_packet),
            100
        );

    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(
            GPIOA,
            GPIO_PIN_8,
            GPIO_PIN_RESET
        );

        return status;
    }

    /* Wait for last byte to leave UART */
    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_TC) == RESET)
    {
    }

    /* -----------------------------------------------------
       MAX485 RX
       ----------------------------------------------------- */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );

    /* -----------------------------------------------------
       RECEIVE
       ----------------------------------------------------- */
    status =
        HAL_UART_Receive(
            &huart1,
            rx_packet,
            sizeof(rx_packet),
            100
        );

    if (status != HAL_OK)
    {
        return status;
    }

    /* -----------------------------------------------------
       HEADER
       ----------------------------------------------------- */
    if ((rx_packet[0] != 0xFFU) ||
        (rx_packet[1] != 0xFFU))
    {
        return HAL_ERROR;
    }

    /* -----------------------------------------------------
       ID
       ----------------------------------------------------- */
    if (rx_packet[2] != SERVO_ID)
    {
        return HAL_ERROR;
    }

    /* -----------------------------------------------------
       LENGTH
       ----------------------------------------------------- */
    if (rx_packet[3] != 0x04U)
    {
        return HAL_ERROR;
    }

    /* -----------------------------------------------------
       ERROR
       ----------------------------------------------------- */
    if (rx_packet[4] != 0x00U)
    {
        return HAL_ERROR;
    }

    /* -----------------------------------------------------
       CHECKSUM
       ----------------------------------------------------- */
    uint16_t response_sum =
        rx_packet[2] +
        rx_packet[3] +
        rx_packet[4] +
        rx_packet[5] +
        rx_packet[6];

    uint8_t expected_checksum =
        (uint8_t)(~response_sum);

    if (rx_packet[7] != expected_checksum)
    {
        return HAL_ERROR;
    }

    /* -----------------------------------------------------
       LOW BYTE FIRST
       ----------------------------------------------------- */
    *value =
        (uint16_t)rx_packet[5] |
        ((uint16_t)rx_packet[6] << 8U);

    return HAL_OK;
}

/* =========================================================
   READ PRESENT CURRENT
   ========================================================= */
static HAL_StatusTypeDef Servo_ReadCurrent(
    uint16_t *current_raw,
    float *current_mA,
    float *current_A)
{
    if ((current_raw == NULL) ||
        (current_mA == NULL) ||
        (current_A == NULL))
    {
        return HAL_ERROR;
    }

    uint16_t raw = 0U;

    HAL_StatusTypeDef status =
        Servo_ReadReg16(
            SERVO_PRESENT_CURRENT_REG,
            &raw
        );

    if (status != HAL_OK)
    {
        *current_raw = 0U;
        *current_mA = 0.0f;
        *current_A = 0.0f;

        return status;
    }

    *current_raw = raw;

    *current_mA =
        (float)raw *
        CURRENT_MA_PER_RAW;

    *current_A =
        *current_mA / 1000.0f;

    return HAL_OK;
}

/* =========================================================
   SPEED -> RAW SERVO VALUE
   =========================================================

   +1000 -> 0x03E8
   -1000 -> 0x83E8
   ========================================================= */
static uint16_t Servo_GetRawSpeedValue(
    int16_t speed)
{
    if (speed == 0)
    {
        return 0U;
    }

    uint16_t magnitude;

    if (speed < 0)
    {
        magnitude =
            (uint16_t)(-speed);
    }
    else
    {
        magnitude =
            (uint16_t)speed;
    }

    if (magnitude > MAX_SPEED)
    {
        magnitude = MAX_SPEED;
    }

    uint16_t raw_value =
        magnitude;

    if (speed < 0)
    {
        raw_value |= 0x8000U;
    }

    return raw_value;
}

/* =========================================================
   SEND SERVO SPEED
   ========================================================= */
static void Servo_SetSpeed(
    int16_t speed)
{
    uint16_t raw_value =
        Servo_GetRawSpeedValue(speed);

    Servo_WriteReg16(
        SERVO_GOAL_SPEED_L_REG,
        raw_value
    );
}

/* =========================================================
   READ JOYSTICK AVERAGE
   ========================================================= */
static uint16_t Joystick_ReadAverage(void)
{
    uint32_t sum = 0U;

    for (uint32_t i = 0U;
         i < ADC_AVG_SAMPLES;
         i++)
    {
        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(
                &hadc1,
                10) == HAL_OK)
        {
            sum +=
                HAL_ADC_GetValue(&hadc1);
        }

        HAL_ADC_Stop(&hadc1);
    }

    return (uint16_t)
        (sum / ADC_AVG_SAMPLES);
}

/* =========================================================
   CALIBRATE JOYSTICK CENTER
   ========================================================= */
static uint16_t Joystick_CalibrateCenter(void)
{
    uint32_t sum = 0U;

    printf(
        "Calibrating joystick center...\r\n"
    );

    printf(
        "Keep joystick at CENTER\r\n"
    );

    HAL_Delay(500);

    for (uint32_t i = 0U;
         i < CENTER_SAMPLES;
         i++)
    {
        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(
                &hadc1,
                10) == HAL_OK)
        {
            sum +=
                HAL_ADC_GetValue(&hadc1);
        }

        HAL_ADC_Stop(&hadc1);

        HAL_Delay(5);
    }

    return (uint16_t)
        (sum / CENTER_SAMPLES);
}

/* =========================================================
   JOYSTICK TARGET SPEED

   UP:
       positive speed
       CLOCKWISE
       0x03xx

   CENTER:
       STOP
       0x0000

   DOWN:
       negative speed
       ANTICLOCKWISE
       0x83xx
   ========================================================= */
static int16_t Joystick_GetTargetSpeed(
    uint16_t adc,
    uint16_t center,
    Direction_t *requested_direction)
{
    int32_t offset =
        (int32_t)adc -
        (int32_t)center;

    /* -----------------------------------------------------
       CENTER
       ----------------------------------------------------- */
    if ((offset <=
         (int32_t)JOYSTICK_DEADZONE) &&
        (offset >=
         -(int32_t)JOYSTICK_DEADZONE))
    {
        *requested_direction =
            DIR_STOP;

        return 0;
    }

    /* =====================================================
       UP -> CLOCKWISE
       ===================================================== */
    if (offset >
        (int32_t)JOYSTICK_DEADZONE)
    {
        *requested_direction =
            DIR_CLOCKWISE;

        int32_t start =
            (int32_t)center +
            (int32_t)JOYSTICK_DEADZONE;

        int32_t range =
            (int32_t)JOYSTICK_UP_MAX_ADC -
            start;

        if (range <= 0)
        {
            range = 1;
        }

        int32_t effective =
            (int32_t)adc -
            start;

        if (effective < 0)
        {
            effective = 0;
        }

        if (effective > range)
        {
            effective = range;
        }

        int32_t speed =
            (effective * MAX_SPEED) /
            range;

        if (speed < MIN_SPEED)
        {
            speed = MIN_SPEED;
        }

        if (speed > MAX_SPEED)
        {
            speed = MAX_SPEED;
        }

        /*
           POSITIVE = CLOCKWISE
        */
        return (int16_t)speed;
    }

    /* =====================================================
       DOWN -> ANTICLOCKWISE
       ===================================================== */
    *requested_direction =
        DIR_ANTICLOCKWISE;

    int32_t start =
        (int32_t)center -
        (int32_t)JOYSTICK_DEADZONE;

    int32_t range =
        start -
        (int32_t)JOYSTICK_DOWN_MIN_ADC;

    if (range <= 0)
    {
        range = 1;
    }

    int32_t effective =
        start -
        (int32_t)adc;

    if (effective < 0)
    {
        effective = 0;
    }

    if (effective > range)
    {
        effective = range;
    }

    int32_t speed =
        (effective * MAX_SPEED) /
        range;

    if (speed < MIN_SPEED)
    {
        speed = MIN_SPEED;
    }

    if (speed > MAX_SPEED)
    {
        speed = MAX_SPEED;
    }

    /*
       NEGATIVE = ANTICLOCKWISE
    */
    return (int16_t)(-speed);
}

/* =========================================================
   LIMIT SWITCH
   ========================================================= */
static int IsLimitPressed(void)
{
    return
        (HAL_GPIO_ReadPin(
            LIMIT_PORT,
            LIMIT_PIN
        ) == GPIO_PIN_RESET);
}

/* =========================================================
   DIRECTION STRING
   ========================================================= */
static const char *Direction_ToString(
    Direction_t direction)
{
    switch (direction)
    {
        case DIR_CLOCKWISE:
            return "CLOCKWISE";

        case DIR_ANTICLOCKWISE:
            return "ANTICLOCKWISE";

        default:
            return "STOP";
    }
}

/* =========================================================
   ENCODER DIRECTION STRING
   ========================================================= */
static const char *EncoderDirection_ToString(
    int8_t direction)
{
    if (direction > 0)
    {
        return "POS";
    }

    if (direction < 0)
    {
        return "NEG";
    }

    return "STOP";
}

/* =========================================================
   PRINTF -> USART2
   ========================================================= */
int _write(
    int file,
    char *ptr,
    int len)
{
    (void)file;

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)ptr,
        (uint16_t)len,
        HAL_MAX_DELAY
    );

    return len;
}

/* =========================================================
   ADC1
   ========================================================= */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig =
        {0};

    hadc1.Instance = ADC1;

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

    sConfig.Channel =
        JOYSTICK_ADC_CHANNEL;

    sConfig.Rank = 1;

    sConfig.SamplingTime =
        ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   USART1
   1 Mbps
   ========================================================= */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;

    huart1.Init.BaudRate =
        1000000;

    huart1.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart1.Init.StopBits =
        UART_STOPBITS_1;

    huart1.Init.Parity =
        UART_PARITY_NONE;

    huart1.Init.Mode =
        UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart1.Init.OverSampling =
        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1)
        != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   USART2
   115200
   ========================================================= */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

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
   GPIO
   ========================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
        {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* -----------------------------------------------------
       PA0 = JOYSTICK ADC
       ----------------------------------------------------- */
    GPIO_InitStruct.Pin =
        GPIO_PIN_0;

    GPIO_InitStruct.Mode =
        GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );

    /* -----------------------------------------------------
       PA5 = EXISTING OUTPUT
       ----------------------------------------------------- */
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

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_5,
        GPIO_PIN_RESET
    );

    /* -----------------------------------------------------
       PA8 = MAX485 DE + /RE
       ----------------------------------------------------- */
    GPIO_InitStruct.Pin =
        GPIO_PIN_8;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );

    /* -----------------------------------------------------
       PC0 = LIMIT SWITCH
       ----------------------------------------------------- */
    GPIO_InitStruct.Pin =
        GPIO_PIN_0;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );
}

/* =========================================================
   SYSTEM CLOCK
   ========================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
        {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct =
        {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    /* HSI */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    /* PLL */
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

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive()
        != HAL_OK)
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