// =========================================================
// X AXIS - RSBL35-24-HS SERVO CONTROLLER
// STM32F446RE
//
// SERIAL COMMAND CONTROL
//
// F = FORWARD  / CLOCKWISE
// B = BACKWARD / ANTICLOCKWISE
// S = STOP
//
// No joystick / ADC
//
// =========================================================
//
// CONNECTIONS
//
// PA8  = MAX485 DE + /RE
// PA9  = USART1 TX -> MAX485 DI
// PA10 = USART1 RX <- MAX485 RO
//
// PA2  = USART2 TX
// PA3  = USART2 RX
//
// PB8  = Encoder A
// PB9  = Encoder B
//
// PC0  = Limit switch
//
// PA5  = Existing output
//
// SERVO ID = 1
//
// =========================================================

#include "main.h"
#include "encoder.h"

#include <stdint.h>
#include <stdio.h>


/* =========================================================
   UART HANDLES
   ========================================================= */

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

#define SERVO_PRESENT_CURRENT_REG   69U

#define SERVO_INST_WRITE            0x03U
#define SERVO_INST_READ             0x02U


/* =========================================================
   MOTOR SPEED
   ========================================================= */

/*
 * F = +DEFAULT_SPEED
 * B = -DEFAULT_SPEED
 */

#define DEFAULT_SPEED               1000

#define MIN_SPEED                   300

#define MAX_SPEED                   1000


/* =========================================================
   SPEED RAMP
   ========================================================= */

#define SPEED_STEP                  50

#define SPEED_UPDATE_MS             10U


/* =========================================================
   DIRECTION CHANGE
   ========================================================= */

#define STOP_BEFORE_REVERSE_MS      100U


/* =========================================================
   MAIN LOOP
   ========================================================= */

#define LOOP_DELAY_MS               5U


/* =========================================================
   STATUS PRINT
   ========================================================= */

#define PRINT_INTERVAL_MS           200U


/* =========================================================
   SERVO CURRENT
   ========================================================= */

#define CURRENT_MA_PER_RAW          6.5f


/* =========================================================
   LIMIT SWITCH
   ========================================================= */

#define LIMIT_PORT                  GPIOC
#define LIMIT_PIN                   GPIO_PIN_0


/* =========================================================
   DIRECTION
   ========================================================= */

typedef enum
{
    DIR_STOP = 0,

    DIR_CLOCKWISE,

    DIR_ANTICLOCKWISE

} Direction_t;


/* =========================================================
   MOTOR VARIABLES
   ========================================================= */

/*
 * Current servo speed command.
 *
 * Positive = clockwise
 * Negative = anticlockwise
 */

static int16_t current_speed = 0;


/*
 * Requested speed.
 */

static int16_t target_speed = 0;


/*
 * Requested direction from serial.
 */

static Direction_t requested_direction =
    DIR_STOP;


/*
 * Direction currently active.
 */

static Direction_t active_direction =
    DIR_STOP;


/*
 * Reversal handling.
 */

static uint8_t reversing = 0U;

static uint32_t reverse_stop_time = 0U;


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART1_UART_Init(void);

static void MX_USART2_UART_Init(void);


/* =========================================================
   SERVO
   ========================================================= */

static void Servo_WriteReg8(
    uint8_t reg,
    uint8_t value
);

static void Servo_WriteReg16(
    uint8_t reg,
    uint16_t value
);

static HAL_StatusTypeDef Servo_ReadReg16(
    uint8_t reg,
    uint16_t *value
);

static uint16_t Servo_GetRawSpeedValue(
    int16_t speed
);

static void Servo_SetSpeed(
    int16_t speed
);

static void Servo_Init(void);


/* =========================================================
   CURRENT
   ========================================================= */

static HAL_StatusTypeDef Servo_ReadCurrent(
    uint16_t *current_raw,
    float *current_mA,
    float *current_A
);


/* =========================================================
   SERIAL
   ========================================================= */

static void Serial_CommandTask(void);

static void Serial_ProcessCharacter(
    uint8_t received
);

static void Serial_PrintHelp(void);

static void Serial_PrintStatus(void);


/* =========================================================
   LIMIT
   ========================================================= */

static int IsLimitPressed(void);


/* =========================================================
   DIRECTION STRING
   ========================================================= */

static const char *Direction_ToString(
    Direction_t direction
);


/* =========================================================
   PRINTF -> USART2
   ========================================================= */

int _write(
    int file,
    char *ptr,
    int len
)
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
   MAIN
   ========================================================= */

int main(void)
{
    HAL_Init();


    SystemClock_Config();


    MX_GPIO_Init();


    MX_USART1_UART_Init();


    MX_USART2_UART_Init();


    /* =====================================================
       ENCODER
       ===================================================== */

    Encoder_Init();

    Encoder_Reset();


    /* =====================================================
       SERVO INITIALIZATION
       ===================================================== */

    Servo_Init();


    /* =====================================================
       STARTUP MESSAGE
       ===================================================== */

    printf("\r\n");

    printf("=============================================\r\n");

    printf(
        " STM32F446RE X-AXIS SERVO CONTROLLER\r\n"
    );

    printf(
        " SERIAL F/B/S CONTROL\r\n"
    );

    printf("=============================================\r\n");


    printf(
        "F = FORWARD / CLOCKWISE\r\n"
    );

    printf(
        "B = BACKWARD / ANTICLOCKWISE\r\n"
    );

    printf(
        "S = STOP\r\n"
    );


    printf("---------------------------------------------\r\n");


    printf(
        "MAX485 DE+/RE  = PA8\r\n"
    );

    printf(
        "Servo TX       = PA9\r\n"
    );

    printf(
        "Servo RX       = PA10\r\n"
    );

    printf(
        "Serial TX      = PA2\r\n"
    );

    printf(
        "Serial RX      = PA3\r\n"
    );

    printf(
        "Encoder A      = PB8\r\n"
    );

    printf(
        "Encoder B      = PB9\r\n"
    );

    printf(
        "Limit Switch   = PC0\r\n"
    );


    printf("---------------------------------------------\r\n");


    printf(
        "Default Speed  = %d\r\n",
        DEFAULT_SPEED
    );


    printf(
        "Speed Range    = %d - %d\r\n",
        MIN_SPEED,
        MAX_SPEED
    );


    printf(
        "Speed Step     = %d\r\n",
        SPEED_STEP
    );


    printf(
        "Reverse Stop   = %lu ms\r\n",
        (unsigned long)
        STOP_BEFORE_REVERSE_MS
    );


    printf(
        "=============================================\r\n"
    );


    printf(
        "Controller ready\r\n"
    );


    printf(
        "Send F, B or S\r\n"
    );


    printf("\r\n");


    /* =====================================================
       TIMING
       ===================================================== */

    uint32_t last_speed_update =
        HAL_GetTick();


    uint32_t last_print_time =
        HAL_GetTick();


    /* =====================================================
       CURRENT VARIABLES
       ===================================================== */

    uint16_t servo_current_raw =
        0U;


    float servo_current_mA =
        0.0f;


    float servo_current_A =
        0.0f;


    HAL_StatusTypeDef current_status =
        HAL_ERROR;


    /* =====================================================
       ENCODER PRINT
       ===================================================== */

    int32_t previous_encoder_count =
        Encoder_GetCount();


    /* =====================================================
       MAIN LOOP
       ===================================================== */

    while (1)
    {
        uint32_t now =
            HAL_GetTick();


        /* =================================================
           1. SERIAL COMMAND
           ================================================= */

        Serial_CommandTask();


        /* =================================================
           2. ENCODER
           ================================================= */

        Encoder_Update();


        /* =================================================
           3. LIMIT SWITCH
           ================================================= */

        int limit_pressed =
            IsLimitPressed();


        /* =================================================
           4. LIMIT SWITCH
           ================================================= */

        if (limit_pressed)
        {
            /*
             * Limit always has priority.
             */

            target_speed =
                0;


            reversing =
                0U;


            requested_direction =
                DIR_STOP;


            active_direction =
                DIR_STOP;


            if (current_speed != 0)
            {
                current_speed =
                    0;


                Servo_SetSpeed(0);
            }
        }


        /* =================================================
           5. NORMAL MOTOR CONTROL
           ================================================= */

        else
        {
            /* =============================================
               STOP COMMAND
               ============================================= */

            if (target_speed == 0)
            {
                /*
                 * Immediate stop.
                 */

                if (current_speed != 0)
                {
                    current_speed =
                        0;


                    Servo_SetSpeed(0);
                }


                active_direction =
                    DIR_STOP;


                reversing =
                    0U;
            }


            /* =============================================
               DIRECTION CHANGE
               ============================================= */

            else if ((requested_direction !=
                      active_direction) &&
                     (current_speed != 0))
            {
                /*
                 * Motor is running in the opposite
                 * direction.
                 *
                 * Stop first.
                 */

                current_speed =
                    0;


                Servo_SetSpeed(0);


                active_direction =
                    DIR_STOP;


                reversing =
                    1U;


                reverse_stop_time =
                    now;
            }


            /* =============================================
               REVERSAL DELAY
               ============================================= */

            else if (reversing)
            {
                /*
                 * Keep motor stopped.
                 */

                if (current_speed != 0)
                {
                    current_speed =
                        0;


                    Servo_SetSpeed(0);
                }


                if ((now -
                     reverse_stop_time) >=
                    STOP_BEFORE_REVERSE_MS)
                {
                    reversing =
                        0U;


                    active_direction =
                        requested_direction;
                }
            }


            /* =============================================
               NORMAL OPERATION
               ============================================= */

            else
            {
                /*
                 * Direction is already active.
                 *
                 * Nothing else required here.
                 */
            }


            /* =============================================
               SPEED RAMP
               ============================================= */

            if ((now -
                 last_speed_update) >=
                SPEED_UPDATE_MS)
            {
                last_speed_update =
                    now;


                if (!reversing &&
                    (target_speed != 0))
                {
                    /* =====================================
                       CLOCKWISE
                       ===================================== */

                    if (target_speed > 0)
                    {
                        /*
                         * Ensure positive speed.
                         */

                        if (current_speed < 0)
                        {
                            current_speed =
                                0;
                        }


                        if (current_speed <
                            target_speed)
                        {
                            current_speed +=
                                SPEED_STEP;


                            if (current_speed >
                                target_speed)
                            {
                                current_speed =
                                    target_speed;
                            }
                        }
                        else if (current_speed >
                                 target_speed)
                        {
                            current_speed -=
                                SPEED_STEP;


                            if (current_speed <
                                target_speed)
                            {
                                current_speed =
                                    target_speed;
                            }
                        }
                    }


                    /* =====================================
                       ANTICLOCKWISE
                       ===================================== */

                    else
                    {
                        /*
                         * Ensure negative speed.
                         */

                        if (current_speed > 0)
                        {
                            current_speed =
                                0;
                        }


                        if (current_speed >
                            target_speed)
                        {
                            current_speed -=
                                SPEED_STEP;


                            if (current_speed <
                                target_speed)
                            {
                                current_speed =
                                    target_speed;
                            }
                        }
                        else if (current_speed <
                                 target_speed)
                        {
                            current_speed +=
                                SPEED_STEP;


                            if (current_speed >
                                target_speed)
                            {
                                current_speed =
                                    target_speed;
                            }
                        }
                    }


                    /* =====================================
                       SEND SPEED
                       ===================================== */

                    if (current_speed != 0)
                    {
                        Servo_SetSpeed(
                            current_speed);
                    }
                }
            }
        }


        /* =================================================
           6. STATUS PRINT
           ================================================= */

        if ((now -
             last_print_time) >=
            PRINT_INTERVAL_MS)
        {
            last_print_time =
                now;


            /* =============================================
               READ CURRENT
               ============================================= */

            current_status =
                Servo_ReadCurrent(
                    &servo_current_raw,
                    &servo_current_mA,
                    &servo_current_A
                );


            /* =============================================
               ENCODER
               ============================================= */

            int32_t encoder_count =
                Encoder_GetCount();


            int8_t encoder_direction =
                Encoder_GetDirection();


            int32_t encoder_position =
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


            /* =============================================
               ENCODER ROTATION DETECTION
               ============================================= */

            const char *encoder_state;


            if (encoder_count !=
                previous_encoder_count)
            {
                encoder_state =
                    "ROTATING";
            }
            else
            {
                encoder_state =
                    "NOT_ROTATING";
            }


            previous_encoder_count =
                encoder_count;


            /* =============================================
               SERVO STATE
               ============================================= */

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


            /* =============================================
               PRINT
               ============================================= */

            if (current_status ==
                HAL_OK)
            {
                printf(
                    "REQ=%s | "
                    "ACTIVE=%s | "
                    "TARGET=%d | "
                    "CURRENT_SPEED=%d | "

                    "ENC_A=%d | "
                    "ENC_B=%d | "
                    "ENC_COUNT=%ld | "
                    "ENC_DIR=%s | "
                    "ENC_POS=%ldmm | "
                    "ENCODER=%s | "

                    "LIMIT=%s | "
                    "SERVO=%s | "

                    "SERVO_CURRENT_RAW=%u | "
                    "CURRENT=%.2f mA | "
                    "CURRENT=%.4f A\r\n",

                    Direction_ToString(
                        requested_direction
                    ),

                    Direction_ToString(
                        active_direction
                    ),

                    target_speed,

                    current_speed,

                    (encoder_a ==
                     GPIO_PIN_SET)
                        ? 1
                        : 0,

                    (encoder_b ==
                     GPIO_PIN_SET)
                        ? 1
                        : 0,

                    (long)encoder_count,

                    (encoder_direction > 0)
                        ? "POS"
                        :
                      (encoder_direction < 0)
                        ? "NEG"
                        :
                          "STOP",

                    (long)encoder_position,

                    encoder_state,

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
                    "REQ=%s | "
                    "ACTIVE=%s | "
                    "TARGET=%d | "
                    "CURRENT_SPEED=%d | "

                    "ENC_A=%d | "
                    "ENC_B=%d | "
                    "ENC_COUNT=%ld | "
                    "ENC_DIR=%s | "
                    "ENC_POS=%ldmm | "
                    "ENCODER=%s | "

                    "LIMIT=%s | "
                    "SERVO=%s | "
                    "SERVO_CURRENT=READ_ERROR\r\n",

                    Direction_ToString(
                        requested_direction
                    ),

                    Direction_ToString(
                        active_direction
                    ),

                    target_speed,

                    current_speed,

                    (encoder_a ==
                     GPIO_PIN_SET)
                        ? 1
                        : 0,

                    (encoder_b ==
                     GPIO_PIN_SET)
                        ? 1
                        : 0,

                    (long)encoder_count,

                    (encoder_direction > 0)
                        ? "POS"
                        :
                      (encoder_direction < 0)
                        ? "NEG"
                        :
                          "STOP",

                    (long)encoder_position,

                    encoder_state,

                    limit_pressed
                        ? "PRESSED"
                        : "RELEASED",

                    servo_state
                );
            }
        }


        HAL_Delay(
            LOOP_DELAY_MS
        );
    }
}


/* =========================================================
   SERIAL COMMAND TASK
   =========================================================
 *
 * F, B and S are processed immediately.
 *
 * No ENTER required.
 *
 * Lowercase and uppercase both work.
 *
 * F -> clockwise
 * B -> anticlockwise
 * S -> stop
 *
 * CR/LF are ignored.
 * ========================================================= */

static void Serial_CommandTask(void)
{
    uint8_t received;


    while (__HAL_UART_GET_FLAG(
               &huart2,
               UART_FLAG_RXNE)
           != RESET)
    {
        received =
            (uint8_t)
            (huart2.Instance->DR &
             0xFFU);


        Serial_ProcessCharacter(
            received
        );
    }
}


/* =========================================================
   PROCESS ONE SERIAL CHARACTER
   ========================================================= */

static void Serial_ProcessCharacter(
    uint8_t received)
{
    /* =====================================================
       IGNORE ENTER / NEWLINE
       ===================================================== */

    if ((received == '\r') ||
        (received == '\n'))
    {
        return;
    }


    /* =====================================================
       IGNORE SPACE / TAB
       ===================================================== */

    if ((received == ' ') ||
        (received == '\t'))
    {
        return;
    }


    /* =====================================================
       LOWERCASE -> UPPERCASE
       ===================================================== */

    if ((received >= 'a') &&
        (received <= 'z'))
    {
        received =
            (uint8_t)
            (received - 'a' + 'A');
    }


    /* =====================================================
       FORWARD / CLOCKWISE
       ===================================================== */

    if (received == 'F')
    {
        if (IsLimitPressed())
        {
            printf(
                "F ignored - LIMIT PRESSED\r\n"
            );

            return;
        }


        requested_direction =
            DIR_CLOCKWISE;


        target_speed =
            DEFAULT_SPEED;


        printf(
            "F -> CLOCKWISE | "
            "TARGET=%d\r\n",
            DEFAULT_SPEED
        );


        return;
    }


    /* =====================================================
       BACKWARD / ANTICLOCKWISE
       ===================================================== */

    if (received == 'B')
    {
        if (IsLimitPressed())
        {
            printf(
                "B ignored - LIMIT PRESSED\r\n"
            );

            return;
        }


        requested_direction =
            DIR_ANTICLOCKWISE;


        target_speed =
            -DEFAULT_SPEED;


        printf(
            "B -> ANTICLOCKWISE | "
            "TARGET=%d\r\n",
            DEFAULT_SPEED
        );


        return;
    }


    /* =====================================================
       STOP
       ===================================================== */

    if (received == 'S')
    {
        /*
         * Immediate stop.
         */

        target_speed =
            0;


        current_speed =
            0;


        requested_direction =
            DIR_STOP;


        active_direction =
            DIR_STOP;


        reversing =
            0U;


        Servo_SetSpeed(0);


        printf(
            "S -> MOTOR STOPPED\r\n"
        );


        return;
    }


    /* =====================================================
       HELP
       ===================================================== */

    if (received == 'H')
    {
        Serial_PrintHelp();

        return;
    }


    /* =====================================================
       STATUS
       ===================================================== */

    if (received == '?')
    {
        Serial_PrintStatus();

        return;
    }


    /* =====================================================
       UNKNOWN
       ===================================================== */

    printf(
        "Unknown command: %c | "
        "Use F, B or S\r\n",
        received
    );
}


/* =========================================================
   HELP
   ========================================================= */

static void Serial_PrintHelp(void)
{
    printf("\r\n");

    printf(
        "=============================================\r\n"
    );

    printf(
        " SERIAL MOTOR COMMANDS\r\n"
    );

    printf(
        "=============================================\r\n"
    );

    printf(
        "F = FORWARD / CLOCKWISE\r\n"
    );

    printf(
        "B = BACKWARD / ANTICLOCKWISE\r\n"
    );

    printf(
        "S = STOP\r\n"
    );

    printf(
        "H = HELP\r\n"
    );

    printf(
        "? = STATUS\r\n"
    );

    printf(
        "=============================================\r\n"
    );

    printf("\r\n");
}


/* =========================================================
   STATUS
   ========================================================= */

static void Serial_PrintStatus(void)
{
    int32_t count =
        Encoder_GetCount();


    int8_t direction =
        Encoder_GetDirection();


    int32_t position =
        Encoder_GetPositionMM();


    printf("\r\n");


    printf(
        "Requested     = %s\r\n",
        Direction_ToString(
            requested_direction)
    );


    printf(
        "Active        = %s\r\n",
        Direction_ToString(
            active_direction)
    );


    printf(
        "Target Speed  = %d\r\n",
        target_speed
    );


    printf(
        "Current Speed = %d\r\n",
        current_speed
    );


    printf(
        "Motor         = %s\r\n",
        (current_speed != 0)
            ? "ROTATING"
            : "STOPPED"
    );


    printf(
        "Limit         = %s\r\n",
        IsLimitPressed()
            ? "PRESSED"
            : "RELEASED"
    );


    printf(
        "Encoder Count = %ld\r\n",
        (long)count
    );


    printf(
        "Encoder Dir   = %s\r\n",

        (direction > 0)
            ? "POS"
            :
          (direction < 0)
            ? "NEG"
            :
              "STOP"
    );


    printf(
        "Encoder Pos   = %ld mm\r\n",
        (long)position
    );


    printf(
        "Reversing     = %s\r\n",
        reversing
            ? "YES"
            : "NO"
    );


    printf("\r\n");
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
   SERVO INITIALIZATION
   ========================================================= */

static void Servo_Init(void)
{
    /*
     * Torque OFF
     */

    Servo_WriteReg8(
        SERVO_TORQUE_REG,
        0U
    );

    HAL_Delay(50);


    /*
     * Unlock EEPROM
     */

    Servo_WriteReg8(
        SERVO_LOCK_REG,
        0U
    );

    HAL_Delay(20);


    /*
     * Wheel / velocity mode
     */

    Servo_WriteReg8(
        SERVO_MODE_REG,
        1U
    );

    HAL_Delay(20);


    /*
     * Lock EEPROM
     */

    Servo_WriteReg8(
        SERVO_LOCK_REG,
        1U
    );

    HAL_Delay(20);


    /*
     * Acceleration
     */

    Servo_WriteReg8(
        SERVO_ACCELERATION_REG,
        50U
    );

    HAL_Delay(20);


    /*
     * Torque ON
     */

    Servo_WriteReg8(
        SERVO_TORQUE_REG,
        1U
    );

    HAL_Delay(20);


    /*
     * Stop
     */

    Servo_SetSpeed(0);

    HAL_Delay(100);
}


/* =========================================================
   SERVO WRITE 8-BIT
   ========================================================= */

static void Servo_WriteReg8(
    uint8_t reg,
    uint8_t value)
{
    uint8_t packet[8];


    packet[0] =
        0xFF;

    packet[1] =
        0xFF;

    packet[2] =
        SERVO_ID;

    packet[3] =
        0x04;

    packet[4] =
        SERVO_INST_WRITE;

    packet[5] =
        reg;

    packet[6] =
        value;


    uint16_t checksum_sum =
        SERVO_ID +
        packet[3] +
        packet[4] +
        packet[5] +
        packet[6];


    packet[7] =
        (uint8_t)
        (~checksum_sum);


    /* -----------------------------------------------------
       MAX485 TX
       ----------------------------------------------------- */

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


    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_TC)
           == RESET)
    {
    }


    HAL_Delay(1);


    /* -----------------------------------------------------
       MAX485 RX
       ----------------------------------------------------- */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );
}


/* =========================================================
   SERVO WRITE 16-BIT
   ========================================================= */

static void Servo_WriteReg16(
    uint8_t reg,
    uint16_t value)
{
    uint8_t packet[9];


    packet[0] =
        0xFF;

    packet[1] =
        0xFF;

    packet[2] =
        SERVO_ID;

    packet[3] =
        0x05;

    packet[4] =
        SERVO_INST_WRITE;

    packet[5] =
        reg;


    packet[6] =
        (uint8_t)
        (value & 0xFFU);


    packet[7] =
        (uint8_t)
        ((value >> 8U) & 0xFFU);


    uint16_t checksum_sum =
        SERVO_ID +
        packet[3] +
        packet[4] +
        packet[5] +
        packet[6] +
        packet[7];


    packet[8] =
        (uint8_t)
        (~checksum_sum);


    /* -----------------------------------------------------
       MAX485 TX
       ----------------------------------------------------- */

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


    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_TC)
           == RESET)
    {
    }


    HAL_Delay(1);


    /* -----------------------------------------------------
       MAX485 RX
       ----------------------------------------------------- */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );
}


/* =========================================================
   SERVO READ 16-BIT REGISTER
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
       Clear stale RX data
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

    tx_packet[0] =
        0xFF;

    tx_packet[1] =
        0xFF;

    tx_packet[2] =
        SERVO_ID;

    tx_packet[3] =
        0x04;

    tx_packet[4] =
        SERVO_INST_READ;

    tx_packet[5] =
        reg;

    tx_packet[6] =
        0x02;


    uint16_t checksum_sum =
        SERVO_ID +
        tx_packet[3] +
        tx_packet[4] +
        tx_packet[5] +
        tx_packet[6];


    tx_packet[7] =
        (uint8_t)
        (~checksum_sum);


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


    /* -----------------------------------------------------
       Wait until entire packet leaves UART
       ----------------------------------------------------- */

    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_TC)
           == RESET)
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
       RECEIVE RESPONSE
       ----------------------------------------------------- */

    status =
        HAL_UART_Receive(
            &huart1,
            rx_packet,
            sizeof(rx_packet),
            20
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
        (uint8_t)
        (~response_sum);


    if (rx_packet[7] !=
        expected_checksum)
    {
        return HAL_ERROR;
    }


    /* -----------------------------------------------------
       LOW BYTE FIRST
       ----------------------------------------------------- */

    *value =
        (uint16_t)
        rx_packet[5] |
        ((uint16_t)
         rx_packet[6] << 8U);


    return HAL_OK;
}


/* =========================================================
   READ SERVO CURRENT
   ========================================================= */

static HAL_StatusTypeDef Servo_ReadCurrent(
    uint16_t *current_raw,
    float *current_mA,
    float *current_A)
{
    uint16_t raw =
        0U;


    if ((current_raw == NULL) ||
        (current_mA == NULL) ||
        (current_A == NULL))
    {
        return HAL_ERROR;
    }


    HAL_StatusTypeDef status =
        Servo_ReadReg16(
            SERVO_PRESENT_CURRENT_REG,
            &raw
        );


    if (status != HAL_OK)
    {
        *current_raw =
            0U;

        *current_mA =
            0.0f;

        *current_A =
            0.0f;

        return status;
    }


    *current_raw =
        raw;


    *current_mA =
        (float)raw *
        CURRENT_MA_PER_RAW;


    *current_A =
        *current_mA /
        1000.0f;


    return HAL_OK;
}


/* =========================================================
   SPEED -> RAW SERVO VALUE
   =========================================================
//
// +1000 = 0x03E8
// -1000 = 0x83E8
//     0 = 0x0000
//
// ========================================================= */

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


    if (magnitude >
        MAX_SPEED)
    {
        magnitude =
            MAX_SPEED;
    }


    uint16_t raw_value =
        magnitude;


    if (speed < 0)
    {
        raw_value |=
            0x8000U;
    }


    return raw_value;
}


/* =========================================================
   SERVO SET SPEED
   ========================================================= */

static void Servo_SetSpeed(
    int16_t speed)
{
    uint16_t raw_value =
        Servo_GetRawSpeedValue(
            speed
        );


    Servo_WriteReg16(
        SERVO_GOAL_SPEED_L_REG,
        raw_value
    );
}


/* =========================================================
   GPIO
   ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
        {0};


    /* -----------------------------------------------------
       GPIO CLOCKS
       ----------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();


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
       LOW  = PRESSED
       HIGH = RELEASED
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


    /*
     * PB8/PB9 are configured by encoder.c.
     */
}


/* =========================================================
   USART1
   SERVO / MAX485
   1 Mbps
   ========================================================= */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance =
        USART1;


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


    if (HAL_UART_Init(
            &huart1)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   USART2
   PC SERIAL COMMAND PORT
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


    if (HAL_UART_Init(
            &huart2)
        != HAL_OK)
    {
        Error_Handler();
    }
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


    /* -----------------------------------------------------
       HSI
       ----------------------------------------------------- */

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;


    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;


    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    /* -----------------------------------------------------
       PLL
       ----------------------------------------------------- */

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


    /* -----------------------------------------------------
       OVERDRIVE
       ----------------------------------------------------- */

    if (HAL_PWREx_EnableOverDrive()
        != HAL_OK)
    {
        Error_Handler();
    }


    /* -----------------------------------------------------
       CLOCK TREE
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