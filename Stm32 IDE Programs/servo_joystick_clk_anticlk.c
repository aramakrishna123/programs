#include "main.h"
#include <stdint.h>
#include <stdio.h>

/*
 * ============================================================
 * STM32F446RE + HW-504 + MAX485 + RSBL35-24-HS
 * ============================================================
 *
 * JOYSTICK:
 *
 * VRY -> PA0
 *
 * UP       -> CLOCKWISE
 * CENTER   -> STOP
 * DOWN     -> ANTICLOCKWISE
 *
 *
 * MAX485:
 *
 * PA8  -> DE + /RE
 * PA9  -> DI / USART1_TX
 * PA10 -> RO / USART1_RX
 *
 *
 * SERIAL MONITOR:
 *
 * PA2 -> USART2_TX
 * PA3 -> USART2_RX
 *
 * USART2 = 115200 8-N-1
 *
 *
 * SERVO:
 *
 * ID = 1
 *
 * Wheel Mode       = Register 33 = 1
 * Torque           = Register 40
 * Acceleration     = Register 41
 * Goal Speed       = Register 46
 * EEPROM Lock      = Register 55
 *
 * Feedback:
 *
 * Present Speed    = Register 58-59
 * Moving           = Register 66
 *
 * ============================================================
 */


/* ============================================================
 * HANDLES
 * ============================================================ */

ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;


/* ============================================================
 * MAX485
 * ============================================================ */

#define RS485_DE_PORT          GPIOA
#define RS485_DE_PIN           GPIO_PIN_8


/* ============================================================
 * SERVO REGISTERS
 * ============================================================ */

#define SERVO_ID               1U

#define REG_MODE               33U
#define REG_TORQUE             40U
#define REG_ACCELERATION       41U
#define REG_GOAL_SPEED_L       46U
#define REG_LOCK               55U

#define REG_PRESENT_SPEED_L    58U
#define REG_MOVING             66U

#define INST_WRITE             0x03U
#define INST_READ              0x02U


/* ============================================================
 * SPEED SETTINGS
 * ============================================================ */

#define MAX_CW_SPEED           1000U
#define MAX_CCW_SPEED          1000U

#define MIN_CW_SPEED           300U
#define MIN_CCW_SPEED          300U

#define JOYSTICK_DEADZONE      300U

#define ADC_FILTER_SAMPLES     8U
#define CENTER_SAMPLES         100U

#define SPEED_STEP             50

#define STOP_BEFORE_REVERSE_MS 100U


/* ============================================================
 * IMPORTANT DIRECTION SETTING
 *
 * Your physical test showed:
 *
 * servo raw negative command -> CLOCKWISE
 * servo raw positive command -> not rotating
 *
 * Therefore keep this enabled so:
 *
 * UP     software +speed -> raw negative command
 * DOWN   software -speed -> raw positive command
 *
 * ============================================================ */

#define SERVO_DIRECTION_INVERT 1U


/* ============================================================
 * DEBUG PRINT INTERVAL
 * ============================================================ */

#define DEBUG_PRINT_INTERVAL   200U


/* ============================================================
 * FUNCTION PROTOTYPES
 * ============================================================ */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

static uint16_t Joystick_Read(void);
static uint16_t Joystick_Read_Average(uint16_t samples);
static uint16_t Joystick_CalibrateCenter(void);

static void RS485_TX_Mode(void);
static void RS485_RX_Mode(void);

static uint8_t Servo_Checksum(
    uint8_t *packet,
    uint8_t length
);

static void Servo_FlushRx(void);

static HAL_StatusTypeDef Servo_Write(
    uint8_t id,
    uint8_t address,
    uint8_t *data,
    uint8_t data_length
);

static HAL_StatusTypeDef Servo_Read(
    uint8_t id,
    uint8_t address,
    uint8_t data_length,
    uint8_t *data
);

static int16_t Servo_ReadPresentSpeed(void);

static uint8_t Servo_ReadMoving(void);

static uint16_t Servo_GetRawSpeedValue(
    int16_t speed
);

static void Servo_Torque(uint8_t enable);

static void Servo_SetWheelMode(void);

static void Servo_SetAcceleration(
    uint8_t acceleration
);

static void Servo_SetSpeed(
    int16_t speed
);


/* ============================================================
 * PRINTF REDIRECT
 *
 * USART2 -> Serial Monitor
 * ============================================================ */

int _write(int file,
           char *ptr,
           int len)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)ptr,
        (uint16_t)len,
        HAL_MAX_DELAY
    );

    return len;
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_USART1_UART_Init();

    MX_USART2_UART_Init();

    MX_ADC1_Init();


    /* ========================================================
     * SERIAL STARTUP MESSAGE
     * ======================================================== */

    printf("\r\n");
    printf("==============================================\r\n");
    printf("RSBL35-24-HS JOYSTICK SERVO CONTROL\r\n");
    printf("==============================================\r\n");
    printf("Servo ID       : %d\r\n", SERVO_ID);
    printf("Joystick       : VRY -> PA0\r\n");
    printf("UP             : CLOCKWISE\r\n");
    printf("CENTER         : STOP\r\n");
    printf("DOWN           : ANTICLOCKWISE\r\n");
    printf("USART1         : 1000000 baud\r\n");
    printf("USART2         : 115200 baud\r\n");
    printf("Direction Inv. : %d\r\n",
           SERVO_DIRECTION_INVERT);
    printf("==============================================\r\n");


    /* ========================================================
     * MAX485 RECEIVE MODE
     * ======================================================== */

    RS485_RX_Mode();

    HAL_Delay(500);


    /* ========================================================
     * SERVO TORQUE OFF
     * ======================================================== */

    Servo_Torque(0U);

    HAL_Delay(50);


    /* ========================================================
     * UNLOCK EEPROM
     * ======================================================== */

    {
        uint8_t unlock = 0U;

        Servo_Write(
            SERVO_ID,
            REG_LOCK,
            &unlock,
            1U
        );
    }

    HAL_Delay(50);


    /* ========================================================
     * SET WHEEL MODE
     *
     * Register 33 = 1
     * ======================================================== */

    Servo_SetWheelMode();

    HAL_Delay(50);


    /* ========================================================
     * LOCK EEPROM
     * ======================================================== */

    {
        uint8_t lock = 1U;

        Servo_Write(
            SERVO_ID,
            REG_LOCK,
            &lock,
            1U
        );
    }

    HAL_Delay(50);


    /* ========================================================
     * SET ACCELERATION
     * ======================================================== */

    Servo_SetAcceleration(50U);

    HAL_Delay(50);


    /* ========================================================
     * TORQUE ON
     * ======================================================== */

    Servo_Torque(1U);

    HAL_Delay(100);


    /* ========================================================
     * SAFETY STOP
     * ======================================================== */

    Servo_SetSpeed(0);

    HAL_Delay(200);


    /* ========================================================
     * JOYSTICK CENTER CALIBRATION
     *
     * KEEP JOYSTICK CENTER
     * ======================================================== */

    uint16_t joystick_center =
        Joystick_CalibrateCenter();


    /* ========================================================
     * SAFETY STOP
     * ======================================================== */

    Servo_SetSpeed(0);

    HAL_Delay(200);


    printf("\r\n");
    printf("Joystick Center = %u\r\n",
           joystick_center);

    printf("System READY\r\n");
    printf("----------------------------------------------\r\n");


    /* ========================================================
     * VARIABLES
     * ======================================================== */

    uint16_t adc_value;

    uint32_t adc_sum;

    uint32_t distance;

    uint32_t down_range;

    uint32_t up_range;

    uint32_t common_range;

    uint16_t magnitude;

    int32_t offset;

    int16_t target_speed = 0;

    int16_t current_speed = 0;

    int16_t feedback_speed = 0;

    uint8_t moving = 0U;

    uint16_t raw_servo_value = 0U;

    uint32_t last_debug_time = 0U;


    /* ========================================================
     * MAIN LOOP
     * ======================================================== */

    while (1)
    {
        /* ====================================================
         * READ JOYSTICK
         * ==================================================== */

        adc_sum = 0U;


        for (uint8_t i = 0U;
             i < ADC_FILTER_SAMPLES;
             i++)
        {
            adc_sum += Joystick_Read();

            HAL_Delay(1);
        }


        adc_value =
            (uint16_t)(
                adc_sum /
                ADC_FILTER_SAMPLES
            );


        /* ====================================================
         * OFFSET FROM CENTER
         *
         * Positive = UP
         * Negative = DOWN
         * ==================================================== */

        offset =
            (int32_t)adc_value -
            (int32_t)joystick_center;


        /* ====================================================
         * JOYSTICK RANGE
         * ==================================================== */

        down_range =
            (uint32_t)joystick_center -
            JOYSTICK_DEADZONE;


        up_range =
            4095U -
            (uint32_t)joystick_center -
            JOYSTICK_DEADZONE;


        /* ====================================================
         * COMMON RANGE
         *
         * Equal mapping both directions.
         * ==================================================== */

        if (down_range < up_range)
        {
            common_range = down_range;
        }
        else
        {
            common_range = up_range;
        }


        /* ====================================================
         * UP -> CLOCKWISE
         * ==================================================== */

        if (offset >
            (int32_t)JOYSTICK_DEADZONE)
        {
            distance =
                (uint32_t)offset -
                JOYSTICK_DEADZONE;


            if (distance > common_range)
            {
                distance = common_range;
            }


            if (common_range == 0U)
            {
                target_speed = 0;
            }
            else
            {
                magnitude =
                    (uint16_t)(
                        (distance *
                         MAX_CW_SPEED) /
                        common_range
                    );


                if ((magnitude > 0U) &&
                    (magnitude < MIN_CW_SPEED))
                {
                    magnitude =
                        MIN_CW_SPEED;
                }


                if (magnitude >
                    MAX_CW_SPEED)
                {
                    magnitude =
                        MAX_CW_SPEED;
                }


                /*
                 * UP = positive software speed
                 */

                target_speed =
                    (int16_t)magnitude;
            }
        }


        /* ====================================================
         * DOWN -> ANTICLOCKWISE
         * ==================================================== */

        else if (offset <
                 -(int32_t)JOYSTICK_DEADZONE)
        {
            distance =
                (uint32_t)(-offset) -
                JOYSTICK_DEADZONE;


            if (distance > common_range)
            {
                distance = common_range;
            }


            if (common_range == 0U)
            {
                target_speed = 0;
            }
            else
            {
                magnitude =
                    (uint16_t)(
                        (distance *
                         MAX_CCW_SPEED) /
                        common_range
                    );


                if ((magnitude > 0U) &&
                    (magnitude < MIN_CCW_SPEED))
                {
                    magnitude =
                        MIN_CCW_SPEED;
                }


                if (magnitude >
                    MAX_CCW_SPEED)
                {
                    magnitude =
                        MAX_CCW_SPEED;
                }


                /*
                 * DOWN = negative software speed
                 */

                target_speed =
                    -(int16_t)magnitude;
            }
        }


        /* ====================================================
         * CENTER -> STOP
         * ==================================================== */

        else
        {
            target_speed = 0;
        }


        /* ====================================================
         * DIRECTION CHANGE
         *
         * Stop before reversing.
         * ==================================================== */

        if (((current_speed > 0) &&
             (target_speed < 0)) ||
            ((current_speed < 0) &&
             (target_speed > 0)))
        {
            current_speed = 0;

            Servo_SetSpeed(0);

            HAL_Delay(
                STOP_BEFORE_REVERSE_MS
            );
        }


        /* ====================================================
         * SPEED RAMP
         * ==================================================== */

        if (current_speed < target_speed)
        {
            current_speed += SPEED_STEP;


            if (current_speed > target_speed)
            {
                current_speed =
                    target_speed;
            }
        }

        else if (current_speed > target_speed)
        {
            current_speed -= SPEED_STEP;


            if (current_speed < target_speed)
            {
                current_speed =
                    target_speed;
            }
        }


        /* ====================================================
         * SEND SPEED
         * ==================================================== */

        Servo_SetSpeed(current_speed);


        /* ====================================================
         * RAW COMMAND VALUE
         * ==================================================== */

        raw_servo_value =
            Servo_GetRawSpeedValue(
                current_speed
            );


        /* ====================================================
         * FEEDBACK EVERY 200 ms
         * ==================================================== */

        if ((HAL_GetTick() -
             last_debug_time) >=
            DEBUG_PRINT_INTERVAL)
        {
            last_debug_time =
                HAL_GetTick();


            /* Read present speed */

            feedback_speed =
                Servo_ReadPresentSpeed();


            /* Read moving flag */

            moving =
                Servo_ReadMoving();


            /* =================================================
             * SERIAL MONITOR
             * ================================================= */

            printf(
                "ADC=%4u | Center=%4u | Offset=%5ld | ",
                adc_value,
                joystick_center,
                (long)offset
            );


            printf(
                "Target=%5d | Current=%5d | ",
                target_speed,
                current_speed
            );


            /* Requested direction */

            if (current_speed > 0)
            {
                printf(
                    "REQ=CLOCKWISE | "
                );
            }
            else if (current_speed < 0)
            {
                printf(
                    "REQ=ANTICLOCKWISE | "
                );
            }
            else
            {
                printf(
                    "REQ=STOP | "
                );
            }


            printf(
                "RAW=0x%04X | ",
                raw_servo_value
            );


            /* Feedback */

            if (feedback_speed >= 0)
            {
                printf(
                    "FB_SPEED=%5d | ",
                    feedback_speed
                );
            }
            else
            {
                printf(
                    "FB_SPEED=%5d | ",
                    feedback_speed
                );
            }


            printf(
                "MOVING=%u\r\n",
                moving
            );
        }


        HAL_Delay(20);
    }
}


/* ============================================================
 * JOYSTICK ADC READ
 * ============================================================ */

static uint16_t Joystick_Read(void)
{
    uint16_t value = 2048U;


    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(
                &hadc1,
                10U) == HAL_OK)
        {
            value =
                (uint16_t)
                HAL_ADC_GetValue(&hadc1);
        }


        HAL_ADC_Stop(&hadc1);
    }


    return value;
}


/* ============================================================
 * JOYSTICK AVERAGE
 * ============================================================ */

static uint16_t Joystick_Read_Average(
    uint16_t samples)
{
    uint32_t sum = 0U;


    if (samples == 0U)
    {
        return 2048U;
    }


    for (uint16_t i = 0U;
         i < samples;
         i++)
    {
        sum += Joystick_Read();

        HAL_Delay(2);
    }


    return
        (uint16_t)(
            sum / samples
        );
}


/* ============================================================
 * JOYSTICK CENTER CALIBRATION
 * ============================================================ */

static uint16_t Joystick_CalibrateCenter(void)
{
    /*
     * Keep joystick at CENTER.
     */

    return
        Joystick_Read_Average(
            CENTER_SAMPLES
        );
}


/* ============================================================
 * RS485 TX MODE
 * ============================================================ */

static void RS485_TX_Mode(void)
{
    HAL_GPIO_WritePin(
        RS485_DE_PORT,
        RS485_DE_PIN,
        GPIO_PIN_SET
    );
}


/* ============================================================
 * RS485 RX MODE
 * ============================================================ */

static void RS485_RX_Mode(void)
{
    HAL_GPIO_WritePin(
        RS485_DE_PORT,
        RS485_DE_PIN,
        GPIO_PIN_RESET
    );
}


/* ============================================================
 * FLUSH RS485 RX BUFFER
 *
 * Important because non-broadcast servo commands can produce
 * status responses.
 * ============================================================ */

static void Servo_FlushRx(void)
{
    uint8_t dummy;


    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_RXNE) != RESET)
    {
        HAL_UART_Receive(
            &huart1,
            &dummy,
            1U,
            1U
        );
    }


    __HAL_UART_CLEAR_OREFLAG(&huart1);
}


/* ============================================================
 * SERVO CHECKSUM
 * ============================================================ */

static uint8_t Servo_Checksum(
    uint8_t *packet,
    uint8_t length)
{
    uint16_t sum = 0U;


    for (uint8_t i = 2U;
         i < length;
         i++)
    {
        sum += packet[i];
    }


    return
        (uint8_t)(~sum);
}


/* ============================================================
 * SERVO WRITE
 * ============================================================ */

static HAL_StatusTypeDef Servo_Write(
    uint8_t id,
    uint8_t address,
    uint8_t *data,
    uint8_t data_length)
{
    uint8_t packet[32];


    /*
     * WRITE DATA:
     *
     * Length = data bytes + 3
     *
     * Instruction
     * Address
     * Data
     */

    uint8_t length =
        (uint8_t)(
            data_length + 3U
        );


    packet[0] = 0xFFU;
    packet[1] = 0xFFU;

    packet[2] = id;

    packet[3] = length;

    packet[4] = INST_WRITE;

    packet[5] = address;


    for (uint8_t i = 0U;
         i < data_length;
         i++)
    {
        packet[6U + i] =
            data[i];
    }


    uint8_t packet_length =
        (uint8_t)(
            data_length + 7U
        );


    packet[packet_length - 1U] =
        Servo_Checksum(
            packet,
            (uint8_t)(
                packet_length - 1U
            )
        );


    /* ========================================================
     * CLEAR OLD RX DATA
     * ======================================================== */

    Servo_FlushRx();


    /* ========================================================
     * TX MODE
     * ======================================================== */

    RS485_TX_Mode();

    HAL_Delay(1);


    /* ========================================================
     * SEND
     * ======================================================== */

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            &huart1,
            packet,
            packet_length,
            100U
        );


    /* ========================================================
     * WAIT TX COMPLETE
     * ======================================================== */

    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_TC) == RESET)
    {
    }


    HAL_Delay(1);


    /* ========================================================
     * RX MODE
     * ======================================================== */

    RS485_RX_Mode();


    /*
     * Give servo time to return status packet.
     */

    HAL_Delay(1);


    /*
     * Discard status response.
     */

    Servo_FlushRx();


    return status;
}


/* ============================================================
 * SERVO READ
 *
 * READ DATA:
 *
 * FF FF ID 04 02 ADDRESS LENGTH CHECKSUM
 *
 * ============================================================ */

static HAL_StatusTypeDef Servo_Read(
    uint8_t id,
    uint8_t address,
    uint8_t data_length,
    uint8_t *data)
{
    uint8_t packet[16];

    uint8_t response[32];


    /* ========================================================
     * BUILD READ REQUEST
     * ======================================================== */

    packet[0] = 0xFFU;
    packet[1] = 0xFFU;

    packet[2] = id;

    /*
     * READ command always has:
     *
     * instruction
     * address
     * read length
     *
     * Length field = 4
     */

    packet[3] = 4U;

    packet[4] = INST_READ;

    packet[5] = address;

    packet[6] = data_length;


    packet[7] =
        Servo_Checksum(
            packet,
            7U
        );


    /* ========================================================
     * FLUSH OLD DATA
     * ======================================================== */

    Servo_FlushRx();


    /* ========================================================
     * TX MODE
     * ======================================================== */

    RS485_TX_Mode();

    HAL_Delay(1);


    /* ========================================================
     * SEND REQUEST
     * ======================================================== */

    if (HAL_UART_Transmit(
            &huart1,
            packet,
            8U,
            100U
        ) != HAL_OK)
    {
        RS485_RX_Mode();

        return HAL_ERROR;
    }


    /* ========================================================
     * WAIT TX COMPLETE
     * ======================================================== */

    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_TC) == RESET)
    {
    }


    HAL_Delay(1);


    /* ========================================================
     * RX MODE
     * ======================================================== */

    RS485_RX_Mode();


    /* ========================================================
     * EXPECTED RESPONSE SIZE
     *
     * Total:
     *
     * Header     = 2
     * ID         = 1
     * Length     = 1
     * Remaining  = length + 1
     *
     * For 2-byte read:
     *
     * 2 + 1 + 1 + 5 = 9?
     *
     * Actual protocol response for 2 data bytes:
     *
     * FF FF ID 04 ERROR DATA DATA CHECKSUM
     *
     * Total = 8
     *
     * ======================================================== */

    uint16_t expected_length =
        (uint16_t)(
            data_length + 6U
        );


    if (expected_length >
        sizeof(response))
    {
        return HAL_ERROR;
    }


    /* ========================================================
     * RECEIVE
     * ======================================================== */

    if (HAL_UART_Receive(
            &huart1,
            response,
            expected_length,
            20U
        ) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }


    /* ========================================================
     * HEADER CHECK
     * ======================================================== */

    if ((response[0] != 0xFFU) ||
        (response[1] != 0xFFU))
    {
        return HAL_ERROR;
    }


    /* ========================================================
     * ID CHECK
     * ======================================================== */

    if (response[2] != id)
    {
        return HAL_ERROR;
    }


    /* ========================================================
     * LENGTH CHECK
     *
     * Error + data + checksum
     *
     * length = data_length + 2
     * ======================================================== */

    if (response[3] !=
        (uint8_t)(
            data_length + 2U
        ))
    {
        return HAL_ERROR;
    }


    /* ========================================================
     * ERROR BYTE
     *
     * response[4]
     * ======================================================== */

    if (response[4] != 0U)
    {
        return HAL_ERROR;
    }


    /* ========================================================
     * CHECKSUM
     * ======================================================== */

    if (Servo_Checksum(
            response,
            (uint8_t)(
                expected_length - 1U
            )
        ) !=
        response[
            expected_length - 1U
        ])
    {
        return HAL_ERROR;
    }


    /* ========================================================
     * COPY DATA
     * ======================================================== */

    for (uint8_t i = 0U;
         i < data_length;
         i++)
    {
        data[i] =
            response[5U + i];
    }


    return HAL_OK;
}


/* ============================================================
 * READ PRESENT SPEED
 *
 * Register 58 = low byte
 * Register 59 = high byte
 *
 * Bit 15 = direction
 * ============================================================ */

static int16_t Servo_ReadPresentSpeed(void)
{
    uint8_t data[2];


    if (Servo_Read(
            SERVO_ID,
            REG_PRESENT_SPEED_L,
            2U,
            data
        ) != HAL_OK)
    {
        return 0;
    }


    uint16_t raw =
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8);


    if (raw & 0x8000U)
    {
        return
            -(int16_t)(
                raw & 0x7FFFU
            );
    }


    return
        (int16_t)raw;
}


/* ============================================================
 * READ MOVING
 *
 * Register 66
 * ============================================================ */

static uint8_t Servo_ReadMoving(void)
{
    uint8_t data[1];


    if (Servo_Read(
            SERVO_ID,
            REG_MOVING,
            1U,
            data
        ) != HAL_OK)
    {
        return 0U;
    }


    return data[0];
}


/* ============================================================
 * GET RAW SERVO SPEED VALUE
 *
 * This is the EXACT 16-bit value sent to register 46.
 *
 * ============================================================ */

static uint16_t Servo_GetRawSpeedValue(
    int16_t speed)
{
    int16_t command_speed;

    uint16_t value;


    command_speed = speed;


    /* ========================================================
     * DIRECTION INVERSION
     * ======================================================== */

    if (SERVO_DIRECTION_INVERT)
    {
        command_speed =
            (int16_t)(-command_speed);
    }


    /* ========================================================
     * STOP
     * ======================================================== */

    if (command_speed == 0)
    {
        return 0U;
    }


    /* ========================================================
     * POSITIVE RAW SPEED
     * ======================================================== */

    if (command_speed > 0)
    {
        uint16_t magnitude =
            (uint16_t)command_speed;


        if (magnitude >
            MAX_CW_SPEED)
        {
            magnitude =
                MAX_CW_SPEED;
        }


        value = magnitude;
    }


    /* ========================================================
     * NEGATIVE RAW SPEED
     * ======================================================== */

    else
    {
        uint16_t magnitude =
            (uint16_t)(-command_speed);


        if (magnitude >
            MAX_CCW_SPEED)
        {
            magnitude =
                MAX_CCW_SPEED;
        }


        value =
            magnitude |
            0x8000U;
    }


    return value;
}


/* ============================================================
 * SERVO SET SPEED
 * ============================================================ */

static void Servo_SetSpeed(
    int16_t speed)
{
    uint16_t value;


    uint8_t data[2];


    value =
        Servo_GetRawSpeedValue(
            speed
        );


    /* LOW BYTE */

    data[0] =
        (uint8_t)(
            value &
            0x00FFU
        );


    /* HIGH BYTE */

    data[1] =
        (uint8_t)(
            (value >> 8) &
            0x00FFU
        );


    Servo_Write(
        SERVO_ID,
        REG_GOAL_SPEED_L,
        data,
        2U
    );
}


/* ============================================================
 * SERVO TORQUE
 * ============================================================ */

static void Servo_Torque(uint8_t enable)
{
    uint8_t value =
        (enable != 0U) ? 1U : 0U;


    Servo_Write(
        SERVO_ID,
        REG_TORQUE,
        &value,
        1U
    );
}


/* ============================================================
 * SERVO WHEEL MODE
 * ============================================================ */

static void Servo_SetWheelMode(void)
{
    uint8_t value = 1U;


    Servo_Write(
        SERVO_ID,
        REG_MODE,
        &value,
        1U
    );
}


/* ============================================================
 * SERVO ACCELERATION
 * ============================================================ */

static void Servo_SetAcceleration(
    uint8_t acceleration)
{
    Servo_Write(
        SERVO_ID,
        REG_ACCELERATION,
        &acceleration,
        1U
    );
}


/* ============================================================
 * GPIO INITIALIZATION
 * ============================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
        {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();


    /* ========================================================
     * PA0 = ADC1_IN0
     *
     * HW-504 VRY
     * ======================================================== */

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


    /* ========================================================
     * PA8 = MAX485 DE + /RE
     * ======================================================== */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        GPIO_PIN_8;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* ========================================================
     * PA5 OUTPUT
     * ======================================================== */

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


/* ============================================================
 * USART1
 *
 * PA9  = TX
 * PA10 = RX
 *
 * 1 Mbps
 * ============================================================ */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance =
        USART1;


    huart1.Init.BaudRate =
        1000000U;

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


/* ============================================================
 * USART2
 *
 * PA2  = TX
 * PA3  = RX
 *
 * 115200
 * ============================================================ */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance =
        USART2;


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


    if (HAL_UART_Init(
            &huart2)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * ADC1
 *
 * PA0 = ADC1_IN0
 * ============================================================ */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig =
        {0};


    hadc1.Instance =
        ADC1;


    hadc1.Init.ClockPrescaler =
        ADC_CLOCKPRESCALER_PCLK_DIV4;

    hadc1.Init.Resolution =
        ADC_RESOLUTION12b;

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


    if (HAL_ADC_Init(
            &hadc1)
        != HAL_OK)
    {
        Error_Handler();
    }


    /* ========================================================
     * CHANNEL 0
     * ======================================================== */

    sConfig.Channel =
        ADC_CHANNEL_0;

    sConfig.Rank =
        1U;

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


/* ============================================================
 * SYSTEM CLOCK
 *
 * HSE  = 8 MHz
 * SYSCLK = 84 MHz
 * APB1 = 42 MHz
 * APB2 = 84 MHz
 * ============================================================ */

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


    /* ========================================================
     * HSE + PLL
     * ======================================================== */

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM =
        8U;

    RCC_OscInitStruct.PLL.PLLN =
        336U;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ =
        7U;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct)
        != HAL_OK)
    {
        Error_Handler();
    }


    /* ========================================================
     * CLOCKS
     * ======================================================== */

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
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * ERROR HANDLER
 * ============================================================ */

void Error_Handler(void)
{
    __disable_irq();


    while (1)
    {
    }
}