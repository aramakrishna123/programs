#include "main.h"
#include "encoder.h"

#include <stdio.h>
#include <stdint.h>

/* =========================================================
   UART HANDLES
   ========================================================= */

UART_HandleTypeDef huart1;   /* USART1 -> Servo RS422 */
UART_HandleTypeDef huart2;   /* USART2 -> not used for PC now */
UART_HandleTypeDef huart3;   /* USART3 -> DTech */


/* =========================================================
   PIN DEFINITIONS
   ========================================================= */

/* ---------------------------------------------------------
   SERVO RS422 / MAX485
   --------------------------------------------------------- */

#define SERVO_RS422_DE_PORT     GPIOA
#define SERVO_RS422_DE_PIN      GPIO_PIN_8

/*
 * PA8  = Servo DE/RE
 * PA9  = USART1_TX
 * PA10 = USART1_RX
 */


/* ---------------------------------------------------------
   DTECH RS422 / MAX485
   --------------------------------------------------------- */

#define DTECH_RS422_DE_PORT     GPIOA
#define DTECH_RS422_DE_PIN      GPIO_PIN_1

/*
 * PA1  = DTech DE/RE
 * PC10 = USART3_TX -> DTech DI
 * PC11 = USART3_RX <- DTech RO
 */


/* ---------------------------------------------------------
   EXISTING OUTPUT
   --------------------------------------------------------- */

#define EXISTING_OUT_PORT       GPIOA
#define EXISTING_OUT_PIN        GPIO_PIN_5


/* ---------------------------------------------------------
   LIMIT SWITCHES
   --------------------------------------------------------- */

#define LIMIT_CCW_PORT          GPIOC
#define LIMIT_CCW_PIN           GPIO_PIN_2

#define LIMIT_CW_PORT           GPIOC
#define LIMIT_CW_PIN            GPIO_PIN_3


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

#define SERVO_WHEEL_MODE            1U


/* =========================================================
   MOTOR SETTINGS
   ========================================================= */

#define DEFAULT_SPEED               1000
#define MIN_SPEED                   300
#define MAX_SPEED                   1000

#define SPEED_STEP                  50
#define SPEED_UPDATE_MS             10U

#define STOP_BEFORE_REVERSE_MS      100U

#define LOOP_DELAY_MS               5U
#define PRINT_INTERVAL_MS           200U

#define CURRENT_MA_PER_RAW          6.5f


/* =========================================================
   DTECH RX BUFFER
   ========================================================= */

#define DTECH_RX_BUFFER_SIZE        128U

static volatile uint8_t dtech_rx_byte = 0U;

static volatile uint8_t
dtech_rx_buffer[DTECH_RX_BUFFER_SIZE];

static volatile uint16_t
dtech_rx_head = 0U;

static volatile uint16_t
dtech_rx_tail = 0U;

static volatile uint8_t
dtech_rx_overflow = 0U;


/* =========================================================
   MOTOR DIRECTION
   ========================================================= */

typedef enum
{
    DIR_STOP = 0,
    DIR_CLOCKWISE = 1,
    DIR_ANTICLOCKWISE = -1
} ServoDirection_t;


/* =========================================================
   MOTOR STATE
   ========================================================= */

static ServoDirection_t requested_direction =
    DIR_STOP;

static ServoDirection_t active_direction =
    DIR_STOP;

static int32_t target_speed = 0;
static int32_t active_speed = 0;


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

/* System */
void SystemClock_Config(void);


/* Peripheral initialization */
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);


/* Servo */
static void Servo_RS422_SetReceive(void);
static void Servo_RS422_SetTransmit(void);

static uint8_t Servo_WriteReg8(
    uint8_t reg,
    uint8_t value
);

static uint8_t Servo_WriteReg16(
    uint8_t reg,
    uint16_t value
);

static uint8_t Servo_ReadReg16(
    uint8_t reg,
    uint16_t *value
);

static uint16_t Servo_GetRawSpeedValue(
    int16_t speed
);

static uint8_t Servo_SetSpeed(
    int16_t speed
);

static uint8_t Servo_Init(void);


/* DTech */
static void DTech_RS422_SetReceive(void);
static void DTech_RS422_SetTransmit(void);

static void DTech_StartReceive(void);
static uint8_t DTech_GetByte(uint8_t *byte);

static void DTech_CommandTask(void);
static void DTech_ProcessByte(uint8_t byte);

/*
 * Called from stm32f4xx_it.c
 */
void DTech_ReceiveByte(uint8_t byte);


/* Limits */
static uint8_t Limit_CCW_Pressed(void);
static uint8_t Limit_CW_Pressed(void);


/* Motor */
static void Motor_ControlTask(void);


/* Display */
static void Serial_PrintStatus(void);
static void Serial_PrintHelp(void);


/* =========================================================
   STDOUT -> USART3 DTECH
   ========================================================= */

/*
 * All printf() output:
 *
 * STM32
 *   ↓
 * USART3
 *   ↓
 * DTech
 *   ↓
 * COM8
 *
 * USART2 / COM3 is NOT used here.
 */

int _write(int file, char *ptr, int len)
{
    (void)file;

    if (len <= 0)
    {
        return 0;
    }

    /*
     * DTech transmit mode.
     */
    DTech_RS422_SetTransmit();

    /*
     * Send to DTech.
     */
    HAL_UART_Transmit(
        &huart3,
        (uint8_t *)ptr,
        (uint16_t)len,
        100U
    );

    /*
     * Return to receive mode.
     */
    DTech_RS422_SetReceive();

    return len;
}


/* =========================================================
   SERVO RS422 DIRECTION
   ========================================================= */

static void Servo_RS422_SetReceive(void)
{
    HAL_GPIO_WritePin(
        SERVO_RS422_DE_PORT,
        SERVO_RS422_DE_PIN,
        GPIO_PIN_RESET
    );
}


static void Servo_RS422_SetTransmit(void)
{
    HAL_GPIO_WritePin(
        SERVO_RS422_DE_PORT,
        SERVO_RS422_DE_PIN,
        GPIO_PIN_SET
    );
}


/* =========================================================
   DTECH RS422 DIRECTION
   ========================================================= */

static void DTech_RS422_SetReceive(void)
{
    HAL_GPIO_WritePin(
        DTECH_RS422_DE_PORT,
        DTECH_RS422_DE_PIN,
        GPIO_PIN_RESET
    );
}


static void DTech_RS422_SetTransmit(void)
{
    HAL_GPIO_WritePin(
        DTECH_RS422_DE_PORT,
        DTECH_RS422_DE_PIN,
        GPIO_PIN_SET
    );
}


/* =========================================================
   DTECH START RECEIVE
   ========================================================= */

static void DTech_StartReceive(void)
{
    /*
     * Keep DTech transceiver in receive mode.
     */
    DTech_RS422_SetReceive();

    /*
     * USART3 receiver.
     */
    USART3->CR1 |= USART_CR1_RE;

    /*
     * RXNE interrupt.
     */
    USART3->CR1 |= USART_CR1_RXNEIE;

    /*
     * USART3 enable.
     */
    USART3->CR1 |= USART_CR1_UE;

    /*
     * Enable NVIC.
     */
    HAL_NVIC_SetPriority(
        USART3_IRQn,
        5,
        0
    );

    HAL_NVIC_EnableIRQ(
        USART3_IRQn
    );
}


/* =========================================================
   DTECH RECEIVE BYTE
   ========================================================= */

/*
 * Called from USART3_IRQHandler().
 *
 * This function must be fast.
 */
void DTech_ReceiveByte(uint8_t byte)
{
    uint16_t next_head;

    next_head =
        (uint16_t)(dtech_rx_head + 1U);


    if (next_head >= DTECH_RX_BUFFER_SIZE)
    {
        next_head = 0U;
    }


    /*
     * Buffer full.
     */
    if (next_head == dtech_rx_tail)
    {
        dtech_rx_overflow = 1U;

        return;
    }


    dtech_rx_buffer[dtech_rx_head] =
        byte;


    dtech_rx_head =
        next_head;
}


/* =========================================================
   DTECH GET BYTE
   ========================================================= */

static uint8_t DTech_GetByte(uint8_t *byte)
{
    uint16_t tail;

    if (byte == NULL)
    {
        return 0U;
    }


    tail = dtech_rx_tail;


    if (tail == dtech_rx_head)
    {
        return 0U;
    }


    *byte =
        dtech_rx_buffer[tail];


    tail++;

    if (tail >= DTECH_RX_BUFFER_SIZE)
    {
        tail = 0U;
    }


    dtech_rx_tail =
        tail;


    return 1U;
}


/* =========================================================
   DTECH COMMAND TASK
   ========================================================= */

static void DTech_CommandTask(void)
{
    uint8_t ch;


    /*
     * Process all received bytes.
     */
    while (DTech_GetByte(&ch))
    {
        DTech_ProcessByte(ch);
    }


    if (dtech_rx_overflow)
    {
        dtech_rx_overflow = 0U;

        printf(
            "DTECH RX BUFFER OVERFLOW\r\n"
        );
    }
}


/* =========================================================
   DTECH COMMAND PROCESSING
   ========================================================= */

/*
 * Commands are executed immediately.
 *
 * F / f     = CW
 * B / b     = CCW
 * S / s     = STOP
 * H / h     = HELP
 * ?         = STATUS
 *
 * Also supported:
 *
 * CW
 * CCW
 * STOP
 * HELP
 * STATUS
 */

static void DTech_ProcessByte(uint8_t byte)
{
    switch (byte)
    {
        /* -------------------------------------------------
           CLOCKWISE
           ------------------------------------------------- */

        case 'F':
        case 'f':

            requested_direction =
                DIR_CLOCKWISE;

            target_speed =
                DEFAULT_SPEED;

            printf(
                "COMMAND: CLOCKWISE\r\n"
            );

            break;


        /* -------------------------------------------------
           ANTICLOCKWISE
           ------------------------------------------------- */

        case 'B':
        case 'b':

            requested_direction =
                DIR_ANTICLOCKWISE;

            target_speed =
                DEFAULT_SPEED;

            printf(
                "COMMAND: ANTICLOCKWISE\r\n"
            );

            break;


        /* -------------------------------------------------
           STOP
           ------------------------------------------------- */

        case 'S':
        case 's':

            requested_direction =
                DIR_STOP;

            target_speed =
                0;

            printf(
                "COMMAND: STOP\r\n"
            );

            break;


        /* -------------------------------------------------
           HELP
           ------------------------------------------------- */

        case 'H':
        case 'h':

            Serial_PrintHelp();

            break;


        /* -------------------------------------------------
           STATUS
           ------------------------------------------------- */

        case '?':

            Serial_PrintStatus();

            break;


        /* -------------------------------------------------
           CR / LF
           ------------------------------------------------- */

        case '\r':
        case '\n':

            break;


        /* -------------------------------------------------
           OTHER CHARACTERS
           ------------------------------------------------- */

        case 'C':
        case 'c':

            /*
             * C is intentionally not a command by itself.
             * CW / CCW words are supported below only when
             * a command parser is added.
             *
             * For current F/B/S mode, ignore C.
             */
            break;


        default:

            /*
             * Numeric or other DTech data can arrive here.
             * It is not used for motor control.
             */
            break;
    }
}


/* =========================================================
   SERVO WRITE 8-BIT
   ========================================================= */

static uint8_t Servo_WriteReg8(
    uint8_t reg,
    uint8_t value)
{
    uint8_t packet[8];
    uint8_t sum;
    uint8_t checksum;


    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = SERVO_ID;
    packet[3] = 4U;
    packet[4] = SERVO_INST_WRITE;
    packet[5] = reg;
    packet[6] = value;


    sum =
        packet[2] +
        packet[3] +
        packet[4] +
        packet[5] +
        packet[6];


    checksum =
        (uint8_t)(~sum);


    packet[7] =
        checksum;


    Servo_RS422_SetTransmit();


    if (HAL_UART_Transmit(
            &huart1,
            packet,
            sizeof(packet),
            100U) != HAL_OK)
    {
        Servo_RS422_SetReceive();

        return 0U;
    }


    Servo_RS422_SetReceive();


    return 1U;
}


/* =========================================================
   SERVO WRITE 16-BIT
   ========================================================= */

static uint8_t Servo_WriteReg16(
    uint8_t reg,
    uint16_t value)
{
    uint8_t packet[9];
    uint8_t sum;
    uint8_t checksum;


    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = SERVO_ID;
    packet[3] = 5U;
    packet[4] = SERVO_INST_WRITE;
    packet[5] = reg;


    /*
     * LOW BYTE
     */
    packet[6] =
        (uint8_t)(value & 0xFFU);


    /*
     * HIGH BYTE
     */
    packet[7] =
        (uint8_t)((value >> 8) & 0xFFU);


    sum =
        packet[2] +
        packet[3] +
        packet[4] +
        packet[5] +
        packet[6] +
        packet[7];


    checksum =
        (uint8_t)(~sum);


    packet[8] =
        checksum;


    Servo_RS422_SetTransmit();


    if (HAL_UART_Transmit(
            &huart1,
            packet,
            sizeof(packet),
            100U) != HAL_OK)
    {
        Servo_RS422_SetReceive();

        return 0U;
    }


    Servo_RS422_SetReceive();


    return 1U;
}


/* =========================================================
   SERVO READ 16-BIT
   ========================================================= */

static uint8_t Servo_ReadReg16(
    uint8_t reg,
    uint16_t *value)
{
    uint8_t tx_packet[8];
    uint8_t rx_packet[8];

    uint8_t sum;
    uint8_t checksum;


    if (value == NULL)
    {
        return 0U;
    }


    /*
     * Clear old bytes.
     */
    while (__HAL_UART_GET_FLAG(
        &huart1,
        UART_FLAG_RXNE))
    {
        volatile uint8_t dummy;

        dummy =
            (uint8_t)(huart1.Instance->DR);

        (void)dummy;
    }


    /*
     * READ REQUEST
     *
     * FF FF ID 04 02 REG 02 CHECKSUM
     */
    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = SERVO_ID;
    tx_packet[3] = 4U;
    tx_packet[4] = SERVO_INST_READ;
    tx_packet[5] = reg;
    tx_packet[6] = 2U;


    sum =
        tx_packet[2] +
        tx_packet[3] +
        tx_packet[4] +
        tx_packet[5] +
        tx_packet[6];


    checksum =
        (uint8_t)(~sum);


    tx_packet[7] =
        checksum;


    Servo_RS422_SetTransmit();


    if (HAL_UART_Transmit(
            &huart1,
            tx_packet,
            sizeof(tx_packet),
            100U) != HAL_OK)
    {
        Servo_RS422_SetReceive();

        return 0U;
    }


    Servo_RS422_SetReceive();


    /*
     * RESPONSE:
     *
     * FF FF ID LEN ERROR LOW HIGH CHECKSUM
     */
    if (HAL_UART_Receive(
            &huart1,
            rx_packet,
            sizeof(rx_packet),
            50U) != HAL_OK)
    {
        return 0U;
    }


    /*
     * Header.
     */
    if ((rx_packet[0] != 0xFFU) ||
        (rx_packet[1] != 0xFFU))
    {
        return 0U;
    }


    /*
     * ID.
     */
    if (rx_packet[2] != SERVO_ID)
    {
        return 0U;
    }


    /*
     * Checksum.
     */
    sum =
        rx_packet[2] +
        rx_packet[3] +
        rx_packet[4] +
        rx_packet[5] +
        rx_packet[6];


    checksum =
        (uint8_t)(~sum);


    if (checksum != rx_packet[7])
    {
        return 0U;
    }


    /*
     * Little endian.
     */
    *value =
        (uint16_t)rx_packet[5] |
        ((uint16_t)rx_packet[6] << 8);


    return 1U;
}


/* =========================================================
   SPEED RAW VALUE
   ========================================================= */

static uint16_t Servo_GetRawSpeedValue(
    int16_t speed)
{
    uint16_t magnitude;


    if (speed == 0)
    {
        return 0U;
    }


    /*
     * Positive = CW.
     */
    if (speed > 0)
    {
        return (uint16_t)speed;
    }


    /*
     * Negative = CCW.
     *
     * Bit 15 = reverse.
     */
    magnitude =
        (uint16_t)(-speed);


    return (uint16_t)(
        magnitude | 0x8000U
    );
}


/* =========================================================
   SET SERVO SPEED
   ========================================================= */

static uint8_t Servo_SetSpeed(
    int16_t speed)
{
    uint16_t raw_value;


    if (speed > MAX_SPEED)
    {
        speed = MAX_SPEED;
    }


    if (speed < -MAX_SPEED)
    {
        speed = -MAX_SPEED;
    }


    raw_value =
        Servo_GetRawSpeedValue(speed);


    return Servo_WriteReg16(
        SERVO_GOAL_SPEED_L_REG,
        raw_value
    );
}


/* =========================================================
   SERVO INITIALIZATION
   ========================================================= */

static uint8_t Servo_Init(void)
{
    printf(
        "\r\n"
        "Initializing RSBL35-24-HS servo...\r\n"
    );


    Servo_RS422_SetReceive();


    /*
     * Torque OFF.
     */
    if (!Servo_WriteReg8(
            SERVO_TORQUE_REG,
            0U))
    {
        printf(
            "WARNING: Torque OFF write failed\r\n"
        );
    }


    /*
     * Unlock.
     */
    Servo_WriteReg8(
        SERVO_LOCK_REG,
        0U
    );


    /*
     * Wheel mode.
     */
    Servo_WriteReg8(
        SERVO_MODE_REG,
        SERVO_WHEEL_MODE
    );


    /*
     * Lock.
     */
    Servo_WriteReg8(
        SERVO_LOCK_REG,
        1U
    );


    /*
     * Acceleration.
     */
    Servo_WriteReg8(
        SERVO_ACCELERATION_REG,
        50U
    );


    /*
     * Torque ON.
     */
    Servo_WriteReg8(
        SERVO_TORQUE_REG,
        1U
    );


    /*
     * Safety stop.
     */
    Servo_SetSpeed(0);


    requested_direction =
        DIR_STOP;

    active_direction =
        DIR_STOP;

    target_speed = 0;
    active_speed = 0;


    printf(
        "Servo ID = %u\r\n",
        SERVO_ID
    );


    printf(
        "Servo initialization complete.\r\n"
    );


    return 1U;
}


/* =========================================================
   LIMIT SWITCHES
   ========================================================= */

static uint8_t Limit_CCW_Pressed(void)
{
    return
        (HAL_GPIO_ReadPin(
            LIMIT_CCW_PORT,
            LIMIT_CCW_PIN
        ) == GPIO_PIN_RESET)
        ? 1U
        : 0U;
}


static uint8_t Limit_CW_Pressed(void)
{
    return
        (HAL_GPIO_ReadPin(
            LIMIT_CW_PORT,
            LIMIT_CW_PIN
        ) == GPIO_PIN_RESET)
        ? 1U
        : 0U;
}


/* =========================================================
   MOTOR CONTROL
   ========================================================= */

static void Motor_ControlTask(void)
{
    static uint32_t last_update = 0U;

    int32_t step;


    /*
     * Speed update every 10 ms.
     */
    if ((HAL_GetTick() -
         last_update) <
        SPEED_UPDATE_MS)
    {
        return;
    }


    last_update =
        HAL_GetTick();


    /* -----------------------------------------------------
       CCW LIMIT
       ----------------------------------------------------- */

    if ((requested_direction ==
         DIR_ANTICLOCKWISE) &&
        Limit_CCW_Pressed())
    {
        requested_direction =
            DIR_STOP;

        target_speed = 0;


        printf(
            "CCW LIMIT PRESSED -> STOP\r\n"
        );
    }


    /* -----------------------------------------------------
       CW LIMIT
       ----------------------------------------------------- */

    if ((requested_direction ==
         DIR_CLOCKWISE) &&
        Limit_CW_Pressed())
    {
        requested_direction =
            DIR_STOP;

        target_speed = 0;


        printf(
            "CW LIMIT PRESSED -> STOP\r\n"
        );
    }


    /* -----------------------------------------------------
       STOP
       ----------------------------------------------------- */

    if (requested_direction ==
        DIR_STOP)
    {
        if (active_speed != 0)
        {
            Servo_SetSpeed(0);

            active_speed = 0;
        }


        active_direction =
            DIR_STOP;

        return;
    }


    /* -----------------------------------------------------
       REVERSAL
       ----------------------------------------------------- */

    if ((active_direction != DIR_STOP) &&
        (active_direction !=
         requested_direction))
    {
        /*
         * Always stop before reversing.
         */
        Servo_SetSpeed(0);

        active_speed = 0;

        active_direction =
            DIR_STOP;


        HAL_Delay(
            STOP_BEFORE_REVERSE_MS
        );
    }


    /* -----------------------------------------------------
       START NEW DIRECTION
       ----------------------------------------------------- */

    if (active_direction ==
        DIR_STOP)
    {
        active_direction =
            requested_direction;

        active_speed = 0;
    }


    /* -----------------------------------------------------
       SPEED RAMP
       ----------------------------------------------------- */

    if (active_speed < target_speed)
    {
        step = SPEED_STEP;

        active_speed += step;


        if (active_speed > target_speed)
        {
            active_speed =
                target_speed;
        }
    }
    else if (active_speed > target_speed)
    {
        step = SPEED_STEP;

        active_speed -= step;


        if (active_speed < target_speed)
        {
            active_speed =
                target_speed;
        }
    }


    /* -----------------------------------------------------
       APPLY SPEED
       ----------------------------------------------------- */

    if (active_direction ==
        DIR_CLOCKWISE)
    {
        Servo_SetSpeed(
            (int16_t)active_speed
        );
    }
    else if (active_direction ==
             DIR_ANTICLOCKWISE)
    {
        Servo_SetSpeed(
            (int16_t)(-active_speed)
        );
    }
}


/* =========================================================
   STATUS
   ========================================================= */

static void Serial_PrintStatus(void)
{
    uint16_t current_raw = 0U;

    uint32_t current_mA = 0U;

    int32_t encoder_count = 0;
    int32_t encoder_position_mm = 0;

    uint8_t ccw_limit;
    uint8_t cw_limit;


    /* -----------------------------------------------------
       LIMITS
       ----------------------------------------------------- */

    ccw_limit =
        Limit_CCW_Pressed();

    cw_limit =
        Limit_CW_Pressed();


    /* -----------------------------------------------------
       ENCODER
       ----------------------------------------------------- */

    Encoder_Update();

    encoder_count =
        Encoder_GetCount();

    encoder_position_mm =
        Encoder_GetPositionMM();


    /* -----------------------------------------------------
       SERVO CURRENT
       ----------------------------------------------------- */

    if (Servo_ReadReg16(
            SERVO_PRESENT_CURRENT_REG,
            &current_raw))
    {
        current_mA =
            (uint32_t)(
                ((float)current_raw) *
                CURRENT_MA_PER_RAW
            );
    }


    /* -----------------------------------------------------
       STATUS
       ----------------------------------------------------- */

    printf(
        "\r\n"
        "===============================================\r\n"
    );


    printf(
        "X-AXIS STATUS\r\n"
    );


    printf(
        "SERVO ID : %u\r\n",
        SERVO_ID
    );


    printf(
        "REQUESTED : "
    );


    if (requested_direction ==
        DIR_CLOCKWISE)
    {
        printf(
            "CLOCKWISE"
        );
    }
    else if (requested_direction ==
             DIR_ANTICLOCKWISE)
    {
        printf(
            "ANTICLOCKWISE"
        );
    }
    else
    {
        printf(
            "STOP"
        );
    }


    printf(
        "\r\n"
    );


    printf(
        "ACTIVE    : "
    );


    if (active_direction ==
        DIR_CLOCKWISE)
    {
        printf(
            "CLOCKWISE"
        );
    }
    else if (active_direction ==
             DIR_ANTICLOCKWISE)
    {
        printf(
            "ANTICLOCKWISE"
        );
    }
    else
    {
        printf(
            "STOP"
        );
    }


    printf(
        "\r\n"
    );


    printf(
        "TARGET SPEED : %ld\r\n",
        target_speed
    );


    printf(
        "ACTIVE SPEED : %ld\r\n",
        active_speed
    );


    printf(
        "ENC COUNT    : %ld\r\n",
        encoder_count
    );


    printf(
        "ENC POSITION : %ld mm\r\n",
        encoder_position_mm
    );


    printf(
        "CCW LIMIT    : %s\r\n",
        ccw_limit ?
        "PRESSED" :
        "RELEASED"
    );


    printf(
        "CW LIMIT     : %s\r\n",
        cw_limit ?
        "PRESSED" :
        "RELEASED"
    );


    printf(
        "SERVO CURRENT RAW : %u\r\n",
        current_raw
    );


    printf(
        "SERVO CURRENT     : %lu mA\r\n",
        (unsigned long)current_mA
    );


    printf(
        "===============================================\r\n"
    );
}


/* =========================================================
   HELP
   ========================================================= */

static void Serial_PrintHelp(void)
{
    printf(
        "\r\n"
        "===============================================\r\n"
        " X-AXIS DTECH RS422 CONTROLLER\r\n"
        " STM32F446RE\r\n"
        "===============================================\r\n"
        "\r\n"
        "PC <-> DTECH <-> USART3\r\n"
        "PC COM8 = DTECH SERIAL PORT\r\n"
        "\r\n"
        "DTECH:\r\n"
        "PA1  = DE/RE\r\n"
        "PC10 = USART3_TX / DI\r\n"
        "PC11 = USART3_RX / RO\r\n"
        "115200 8-N-1\r\n"
        "\r\n"
        "SERVO:\r\n"
        "USART1\r\n"
        "PA8  = DE/RE\r\n"
        "PA9  = TX\r\n"
        "PA10 = RX\r\n"
        "1000000 8-N-1\r\n"
        "SERVO ID = 1\r\n"
        "\r\n"
        "LIMITS:\r\n"
        "PC2 = CCW LIMIT\r\n"
        "PC3 = CW LIMIT\r\n"
        "\r\n"
        "COMMANDS:\r\n"
        "F = CLOCKWISE\r\n"
        "B = ANTICLOCKWISE\r\n"
        "S = STOP\r\n"
        "H = HELP\r\n"
        "? = STATUS\r\n"
        "\r\n"
        "DEFAULT SPEED = 1000\r\n"
        "MIN SPEED     = 300\r\n"
        "MAX SPEED     = 1000\r\n"
        "\r\n"
        "===============================================\r\n"
    );
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    uint32_t last_status_print = 0U;


    /* -----------------------------------------------------
       HAL
       ----------------------------------------------------- */

    HAL_Init();


    /* -----------------------------------------------------
       CLOCK
       ----------------------------------------------------- */

    SystemClock_Config();


    /* -----------------------------------------------------
       GPIO
       ----------------------------------------------------- */

    MX_GPIO_Init();


    /* -----------------------------------------------------
       USART1 = SERVO
       ----------------------------------------------------- */

    MX_USART1_UART_Init();


    /* -----------------------------------------------------
       USART2 = LOCAL USB UART
       -----------------------------------------------------
       Not used for PC communication in this version.
       COM3 is only used for USB power/programming.
       ----------------------------------------------------- */

    MX_USART2_UART_Init();


    /* -----------------------------------------------------
       USART3 = DTECH
       ----------------------------------------------------- */

    MX_USART3_UART_Init();


    /* -----------------------------------------------------
       RS422 DEFAULT STATES
       ----------------------------------------------------- */

    Servo_RS422_SetReceive();

    DTech_RS422_SetReceive();


    /* -----------------------------------------------------
       START DTECH RX
       ----------------------------------------------------- */

    DTech_StartReceive();


    /* -----------------------------------------------------
       ENCODER
       ----------------------------------------------------- */

    Encoder_Init();


    /* -----------------------------------------------------
       SERVO
       ----------------------------------------------------- */

    Servo_Init();


    /* -----------------------------------------------------
       MOTOR INITIAL STATE
       ----------------------------------------------------- */

    requested_direction =
        DIR_STOP;

    active_direction =
        DIR_STOP;

    target_speed = 0;
    active_speed = 0;


    /* -----------------------------------------------------
       STARTUP MESSAGE
       ----------------------------------------------------- */

    printf(
        "\r\n\r\n"
        "===============================================\r\n"
        " X-AXIS DTECH RS422 CONTROLLER\r\n"
        " STM32F446RE\r\n"
        "===============================================\r\n"
        "USART1 = SERVO RS422\r\n"
        "USART2 = NOT USED FOR PC DATA\r\n"
        "USART3 = DTECH / COM8\r\n"
        "SERVO ID = 1\r\n"
        "===============================================\r\n"
    );


    Serial_PrintHelp();


    /* -----------------------------------------------------
       MAIN LOOP
       ----------------------------------------------------- */

    while (1)
    {
        /*
         * Process commands received from:
         *
         * COM8
         *   ↓
         * DTech
         *   ↓
         * USART3
         */
        DTech_CommandTask();


        /*
         * Execute motor command.
         */
        Motor_ControlTask();


        /*
         * Periodic status.
         *
         * STM32
         *   ↓
         * USART3
         *   ↓
         * DTech
         *   ↓
         * COM8
         */
        if ((HAL_GetTick() -
             last_status_print) >=
            PRINT_INTERVAL_MS)
        {
            last_status_print =
                HAL_GetTick();

            Serial_PrintStatus();
        }


        HAL_Delay(
            LOOP_DELAY_MS
        );
    }
}


/* =========================================================
   USART1 INITIALIZATION
   SERVO RS422
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
            &huart1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   USART2 INITIALIZATION
   LOCAL USB UART
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
            &huart2) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   USART3 INITIALIZATION
   DTECH RS422
   ========================================================= */

static void MX_USART3_UART_Init(void)
{
    huart3.Instance =
        USART3;

    huart3.Init.BaudRate =
        115200;

    huart3.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart3.Init.StopBits =
        UART_STOPBITS_1;

    huart3.Init.Parity =
        UART_PARITY_NONE;

    huart3.Init.Mode =
        UART_MODE_TX_RX;

    huart3.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart3.Init.OverSampling =
        UART_OVERSAMPLING_16;


    if (HAL_UART_Init(
            &huart3) != HAL_OK)
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


    /* -----------------------------------------------------
       GPIO CLOCKS
       ----------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();


    /* -----------------------------------------------------
       SERVO DE/RE = PA8
       ----------------------------------------------------- */

    HAL_GPIO_WritePin(
        SERVO_RS422_DE_PORT,
        SERVO_RS422_DE_PIN,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        SERVO_RS422_DE_PIN;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;


    HAL_GPIO_Init(
        SERVO_RS422_DE_PORT,
        &GPIO_InitStruct
    );


    /* -----------------------------------------------------
       DTECH DE/RE = PA1
       ----------------------------------------------------- */

    HAL_GPIO_WritePin(
        DTECH_RS422_DE_PORT,
        DTECH_RS422_DE_PIN,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        DTECH_RS422_DE_PIN;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;


    HAL_GPIO_Init(
        DTECH_RS422_DE_PORT,
        &GPIO_InitStruct
    );


    /* -----------------------------------------------------
       EXISTING OUTPUT = PA5
       ----------------------------------------------------- */

    HAL_GPIO_WritePin(
        EXISTING_OUT_PORT,
        EXISTING_OUT_PIN,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        EXISTING_OUT_PIN;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        EXISTING_OUT_PORT,
        &GPIO_InitStruct
    );


    /* -----------------------------------------------------
       LIMIT SWITCHES
       PC2 / PC3
       ----------------------------------------------------- */

    GPIO_InitStruct.Pin =
        LIMIT_CCW_PIN |
        LIMIT_CW_PIN;

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
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* -----------------------------------------------------
       POWER
       ----------------------------------------------------- */

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
        16;

    RCC_OscInitStruct.PLL.PLLN =
        360;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ =
        7;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    /* -----------------------------------------------------
       OVERDRIVE
       ----------------------------------------------------- */

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }


    /* -----------------------------------------------------
       CLOCK
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