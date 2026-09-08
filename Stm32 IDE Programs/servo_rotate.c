// re de short pa8 (mx gpio output)...di pa9....ro pa10 (usart1 tx rx)


#include "main.h"

UART_HandleTypeDef huart1;

/* ============================================================
 * RS485
 * ============================================================ */
#define RS485_DE_PORT          GPIOA
#define RS485_DE_PIN           GPIO_PIN_8

/* ============================================================
 * RSBL35-24-HS
 * ============================================================ */
#define SERVO_ID               1U

#define REG_MODE               33U
#define REG_TORQUE             40U
#define REG_ACCELERATION       41U
#define REG_GOAL_SPEED_L       46U
#define REG_LOCK               55U

#define INST_WRITE             0x03U

/*
 * Start with 1000.
 * Increase later only after confirming rotation.
 */
#define TEST_SPEED             1500U

/* ============================================================
 * Function prototypes
 * ============================================================ */
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

static void RS485_TX_Mode(void);
static void RS485_RX_Mode(void);

static uint8_t Servo_Checksum(uint8_t *packet, uint8_t length);

static HAL_StatusTypeDef Servo_Write(uint8_t id,
                                     uint8_t address,
                                     uint8_t *data,
                                     uint8_t data_length);

static void Servo_Torque(uint8_t enable);
static void Servo_WheelMode(void);
static void Servo_SetAcceleration(uint8_t acceleration);
static void Servo_SetSpeed(uint16_t speed);

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_USART1_UART_Init();

    /* MAX485 receive initially */
    RS485_RX_Mode();

    HAL_Delay(500);

    /* ========================================================
     * SERVO SETUP
     * ======================================================== */

    /* Torque OFF */
    Servo_Torque(0U);
    HAL_Delay(50);

    /* Unlock EEPROM */
    {
        uint8_t value = 0U;

        Servo_Write(SERVO_ID,
                    REG_LOCK,
                    &value,
                    1U);
    }

    HAL_Delay(50);

    /* Motor / wheel mode */
    {
        uint8_t value = 1U;

        Servo_Write(SERVO_ID,
                    REG_MODE,
                    &value,
                    1U);
    }

    HAL_Delay(50);

    /* Lock EEPROM */
    {
        uint8_t value = 1U;

        Servo_Write(SERVO_ID,
                    REG_LOCK,
                    &value,
                    1U);
    }

    HAL_Delay(50);

    /* Acceleration */
    Servo_SetAcceleration(50U);

    HAL_Delay(50);

    /* Torque ON */
    Servo_Torque(1U);

    HAL_Delay(100);

    /*
     * Continuous clockwise command.
     *
     * Positive speed only.
     */
    Servo_SetSpeed(TEST_SPEED);

    /*
     * Keep motor running continuously.
     */
    while (1)
    {
        HAL_Delay(1000);
    }
}

/* ============================================================
 * RS485 TX
 * ============================================================ */
static void RS485_TX_Mode(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT,
                      RS485_DE_PIN,
                      GPIO_PIN_SET);
}

/* ============================================================
 * RS485 RX
 * ============================================================ */
static void RS485_RX_Mode(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT,
                      RS485_DE_PIN,
                      GPIO_PIN_RESET);
}

/* ============================================================
 * CHECKSUM
 * ============================================================ */
static uint8_t Servo_Checksum(uint8_t *packet,
                              uint8_t length)
{
    uint16_t sum = 0U;

    for (uint8_t i = 2U; i < length; i++)
    {
        sum += packet[i];
    }

    return (uint8_t)(~sum);
}

/* ============================================================
 * SERVO WRITE
 *
 * FF FF ID LENGTH INST ADDRESS DATA CHECKSUM
 *
 * LENGTH includes:
 * instruction + address + data + checksum
 * ============================================================ */
static HAL_StatusTypeDef Servo_Write(uint8_t id,
                                     uint8_t address,
                                     uint8_t *data,
                                     uint8_t data_length)
{
    uint8_t packet[16];

    /*
     * Correct length field.
     *
     * 1 data byte -> 04
     * 2 data bytes -> 05
     */
    uint8_t length =
        (uint8_t)(data_length + 3U);

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
        packet[6U + i] = data[i];
    }

    /*
     * Total packet:
     *
     * 2 header
     * 1 ID
     * 1 length
     * 1 instruction
     * 1 address
     * N data
     * 1 checksum
     */
    uint8_t packet_length =
        (uint8_t)(data_length + 7U);

    packet[packet_length - 1U] =
        Servo_Checksum(
            packet,
            (uint8_t)(packet_length - 1U));

    /* TX mode */
    RS485_TX_Mode();

    HAL_Delay(1);

    /* Send packet */
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            &huart1,
            packet,
            packet_length,
            100);

    /*
     * Wait for last byte to leave USART.
     */
    while (__HAL_UART_GET_FLAG(
               &huart1,
               UART_FLAG_TC) == RESET)
    {
    }

    HAL_Delay(1);

    /* RX mode */
    RS485_RX_Mode();

    return status;
}

/* ============================================================
 * TORQUE
 * ============================================================ */
static void Servo_Torque(uint8_t enable)
{
    uint8_t value =
        (enable != 0U) ? 1U : 0U;

    Servo_Write(SERVO_ID,
                REG_TORQUE,
                &value,
                1U);
}

/* ============================================================
 * WHEEL / MOTOR MODE
 * ============================================================ */
static void Servo_WheelMode(void)
{
    uint8_t value = 1U;

    Servo_Write(SERVO_ID,
                REG_MODE,
                &value,
                1U);
}

/* ============================================================
 * ACCELERATION
 * ============================================================ */
static void Servo_SetAcceleration(uint8_t acceleration)
{
    Servo_Write(SERVO_ID,
                REG_ACCELERATION,
                &acceleration,
                1U);
}

/* ============================================================
 * SPEED
 *
 * Positive value only.
 * No sign bit is added.
 * ============================================================ */
static void Servo_SetSpeed(uint16_t speed)
{
    uint8_t data[2];

    data[0] =
        (uint8_t)(speed & 0x00FFU);

    data[1] =
        (uint8_t)((speed >> 8) & 0x00FFU);

    Servo_Write(SERVO_ID,
                REG_GOAL_SPEED_L,
                data,
                2U);
}

/* ============================================================
 * GPIO
 * ============================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*
     * PA8 = MAX485 DE + /RE
     */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        GPIO_PIN_RESET);

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
        &GPIO_InitStruct);

    /*
     * PA5 - keep existing project output
     */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_5,
        GPIO_PIN_RESET);

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
        &GPIO_InitStruct);
}

/* ============================================================
 * USART1
 *
 * PA9  = TX
 * PA10 = RX
 *
 * 1 Mbps, 8-N-1
 * ============================================================ */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;

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

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================
 * SYSTEM CLOCK
 *
 * STM32F446RE
 *
 * HSE 8 MHz
 * SYSCLK 84 MHz
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM = 8U;

    RCC_OscInitStruct.PLL.PLLN = 336U;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ = 7U;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
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
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK)
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

