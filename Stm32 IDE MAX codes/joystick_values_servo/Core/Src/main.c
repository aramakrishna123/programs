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
 * PA9  -> DI
 * PA10 -> RO
 *
 *
 * SERIAL MONITOR:
 *
 * USART2
 * PA2 -> TX
 * PA3 -> RX
 * 115200 baud
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
 * ============================================================
 */


/* ============================================================
 * HANDLES
 * ============================================================ */

ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;     // MAX485 -> Servo
UART_HandleTypeDef huart2;     // Serial Monitor


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

#define INST_WRITE             0x03U


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

static uint8_t Servo_Checksum(uint8_t *packet,
                              uint8_t length);

static HAL_StatusTypeDef Servo_Write(
    uint8_t id,
    uint8_t address,
    uint8_t *data,
    uint8_t data_length
);

static void Servo_Torque(uint8_t enable);
static void Servo_SetWheelMode(void);
static void Servo_SetAcceleration(uint8_t acceleration);
static void Servo_SetSpeed(int16_t speed);


/* ============================================================
 * PRINTF -> USART2
 * ============================================================ */

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)ptr,
        len,
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
     * MAX485 RECEIVE MODE
     * ======================================================== */

    RS485_RX_Mode();

    HAL_Delay(500);


    /* ========================================================
     * SERIAL MONITOR START MESSAGE
     * ======================================================== */

    printf("\r\n");
    printf("============================================\r\n");
    printf(" STM32F446RE JOYSTICK + SERVO\r\n");
    printf("============================================\r\n");
    printf("Joystick VRY -> PA0\r\n");
    printf("UP     -> CLOCKWISE\r\n");
    printf("CENTER -> STOP\r\n");
    printf("DOWN   -> ANTICLOCKWISE\r\n");
    printf("Serial Monitor -> USART2 / 115200 baud\r\n");
    printf("============================================\r\n");
    printf("\r\n");


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
     * IMPORTANT:
     * Keep joystick at CENTER while board starts.
     * ======================================================== */

    printf("Keep joystick at CENTER...\r\n");

    uint16_t joystick_center =
        Joystick_CalibrateCenter();

    printf("Joystick CENTER = %u\r\n",
           joystick_center);

    printf("\r\n");


    /* ========================================================
     * SAFETY STOP
     * ======================================================== */

    Servo_SetSpeed(0);

    HAL_Delay(200);


    /* ========================================================
     * VARIABLES
     * ======================================================== */

    uint16_t adc_value;

    uint32_t adc_sum;

    uint32_t distance;

    uint32_t left_range;

    uint32_t right_range;

    uint32_t common_range;

    uint16_t magnitude;

    int32_t offset;

    int16_t target_speed = 0;

    int16_t current_speed = 0;


    /* ========================================================
     * MAIN LOOP
     * ======================================================== */

while (1)
{
    uint16_t adc_value = Joystick_Read_Average(ADC_FILTER_SAMPLES);

    int16_t target_speed = 0;

    /*
     * JOYSTICK DIRECTION
     *
     * ADC < 1650       -> ANTICLOCKWISE
     * ADC 1650-2350    -> DEAD ZONE / STOP
     * ADC > 2350       -> CLOCKWISE
     */

    if (adc_value < 1650)
    {
        /*
         * ANTICLOCKWISE
         *
         * 1650 -> minimum speed
         * 0    -> maximum speed
         */

        uint16_t speed;

        speed = ((1650 - adc_value) *
                 (MAX_CCW_SPEED - MIN_CCW_SPEED)) / 1650;

        speed += MIN_CCW_SPEED;

        if (speed > MAX_CCW_SPEED)
            speed = MAX_CCW_SPEED;

        target_speed = -(int16_t)speed;
    }
    else if (adc_value > 2350)
    {
        /*
         * CLOCKWISE
         *
         * 2350 -> minimum speed
         * 4095 -> maximum speed
         */

        uint16_t speed;

        speed = ((adc_value - 2350) *
                 (MAX_CW_SPEED - MIN_CW_SPEED)) /
                (4095 - 2350);

        speed += MIN_CW_SPEED;

        if (speed > MAX_CW_SPEED)
            speed = MAX_CW_SPEED;

        target_speed = speed;
    }
    else
    {
        /*
         * DEAD ZONE
         */
        target_speed = 0;
    }

    /*
     * Stop before changing direction
     */
    if ((current_speed > 0 && target_speed < 0) ||
        (current_speed < 0 && target_speed > 0))
    {
        current_speed = 0;

        Servo_SetSpeed(0);

        HAL_Delay(100);
    }

    /*
     * Gradually change speed
     */
    if (current_speed < target_speed)
    {
        current_speed += SPEED_STEP;

        if (current_speed > target_speed)
            current_speed = target_speed;
    }
    else if (current_speed > target_speed)
    {
        current_speed -= SPEED_STEP;

        if (current_speed < target_speed)
            current_speed = target_speed;
    }

    Servo_SetSpeed(current_speed);

    /*
     * Serial Monitor
     */
    if (current_speed < 0)
    {
        printf("Joystick ADC = %u | ANTICW | Speed = %d\r\n",
               adc_value, -current_speed);
    }
    else if (current_speed > 0)
    {
        printf("Joystick ADC = %u | CW | Speed = %d\r\n",
               adc_value, current_speed);
    }
    else
    {
        printf("Joystick ADC = %u | STOP | Speed = 0\r\n",
               adc_value);
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
     * Keep joystick at CENTER during startup.
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
    uint8_t packet[16];


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
     * TX MODE
     * ======================================================== */

    RS485_TX_Mode();

    HAL_Delay(1);


    /* ========================================================
     * TRANSMIT
     * ======================================================== */

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            &huart1,
            packet,
            packet_length,
            100U
        );


    /* ========================================================
     * WAIT FOR TRANSMISSION COMPLETE
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


    return status;
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
 * SERVO SPEED
 *
 * Positive = CLOCKWISE
 * Negative = ANTICLOCKWISE
 *
 * Bit 15 = direction
 * ============================================================ */

static void Servo_SetSpeed(int16_t speed)
{
    uint16_t value;


    if (speed >= 0)
    {
        /*
         * Positive = CLOCKWISE
         */

        value =
            (uint16_t)speed;
    }

    else
    {
        /*
         * Negative = ANTICLOCKWISE
         */

        uint16_t magnitude =
            (uint16_t)(-speed);


        value =
            magnitude |
            0x8000U;
    }


    uint8_t data[2];


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
 * GPIO INITIALIZATION
 * ============================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
        {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();


    /* ========================================================
     * PA0 = ADC1_IN0
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
 * USART1 INITIALIZATION
 *
 * PA9  = TX
 * PA10 = RX
 *
 * 1,000,000 baud
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


    if (HAL_UART_Init(&huart1)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * USART2 INITIALIZATION
 *
 * PA2 = TX
 * PA3 = RX
 *
 * 115200 baud
 *
 * Used only for Serial Monitor.
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


    if (HAL_UART_Init(&huart2)
        != HAL_OK)
    {
        Error_Handler();
    }
}


/* ============================================================
 * ADC1 INITIALIZATION
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


    if (HAL_ADC_Init(&hadc1)
        != HAL_OK)
    {
        Error_Handler();
    }


    /* ========================================================
     * ADC CHANNEL 0
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
     * CLOCK CONFIGURATION
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