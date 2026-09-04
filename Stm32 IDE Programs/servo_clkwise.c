//baud rate 1000000....re de short pa8..di pa9...ro pa10
#include "main.h"
#include <stdint.h>

/* ==============================
   UART
   ============================== */
UART_HandleTypeDef huart1;

/* ==============================
   RSBL35-24-HS
   ============================== */
#define SERVO_ID       1

/* PA8 -> MAX485 DE + /RE */
#define RS485_DIR_GPIO GPIOA
#define RS485_DIR_PIN  GPIO_PIN_8

#define RS485_TX_MODE() HAL_GPIO_WritePin(RS485_DIR_GPIO, \
                                           RS485_DIR_PIN, \
                                           GPIO_PIN_SET)

#define RS485_RX_MODE() HAL_GPIO_WritePin(RS485_DIR_GPIO, \
                                           RS485_DIR_PIN, \
                                           GPIO_PIN_RESET)

/* RSBL registers */
#define REG_MODE        0x21
#define REG_TORQUE      0x28
#define REG_SPEED       0x2E

/* RSBL instructions */
#define INST_WRITE      0x03


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);


/* =========================================================
   Calculate RSBL checksum

   Checksum = bitwise NOT of:
   ID + Length + Instruction + Address + Parameters
   ========================================================= */
static uint8_t RSBL_Checksum(uint8_t *packet, uint8_t length)
{
    uint16_t sum = 0;

    for (uint8_t i = 2; i < length; i++)
    {
        sum += packet[i];
    }

    return (uint8_t)(~sum);
}


/* =========================================================
   Send RSBL packet through MAX485
   ========================================================= */
static void RSBL_SendPacket(uint8_t *packet, uint8_t length)
{
    RS485_TX_MODE();

    HAL_Delay(1);

    HAL_UART_Transmit(&huart1, packet, length, 100);

    /* Wait until final byte is transmitted */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
    }

    HAL_Delay(1);

    RS485_RX_MODE();
}


/* =========================================================
   Write one byte to an RSBL register
   ========================================================= */
static void RSBL_WriteByte(uint8_t id,
                           uint8_t address,
                           uint8_t value)
{
    uint8_t packet[8];

    packet[0] = 0xFF;
    packet[1] = 0xFF;

    packet[2] = id;

    /* Length = instruction + address + 1 parameter + checksum */
    packet[3] = 0x04;

    packet[4] = INST_WRITE;

    packet[5] = address;

    packet[6] = value;

    packet[7] = RSBL_Checksum(packet, 7);

    RSBL_SendPacket(packet, 8);
}


/* =========================================================
   Write two bytes to an RSBL register
   Low byte first
   ========================================================= */
static void RSBL_WriteWord(uint8_t id,
                           uint8_t address,
                           uint16_t value)
{
    uint8_t packet[9];

    packet[0] = 0xFF;
    packet[1] = 0xFF;

    packet[2] = id;

    /* instruction + address + 2 parameters + checksum */
    packet[3] = 0x05;

    packet[4] = INST_WRITE;

    packet[5] = address;

    packet[6] = (uint8_t)(value & 0xFF);
    packet[7] = (uint8_t)((value >> 8) & 0xFF);

    packet[8] = RSBL_Checksum(packet, 8);

    RSBL_SendPacket(packet, 9);
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

    /* MAX485 receive initially */
    RS485_RX_MODE();

    HAL_Delay(1000);


    /*
       STEP 1:
       Put servo into MOTOR CONSTANT-SPEED MODE

       Register 0x21
       Value 1 = motor constant-speed mode
    */
    RSBL_WriteByte(SERVO_ID, REG_MODE, 1);

    HAL_Delay(100);


    /*
       STEP 2:
       Turn torque ON

       Register 0x28
       Value 1 = torque ON
    */
    RSBL_WriteByte(SERVO_ID, REG_TORQUE, 1);

    HAL_Delay(100);


    /*
       STEP 3:
       Clockwise rotation

       Register 0x2E = running speed

       Default speed unit:
       0.732 RPM

       100 × 0.732 = 73.2 RPM

       Bit 15 = direction.
       Here bit 15 = 0.
    */
    RSBL_WriteWord(SERVO_ID, REG_SPEED, 1000);


    /*
       Keep rotating clockwise.
    */
    while (1)
    {
        HAL_Delay(100);
    }
}


/* =========================================================
   GPIO
   ========================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
       PA8 LOW = MAX485 receive
    */
    HAL_GPIO_WritePin(GPIOA,
                      GPIO_PIN_8,
                      GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_8;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/* =========================================================
   USART1
   ========================================================= */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;

    huart1.Init.BaudRate = 1000000;

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


/* =========================================================
   USART1 MSP INITIALIZATION

   PA9  = USART1_TX
   PA10 = USART1_RX
   ========================================================= */

/* =========================================================
   SYSTEM CLOCK
   ========================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

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
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
