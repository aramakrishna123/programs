#include "main.h"

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

#define SERVO_ID 1

/* ---------------------------------------------------------
Write data to RSBL servo
--------------------------------------------------------- */
void Servo_Write(uint8_t address, uint8_t *data, uint8_t length)
{
uint8_t packet[20];
uint8_t checksum = 0;

```
packet[0] = 0xFF;
packet[1] = 0xFF;
packet[2] = SERVO_ID;

/* Length = instruction + address + data + checksum */
packet[3] = length + 3;

packet[4] = 0x03;       // WRITE
packet[5] = address;

for (uint8_t i = 0; i < length; i++)
{
    packet[6 + i] = data[i];
}

/* Checksum */
for (uint8_t i = 2; i < 6 + length; i++)
{
    checksum += packet[i];
}

packet[6 + length] = ~checksum;

/* Complete packet */
HAL_UART_Transmit(
    &huart1,
    packet,
    7 + length,
    100
);
```

}

/* ---------------------------------------------------------
Set operation mode
0x21 = Operation Mode
1 = Constant-speed motor
--------------------------------------------------------- */
void Servo_SetMode(uint8_t mode)
{
uint8_t data = mode;

```
Servo_Write(0x21, data, 1);
```

}

/* ---------------------------------------------------------
Torque
0x28
1 = ON
--------------------------------------------------------- */
void Servo_Torque(uint8_t enable)
{
uint8_t data = enable;

```
Servo_Write(0x28, &data, 1);
```

}

/* ---------------------------------------------------------
Acceleration
0x29
0 = maximum acceleration
--------------------------------------------------------- */
void Servo_SetAcceleration(uint8_t acceleration)
{
uint8_t data = acceleration;

```
Servo_Write(0x29, &data, 1);
```

}

/* ---------------------------------------------------------
Motor speed
0x2E
16-bit signed value

Positive = one direction
Negative = opposite direction

Default unit:
1 count = 0.732 RPM

50 = approximately 36.6 RPM
--------------------------------------------------------- */
void Servo_SetSpeed(int16_t speed)
{
uint8_t data[2];

```
data[0] = speed & 0xFF;
data[1] = (speed >> 8) & 0xFF;

Servo_Write(0x2E, data, 2);
```

}

/* ---------------------------------------------------------
MAIN
--------------------------------------------------------- */
int main(void)
{
HAL_Init();

```
SystemClock_Config();

MX_GPIO_Init();

MX_USART1_UART_Init();

HAL_Delay(1000);

/* Constant-speed mode */
Servo_SetMode(1);

HAL_Delay(200);

/* Torque ON */
Servo_Torque(1);

HAL_Delay(200);

/* Maximum acceleration */
Servo_SetAcceleration(0);

HAL_Delay(200);

/* Start motor */
Servo_SetSpeed(50);

while (1)
{
    HAL_Delay(1000);
}
```

}

/* ---------------------------------------------------------
USART1

PA9  = TX
PA10 = RX
115200 baud, 8-N-1
--------------------------------------------------------- */
static void MX_USART1_UART_Init(void)
{
huart1.Instance = USART1;

```
huart1.Init.BaudRate = 115200;
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits = UART_STOPBITS_1;
huart1.Init.Parity = UART_PARITY_NONE;
huart1.Init.Mode = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;

if (HAL_UART_Init(&huart1) != HAL_OK)
{
    Error_Handler();
}
```

}

/* ---------------------------------------------------------
GPIO
--------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
__HAL_RCC_GPIOA_CLK_ENABLE();
}

/* ---------------------------------------------------------
SYSTEM CLOCK
--------------------------------------------------------- */
void SystemClock_Config(void)
{
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

```
__HAL_RCC_PWR_CLK_ENABLE();

__HAL_PWR_VOLTAGESCALING_CONFIG(
    PWR_REGULATOR_VOLTAGE_SCALE3
);

RCC_OscInitStruct.OscillatorType =
    RCC_OSCILLATORTYPE_HSI;

RCC_OscInitStruct.HSIState = RCC_HSI_ON;

RCC_OscInitStruct.HSICalibrationValue =
    RCC_HSICALIBRATION_DEFAULT;

RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

RCC_OscInitStruct.PLL.PLLSource =
    RCC_PLLSOURCE_HSI;

RCC_OscInitStruct.PLL.PLLM = 16;
RCC_OscInitStruct.PLL.PLLN = 192;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
RCC_OscInitStruct.PLL.PLLQ = 2;
RCC_OscInitStruct.PLL.PLLR = 2;

if (HAL_RCC_OscConfig(&RCC_OscInitStruct)
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
        FLASH_LATENCY_3) != HAL_OK)
{
    Error_Handler();
}
```

}

/* ---------------------------------------------------------
ERROR HANDLER
--------------------------------------------------------- */
void Error_Handler(void)
{
__disable_irq();

```
while (1)
{
}
```

}
