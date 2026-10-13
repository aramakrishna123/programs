/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Z-axis RS485 Remote Joystick Stepper Controller
  *
  * STM32F446RE
  *
  * MOTOR (Leadshine EM870S):
  *   PUL+ = PB0 = TIM1_CH2N,  DIR+ = PB1,  PUL-/DIR- = STM32 GND
  * ENCODER:        A = PB8, B = PB9
  * LIMIT SWITCHES: PC2, PC3 (other terminal GND, LOW = pressed)
  * INA219:         SCL = PB6, SDA = PB7
  * RS485 REMOTE:   DE+RE = PA8, DI = PA9 (USART1_TX), RO = PA10 (USART1_RX)
  * PC SERIAL:      TX = PA2, RX = PA3 (USART2, 115200)
  *
  * Remote data format (ASCII, 115200 8N1):   "<0..4095>\n"   e.g. "3500\n"
  * 2048 = stop. Send repeatedly (every ~50 ms); no data for 250 ms = stop.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "encoder.h"
#include "isensor.h"
#include "remote_joystick.h"

#include <stdio.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_CW   = 1,
    MOTOR_CCW  = -1
} MotorDirection_t;

/* REMOTE JOYSTICK */
#ifndef REMOTE_CENTER
#define REMOTE_CENTER                2048U
#endif
#define REMOTE_MIN_VALUE             0U
#define REMOTE_MAX_VALUE             4095U
#define JOYSTICK_STOP_BAND           100U

/* MOTOR SPEED */
#define MOTOR_MIN_SPEED_HZ           100U
#define MOTOR_MAX_SPEED_HZ           2000U
#define SPEED_RAMP_STEP_HZ           25U
#define SPEED_UPDATE_MS              10U

/* MOTOR TIMING */
#define TIM1_TIMER_TICK_HZ           1000000UL
#define TIM1_PRESCALER               179U
#define DIRECTION_SETUP_US           10U
#define REVERSAL_STOP_MS             2U

/* CURRENT PROTECTION */
#define CURRENT_LIMIT_A              INA219_CURRENT_LIMIT_A
#define CURRENT_RESTART_A            (CURRENT_LIMIT_A * 0.90f)
#define CURRENT_TRIP_SAMPLES         3U
#define CURRENT_CHECK_MS             20U

/* PRINTING (one combined line) */
#define STATUS_PRINT_MS              250U

/* RS485 DE/RE */
#define RS485_DE_PORT                GPIOA
#define RS485_DE_PIN                 GPIO_PIN_8

/* DIR logic: HIGH = CW, LOW = CCW */
#define MOTOR_CW_DIR_LEVEL           GPIO_PIN_SET
#define MOTOR_CCW_DIR_LEVEL          GPIO_PIN_RESET

/* ------------------------------------------------------------------------- */
ADC_HandleTypeDef  hadc1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef  hi2c1;

static MotorDirection_t active_direction    = MOTOR_STOP;
static MotorDirection_t requested_direction = MOTOR_STOP;

static uint32_t current_speed_hz = 0U;
static uint32_t target_speed_hz  = 0U;
static uint8_t  motor_running    = 0U;

static float   current_A     = 0.0f;
static float   bus_voltage_V = 0.0f;
static float   power_W       = 0.0f;
static uint8_t current_over_limit_count = 0U;
static uint8_t current_fault = 0U;

static int32_t     previous_encoder_count = 0;
static const char *encoder_direction_text = "STOP";

static uint32_t last_speed_update  = 0U;
static uint32_t last_current_check = 0U;
static uint32_t last_status_print  = 0U;

volatile uint32_t usart1_irq_count = 0U;

/* ------------------------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

static void Motor_PinToGPIO(void);
static void Motor_PinToTimer(void);
static void Motor_SetDirection(MotorDirection_t direction);
static void Motor_Stop(void);
static void Motor_Start(void);
static void TIM1_SetFrequency(uint32_t frequency_hz);

static void Joystick_Process(uint16_t joystick_value,
                             MotorDirection_t *direction,
                             uint32_t *speed_hz);

static uint8_t Limit_IsPressed(void);
static void CurrentProtection_Update(void);

static const char *MotorDirectionToString(MotorDirection_t direction);
static void Encoder_UpdateDirectionText(void);

/* =========================================================
   DWT MICROSECOND DELAY
   ========================================================= */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Delay_us(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000UL);

    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}

/* =========================================================
   RS485 DIRECTION (receive only)
   ========================================================= */
static void RS485_SetReceive(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
}

/* =========================================================
   MOTOR PIN CONTROL (PB0: GPIO low <-> TIM1_CH2N)
   ========================================================= */
static void Motor_PinToGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

static void Motor_PinToTimer(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin       = GPIO_PIN_0;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* =========================================================
   TIM1 FREQUENCY  (PB0 = TIM1_CH2N)
   ========================================================= */
static void TIM1_SetFrequency(uint32_t frequency_hz)
{
    uint32_t period_counts;
    uint32_t compare_counts;

    if (frequency_hz == 0U)
    {
        TIM1->CR1  &= ~TIM_CR1_CEN;
        TIM1->CCER &= ~TIM_CCER_CC2NE;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        return;
    }

    if (frequency_hz > MOTOR_MAX_SPEED_HZ) { frequency_hz = MOTOR_MAX_SPEED_HZ; }
    if (frequency_hz < MOTOR_MIN_SPEED_HZ) { frequency_hz = MOTOR_MIN_SPEED_HZ; }

    period_counts = TIM1_TIMER_TICK_HZ / frequency_hz;

    if (period_counts < 2U)     { period_counts = 2U; }
    if (period_counts > 65536U) { period_counts = 65536U; }

    compare_counts = period_counts / 2U;

    TIM1->ARR  = period_counts - 1U;
    TIM1->CCR2 = compare_counts;
    TIM1->EGR  = TIM_EGR_UG;
}

/* =========================================================
   MOTOR DIRECTION / STOP / START
   ========================================================= */
static void Motor_SetDirection(MotorDirection_t direction)
{
    if (direction == MOTOR_CW)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, MOTOR_CW_DIR_LEVEL);
    }
    else if (direction == MOTOR_CCW)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, MOTOR_CCW_DIR_LEVEL);
    }
}

static void Motor_Stop(void)
{
    TIM1->CR1  &= ~TIM_CR1_CEN;
    TIM1->CCER &= ~TIM_CCER_CC2NE;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    Motor_PinToGPIO();

    current_speed_hz = 0U;
    motor_running    = 0U;
    active_direction = MOTOR_STOP;
}

static void Motor_Start(void)
{
    if (active_direction == MOTOR_STOP)
    {
        return;
    }

    if (current_speed_hz < MOTOR_MIN_SPEED_HZ)
    {
        return;
    }

    Motor_PinToTimer();

    TIM1_SetFrequency(current_speed_hz);

    TIM1->CNT = 0U;
    TIM1->CCER |= TIM_CCER_CC2NE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1  |= TIM_CR1_CEN;

    motor_running = 1U;
}

/* =========================================================
   JOYSTICK VALUE -> DIRECTION + SPEED
   ========================================================= */
static void Joystick_Process(uint16_t joystick_value,
                             MotorDirection_t *direction,
                             uint32_t *speed_hz)
{
    int32_t  delta;
    uint32_t magnitude;
    uint32_t maximum_magnitude;

    if (joystick_value > REMOTE_MAX_VALUE)
    {
        joystick_value = REMOTE_MAX_VALUE;
    }

    delta = (int32_t)joystick_value - (int32_t)REMOTE_CENTER;

    /* Centre region = STOP */
    if ((delta >= -(int32_t)JOYSTICK_STOP_BAND) &&
        (delta <=  (int32_t)JOYSTICK_STOP_BAND))
    {
        *direction = MOTOR_STOP;
        *speed_hz  = 0U;
        return;
    }

    if (delta > 0)
    {
        *direction = MOTOR_CW;
        magnitude  = (uint32_t)(delta - (int32_t)JOYSTICK_STOP_BAND);
        maximum_magnitude = REMOTE_MAX_VALUE - REMOTE_CENTER - JOYSTICK_STOP_BAND;
    }
    else
    {
        *direction = MOTOR_CCW;
        magnitude  = (uint32_t)((-delta) - (int32_t)JOYSTICK_STOP_BAND);
        maximum_magnitude = REMOTE_CENTER - JOYSTICK_STOP_BAND;
    }

    if (maximum_magnitude == 0U)
    {
        *speed_hz  = 0U;
        *direction = MOTOR_STOP;
        return;
    }

    if (magnitude > maximum_magnitude)
    {
        magnitude = maximum_magnitude;
    }

    *speed_hz = MOTOR_MIN_SPEED_HZ +
                ((magnitude * (MOTOR_MAX_SPEED_HZ - MOTOR_MIN_SPEED_HZ)) /
                 maximum_magnitude);

    if (*speed_hz > MOTOR_MAX_SPEED_HZ)
    {
        *speed_hz = MOTOR_MAX_SPEED_HZ;
    }
}

/* =========================================================
   LIMIT SWITCHES (LOW = pressed)
   ========================================================= */
static uint8_t Limit_IsPressed(void)
{
    if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) ||
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET))
    {
        return 1U;
    }
    return 0U;
}

/* =========================================================
   CURRENT PROTECTION
   ========================================================= */
static void CurrentProtection_Update(void)
{
    uint8_t read_ok = INA219_ReadCurrent(&hi2c1, &current_A);

    /* If the read fails, do not fabricate a current value. */
    if (read_ok == 0U)
    {
        return;
    }

    if ((motor_running != 0U) && (current_A >= CURRENT_LIMIT_A))
    {
        if (current_over_limit_count < CURRENT_TRIP_SAMPLES)
        {
            current_over_limit_count++;
        }

        if (current_over_limit_count >= CURRENT_TRIP_SAMPLES)
        {
            if (current_fault == 0U)
            {
                printf("\r\n*** OVERCURRENT FAULT ***\r\n");
                printf("CURRENT_LIMIT=%.0f mA\r\n", CURRENT_LIMIT_A * 1000.0f);
            }

            current_fault = 1U;
            target_speed_hz = 0U;
            Motor_Stop();
        }
    }
    else
    {
        current_over_limit_count = 0U;

        if ((current_fault != 0U) && (current_A <= CURRENT_RESTART_A))
        {
            current_fault = 0U;
            printf("*** CURRENT FAULT CLEARED ***\r\n");
        }
    }
}

/* =========================================================
   TEXT HELPERS
   ========================================================= */
static const char *MotorDirectionToString(MotorDirection_t direction)
{
    if (direction == MOTOR_CW)  { return "CW"; }
    if (direction == MOTOR_CCW) { return "CCW"; }
    return "STOP";
}

static void Encoder_UpdateDirectionText(void)
{
    int32_t current_count = Encoder_GetCount();

    if (current_count > previous_encoder_count)
    {
        encoder_direction_text = "CW";
    }
    else if (current_count < previous_encoder_count)
    {
        encoder_direction_text = "CCW";
    }
    else
    {
        encoder_direction_text = "STOP";
    }

    previous_encoder_count = current_count;
}

/* =========================================================
   MAIN
   ========================================================= */
int main(void)
{
    uint16_t         joystick_value;
    MotorDirection_t new_direction;
    uint32_t         new_speed;
    uint32_t         now;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();

    DWT_Init();

    /* MAX485 stays in receive mode: remote data comes in on USART1 RX */
    RS485_SetReceive();

    Remote_Init();

    Encoder_Init();

    if (INA219_Init(&hi2c1) != 0U)
    {
        printf("\r\nINA219 INIT: OK\r\n");
    }
    else
    {
        printf("\r\nINA219 INIT: FAILED\r\n");
    }

    Motor_Stop();

    requested_direction = MOTOR_STOP;
    active_direction    = MOTOR_STOP;
    current_speed_hz    = 0U;
    target_speed_hz     = 0U;
    current_fault       = 0U;
    current_over_limit_count = 0U;

    previous_encoder_count = Encoder_GetCount();

    printf(
        "\r\n"
        "========================================\r\n"
        "     Y-AXIS RS485 MOTOR CONTROLLER\r\n"
        "========================================\r\n"
        "REMOTE    = USART1 RS485 (PA9/PA10, DE=PA8)\r\n"
        "PC SERIAL = USART2 115200\r\n"
        "FORMAT    = ASCII \"0..4095\\n\", 2048 = stop\r\n"
        "STOP BAND = +/- %u\r\n"
        "SPEED     = %u ... %u Hz\r\n"
        "CURRENT LIMIT = %.0f mA\r\n"
        "========================================\r\n\r\n",
        JOYSTICK_STOP_BAND,
        MOTOR_MIN_SPEED_HZ,
        MOTOR_MAX_SPEED_HZ,
        CURRENT_LIMIT_A * 1000.0f
    );

    HAL_Delay(100);

    while (1)
    {
        now = HAL_GetTick();

        /* ---- ENCODER ---- */
        Encoder_Update();
        Encoder_UpdateDirectionText();

        /* ---- REMOTE JOYSTICK ---- */
        joystick_value = Remote_ReadOrCenter();

        Joystick_Process(joystick_value, &new_direction, &new_speed);

        requested_direction = new_direction;
        target_speed_hz     = new_speed;

        /* ---- LIMIT SWITCH SAFETY ---- */
        if (Limit_IsPressed() != 0U)
        {
            target_speed_hz = 0U;

            if (motor_running != 0U)
            {
                Motor_Stop();
                printf("\r\n*** LIMIT SWITCH PRESSED - MOTOR STOPPED ***\r\n");
            }
        }

        /* ---- CURRENT FAULT ---- */
        if (current_fault != 0U)
        {
            target_speed_hz = 0U;
        }

        /* ---- DIRECTION REVERSAL: ALWAYS STOP FIRST ---- */
        if ((requested_direction != MOTOR_STOP) &&
            (active_direction != MOTOR_STOP) &&
            (requested_direction != active_direction) &&
            (current_speed_hz > 0U))
        {
            Motor_Stop();
            HAL_Delay(REVERSAL_STOP_MS);
        }

        /* After stopping, select the new direction */
        if ((current_speed_hz == 0U) &&
            (requested_direction != MOTOR_STOP) &&
            (current_fault == 0U) &&
            (Limit_IsPressed() == 0U))
        {
            if (active_direction != requested_direction)
            {
                Motor_SetDirection(requested_direction);
                Delay_us(DIRECTION_SETUP_US);
                active_direction = requested_direction;
            }
        }

        /* ---- SPEED RAMP ---- */
        if ((now - last_speed_update) >= SPEED_UPDATE_MS)
        {
            last_speed_update = now;

            if (current_speed_hz < target_speed_hz)
            {
                current_speed_hz += SPEED_RAMP_STEP_HZ;

                if (current_speed_hz > target_speed_hz)
                {
                    current_speed_hz = target_speed_hz;
                }
                if (current_speed_hz > MOTOR_MAX_SPEED_HZ)
                {
                    current_speed_hz = MOTOR_MAX_SPEED_HZ;
                }
            }
            else if (current_speed_hz > target_speed_hz)
            {
                if (current_speed_hz > SPEED_RAMP_STEP_HZ)
                {
                    current_speed_hz -= SPEED_RAMP_STEP_HZ;
                }
                else
                {
                    current_speed_hz = 0U;
                }

                if (current_speed_hz < target_speed_hz)
                {
                    current_speed_hz = target_speed_hz;
                }
            }

            /* ---- START / UPDATE / STOP MOTOR ---- */
            if (current_speed_hz == 0U)
            {
                if (motor_running != 0U)
                {
                    Motor_Stop();
                }

                if (requested_direction == MOTOR_STOP)
                {
                    active_direction = MOTOR_STOP;
                }
            }
            else
            {
                if ((Limit_IsPressed() != 0U) ||
                    (current_fault != 0U) ||
                    (requested_direction == MOTOR_STOP))
                {
                    current_speed_hz = 0U;
                    Motor_Stop();

                    if (requested_direction == MOTOR_STOP)
                    {
                        active_direction = MOTOR_STOP;
                    }
                }
                else
                {
                    if (motor_running == 0U)
                    {
                        Motor_Start();
                    }
                    else
                    {
                        TIM1_SetFrequency(current_speed_hz);
                    }
                }
            }
        }

        /* ---- CURRENT / VOLTAGE / POWER ---- */
        if ((now - last_current_check) >= CURRENT_CHECK_MS)
        {
            last_current_check = now;

            CurrentProtection_Update();
            INA219_ReadBusVoltage(&hi2c1, &bus_voltage_V);
            INA219_ReadPower(&hi2c1, &power_W);
        }

        /* ---- ONE COMBINED STATUS LINE ---- */
        if ((now - last_status_print) >= STATUS_PRINT_MS)
        {
            last_status_print = now;

            float   pos_mm   = Encoder_GetPositionMM();
            int32_t pos_int  = (int32_t)pos_mm;
            int32_t pos_frac = (int32_t)((pos_mm - (float)pos_int) * 100.0f);
            if (pos_frac < 0) { pos_frac = -pos_frac; }

            printf(
                "REMOTE=%u IRQ=%lu | REQ=%s ACT=%s | TGT=%lu CUR=%lu Hz | "
                "MOTOR=%s LIMIT=%s FAULT=%s | "
                "ENC=%ld %s %ld.%02ldmm | "
                "%lu mA %lu mV %lu mW\r\n",

                joystick_value,
                (unsigned long)usart1_irq_count,
                MotorDirectionToString(requested_direction),
                MotorDirectionToString(active_direction),
                (unsigned long)target_speed_hz,
                (unsigned long)current_speed_hz,
                (motor_running != 0U) ? "RUN" : "STOP",
                (Limit_IsPressed() != 0U) ? "PRESSED" : "OK",
                (current_fault != 0U) ? "CURRENT" : "OK",
                (long)Encoder_GetCount(),
                encoder_direction_text,
                (long)pos_int,
                (long)pos_frac,
                (unsigned long)(current_A * 1000.0f),
                (unsigned long)(bus_voltage_V * 1000.0f),
                (unsigned long)(power_W * 1000.0f)
            );
        }

        HAL_Delay(1);
    }
}

/* =========================================================
   TIM1 INITIALIZATION (PB0 = TIM1_CH2N), direct registers
   TIM1 tick = 180 MHz / 180 = 1 MHz
   ========================================================= */
static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM1->CR1  = 0U;
    TIM1->CR1 |= TIM_CR1_ARPE;

    TIM1->PSC  = TIM1_PRESCALER;
    TIM1->ARR  = 999U;    /* 1000 Hz initial */
    TIM1->CCR2 = 500U;    /* 50% */

    /* Channel 2: PWM mode 1 (OC2M = 110), preload enabled */
    TIM1->CCMR1  = 0U;
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;

    TIM1->CCER = 0U;      /* outputs disabled */
    TIM1->BDTR = 0U;      /* main output disabled */

    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
}

/* =========================================================
   ADC1 (PA0 = ADC1_IN0, unused by remote mode but kept)
   ========================================================= */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1U;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1U;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   USART1 - RS485 REMOTE JOYSTICK (PA9 TX, PA10 RX, AF7)
   ========================================================= */
static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   USART2 - PC SERIAL MONITOR (PA2 TX, PA3 RX)
   ========================================================= */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   I2C1 (PB6 SCL, PB7 SDA)
   ========================================================= */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0U;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   GPIO
   ========================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB0 = PUL (GPIO low until motor starts) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB1 = DIR */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB8 / PB9 = ENCODER */
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PC2 / PC3 = LIMIT SWITCHES (LOW = pressed) */
    GPIO_InitStruct.Pin   = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PA8 = MAX485 DE + /RE (LOW = receive) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* =========================================================
   SYSTEM CLOCK: 180 MHz from HSI (16 MHz), PLLM=8 N=180 P=2
   ========================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8U;
    RCC_OscInitStruct.PLL.PLLN            = 180U;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 2U;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
   USART1 IRQ (RS485 remote joystick)
   Only ONE USART1_IRQHandler may exist in the project:
   delete it from stm32f4xx_it.c if it is there.
   ========================================================= */
void USART1_IRQHandler(void)
{
    usart1_irq_count++;
    Remote_OnUartIRQ();
}

/* =========================================================
   printf -> USART2
   ========================================================= */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* =========================================================
   ERROR HANDLER
   ========================================================= */
void Error_Handler(void)
{
    Motor_Stop();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: %s:%lu\r\n", file, (unsigned long)line);
}
#endif