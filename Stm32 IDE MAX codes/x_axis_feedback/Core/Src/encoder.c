#include "encoder.h"


/* ============================================================
 * INTERNAL VARIABLES
 * ============================================================ */

static volatile int32_t encoder_count = 0;

static volatile int8_t encoder_direction = 0;

static volatile uint8_t encoder_previous_state = 0U;


/* ============================================================
 * QUADRATURE TRANSITION TABLE
 *
 * Positive sequence:
 *
 * 00 -> 01 -> 11 -> 10 -> 00
 *
 * Negative sequence:
 *
 * 00 -> 10 -> 11 -> 01 -> 00
 * ============================================================ */

static const int8_t encoder_transition_table[16] =
{
     0,  1, -1,  0,
    -1,  0,  1,  0,
     1,  0,  0, -1,
     0, -1,  1,  0
};


/* ============================================================
 * READ ENCODER A/B STATE
 * ============================================================ */

static uint8_t Encoder_ReadState(void)
{
    uint8_t state = 0U;


    /* Encoder A = PB8 */

    if (HAL_GPIO_ReadPin(
            ENCODER_A_PORT,
            ENCODER_A_PIN) == GPIO_PIN_SET)
    {
        state |= 0x02U;
    }


    /* Encoder B = PB9 */

    if (HAL_GPIO_ReadPin(
            ENCODER_B_PORT,
            ENCODER_B_PIN) == GPIO_PIN_SET)
    {
        state |= 0x01U;
    }


    return state;
}


/* ============================================================
 * ENCODER INITIALIZATION
 *
 * PB8 = Encoder A
 * PB9 = Encoder B
 *
 * Both rising and falling edges are used.
 * ============================================================ */

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* --------------------------------------------------------
     * GPIOB CLOCK
     * -------------------------------------------------------- */

    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* --------------------------------------------------------
     * PB8 = ENCODER A
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        ENCODER_A_PIN;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_RISING_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
        ENCODER_A_PORT,
        &GPIO_InitStruct
    );


    /* --------------------------------------------------------
     * PB9 = ENCODER B
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        ENCODER_B_PIN;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_RISING_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
        ENCODER_B_PORT,
        &GPIO_InitStruct
    );


    /* --------------------------------------------------------
     * INITIAL STATE
     * -------------------------------------------------------- */

    encoder_previous_state =
        Encoder_ReadState();

    encoder_count = 0;

    encoder_direction = 0;


    /* --------------------------------------------------------
     * PB8/PB9 -> EXTI9_5
     * -------------------------------------------------------- */

    HAL_NVIC_SetPriority(
        EXTI9_5_IRQn,
        5,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI9_5_IRQn
    );
}


/* ============================================================
 * RESET ENCODER
 * ============================================================ */

void Encoder_Reset(void)
{
    __disable_irq();


    encoder_count = 0;

    encoder_direction = 0;

    encoder_previous_state =
        Encoder_ReadState();


    __enable_irq();
}


/* ============================================================
 * GET ENCODER COUNT
 * ============================================================ */

int32_t Encoder_GetCount(void)
{
    int32_t count;


    __disable_irq();

    count = encoder_count;

    __enable_irq();


    return count;
}


/* ============================================================
 * GET ENCODER DIRECTION
 *
 * +1 = positive
 * -1 = negative
 *  0 = no movement yet
 * ============================================================ */

int8_t Encoder_GetDirection(void)
{
    int8_t direction;


    __disable_irq();

    direction = encoder_direction;

    __enable_irq();


    return direction;
}


/* ============================================================
 * GET POSITION IN MILLIMETERS
 *
 * 720 counts = 1 inch
 * 1 inch = 25.4 mm
 * ============================================================ */

int32_t Encoder_GetPositionMM(void)
{
    int32_t count;

    int64_t position_um;


    __disable_irq();

    count = encoder_count;

    __enable_irq();


    position_um =
        ((int64_t)count * 25400LL)
        / ENCODER_COUNTS_PER_INCH;


    return (int32_t)(
        position_um / 1000LL
    );
}


/* ============================================================
 * ENCODER INTERRUPT CALLBACK
 * ============================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t current_state;

    uint8_t transition;

    int8_t delta;


    /* --------------------------------------------------------
     * Only process PB8 / PB9
     * -------------------------------------------------------- */

    if ((GPIO_Pin != ENCODER_A_PIN) &&
        (GPIO_Pin != ENCODER_B_PIN))
    {
        return;
    }


    /* --------------------------------------------------------
     * Read current A/B
     * -------------------------------------------------------- */

    current_state =
        Encoder_ReadState();


    /* --------------------------------------------------------
     * Build transition index
     * -------------------------------------------------------- */

    transition =
        (uint8_t)(
            (encoder_previous_state << 2U) |
            current_state
        );


    /* --------------------------------------------------------
     * Get transition delta
     * -------------------------------------------------------- */

    delta =
        encoder_transition_table[
            transition
        ];


    /* --------------------------------------------------------
     * Update encoder count
     * -------------------------------------------------------- */

    if (delta != 0)
    {
        if (ENCODER_DIRECTION_INVERT != 0U)
        {
            delta = -delta;
        }


        encoder_count += delta;


        if (delta > 0)
        {
            encoder_direction = 1;
        }
        else
        {
            encoder_direction = -1;
        }
    }


    /* --------------------------------------------------------
     * Save current state
     * -------------------------------------------------------- */

    encoder_previous_state =
        current_state;
}

/* ============================================================
 * ENCODER UPDATE
 *
 * Encoder counting is handled by the PB8/PB9 EXTI interrupt
 * in HAL_GPIO_EXTI_Callback().
 *
 * This function is provided because main.c calls Encoder_Update().
 * ============================================================ */

void Encoder_Update(void)
{
    /* No polling required.
       Encoder is updated by EXTI interrupt. */
}
