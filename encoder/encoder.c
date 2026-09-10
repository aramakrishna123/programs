#include "encoder.h"
#include "main.h"

/*
 * HEDS-9730 encoder
 *
 * Encoder A -> PB8
 * Encoder B -> PB9
 *
 * Assumed encoder resolution:
 * 180 lines/inch
 *
 * 4x quadrature decoding:
 * 180 x 4 = 720 counts/inch
 */

#define ENCODER_LINES_PER_INCH     180L
#define QUADRATURE_MULTIPLIER      4L

#define ENCODER_COUNTS_PER_INCH \
        (ENCODER_LINES_PER_INCH * QUADRATURE_MULTIPLIER)

#define MICROMETERS_PER_INCH       25400L


/* Encoder count */
static volatile int32_t encoder_count = 0;

/* Previous A/B state */
static uint8_t previous_state = 0U;


/**
 * @brief Initialize encoder
 */
void Encoder_Init(void)
{
    uint8_t A;
    uint8_t B;

    /* Read initial A state */
    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
            ? 1U : 0U;

    /* Read initial B state */
    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            ? 1U : 0U;

    /* Store initial quadrature state */
    previous_state = (uint8_t)((A << 1U) | B);

    /* Reset count */
    encoder_count = 0;
}


/**
 * @brief Update quadrature encoder
 *
 * Call this function continuously from main().
 */
void Encoder_Update(void)
{
    uint8_t A;
    uint8_t B;
    uint8_t current_state;
    uint8_t transition;

    /* Read Channel A */
    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
            ? 1U : 0U;

    /* Read Channel B */
    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
            ? 1U : 0U;

    /*
     * A = bit 1
     * B = bit 0
     *
     * 00 = 0
     * 01 = 1
     * 10 = 2
     * 11 = 3
     */
    current_state = (uint8_t)((A << 1U) | B);

    /*
     * Combine previous and current states.
     *
     * Example:
     * previous = 00
     * current  = 01
     *
     * transition = 000001
     */
    transition =
        (uint8_t)((previous_state << 2U) | current_state);

    /*
     * 4x quadrature decoding
     *
     * Direction 1:
     * 00 -> 01 -> 11 -> 10 -> 00
     */
    switch (transition)
    {
        case 0x01:   /* 00 -> 01 */
        case 0x07:   /* 01 -> 11 */
        case 0x0E:   /* 11 -> 10 */
        case 0x08:   /* 10 -> 00 */
            encoder_count++;
            break;

        /*
         * Direction 2:
         * 00 -> 10 -> 11 -> 01 -> 00
         */
        case 0x02:   /* 00 -> 10 */
        case 0x0B:   /* 10 -> 11 */
        case 0x0D:   /* 11 -> 01 */
        case 0x04:   /* 01 -> 00 */
            encoder_count--;
            break;

        /*
         * Invalid transition or no movement
         */
        default:
            break;
    }

    /* Save current state */
    previous_state = current_state;
}


/**
 * @brief Get raw encoder count
 */
int32_t Encoder_GetCount(void)
{
    return encoder_count;
}


/**
 * @brief Get encoder position in micrometers
 *
 * 720 counts = 1 inch
 * 1 inch    = 25400 micrometers
 */
int32_t Encoder_GetPositionUM(void)
{
    return (encoder_count * MICROMETERS_PER_INCH)
           / ENCODER_COUNTS_PER_INCH;
}


/**
 * @brief Get encoder position in millimeters
 */
float Encoder_GetPositionMM(void)
{
    return (float)Encoder_GetPositionUM() / 1000.0f;
}


/**
 * @brief Reset encoder count
 */
void Encoder_Reset(void)
{
    encoder_count = 0;
}