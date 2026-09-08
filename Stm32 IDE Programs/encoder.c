#include "encoder.h"
#include "main.h"

#define ENCODER_LINES_PER_INCH    180L
#define QUADRATURE_MULTIPLIER     4L

#define ENCODER_COUNTS_PER_INCH   \
        (ENCODER_LINES_PER_INCH * QUADRATURE_MULTIPLIER)

#define MICROMETERS_PER_INCH      25400L

volatile int32_t encoder_count = 0;

static uint8_t previous_state = 0U;


/* Initialize encoder */
void Encoder_Init(void)
{
    uint8_t A;
    uint8_t B;

    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
        ? 1U : 0U;

    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
        ? 1U : 0U;

    previous_state = (uint8_t)((A << 1U) | B);

    encoder_count = 0;
}


/* Update encoder */
void Encoder_Update(void)
{
    uint8_t A;
    uint8_t B;
    uint8_t current_state;
    uint8_t transition;

    A = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
        ? 1U : 0U;

    B = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
        ? 1U : 0U;

    current_state = (uint8_t)((A << 1U) | B);

    transition = (uint8_t)((previous_state << 2U) | current_state);

    switch (transition)
    {
        case 0x01:
        case 0x07:
        case 0x0E:
        case 0x08:
            encoder_count++;
            break;

        case 0x02:
        case 0x0B:
        case 0x0D:
        case 0x04:
            encoder_count--;
            break;

        default:
            break;
    }

    previous_state = current_state;
}


/* Get raw encoder count */
int32_t Encoder_GetCount(void)
{
    return encoder_count;
}


/* Get position in micrometers */
int32_t Encoder_GetPositionUM(void)
{
    return (encoder_count * MICROMETERS_PER_INCH)
           / ENCODER_COUNTS_PER_INCH;
}


/* Get position in millimeters */
float Encoder_GetPositionMM(void)
{
    return (float)Encoder_GetPositionUM() / 1000.0f;
}


/* Reset encoder count */
void Encoder_Reset(void)
{
    encoder_count = 0;
}