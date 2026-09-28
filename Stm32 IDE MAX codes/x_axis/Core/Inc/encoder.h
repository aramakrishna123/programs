#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

/* ============================================================
 * ENCODER PINS
 * ============================================================ */

#define ENCODER_A_PORT               GPIOB
#define ENCODER_A_PIN                GPIO_PIN_8

#define ENCODER_B_PORT               GPIOB
#define ENCODER_B_PIN                GPIO_PIN_9


/* ============================================================
 * ENCODER DIRECTION
 *
 * 0 = normal
 * 1 = invert count direction
 * ============================================================ */

#define ENCODER_DIRECTION_INVERT     0U


/* ============================================================
 * ENCODER MECHANICAL CONVERSION
 *
 * 180 squares / inch
 * 4 counts / square
 *
 * = 720 counts / inch
 * ============================================================ */

#define ENCODER_SQUARES_PER_INCH     180L
#define ENCODER_COUNTS_PER_SQUARE    4L

#define ENCODER_COUNTS_PER_INCH \
    (ENCODER_SQUARES_PER_INCH * ENCODER_COUNTS_PER_SQUARE)


/* ============================================================
 * FUNCTIONS
 * ============================================================ */

void Encoder_Init(void);

void Encoder_Reset(void);

int32_t Encoder_GetCount(void);

int8_t Encoder_GetDirection(void);

int32_t Encoder_GetPositionMM(void);

#endif