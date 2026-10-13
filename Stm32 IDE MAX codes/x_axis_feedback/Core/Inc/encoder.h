
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

/* =========================================================
   ENCODER PIN CONFIGURATION
   ========================================================= */

#define ENCODER_A_PORT              GPIOB
#define ENCODER_A_PIN               GPIO_PIN_8

#define ENCODER_B_PORT              GPIOB
#define ENCODER_B_PIN               GPIO_PIN_9

/* =========================================================
   ENCODER SETTINGS
   ========================================================= */

/*
 * Keep the same value used by your existing encoder
 * position calculation.
 */
#define ENCODER_COUNTS_PER_INCH     1000.0f

/*
 * 0 = normal direction
 * 1 = invert encoder direction
 */
#define ENCODER_DIRECTION_INVERT    0U

/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void Encoder_Init(void);

void Encoder_Update(void);

void Encoder_Reset(void);

int32_t Encoder_GetCount(void);

int8_t Encoder_GetDirection(void);

int32_t Encoder_GetPositionMM(void);

#endif /* ENCODER_H */
