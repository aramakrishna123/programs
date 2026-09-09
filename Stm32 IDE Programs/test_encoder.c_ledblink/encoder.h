#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Encoder initialization */
void Encoder_Init(void);

/* Call continuously from main loop */
void Encoder_Update(void);

/* Get current encoder count */
int32_t Encoder_GetCount(void);

/* Get position in micrometers */
int32_t Encoder_GetPositionUM(void);

/* Get position in millimeters */
float Encoder_GetPositionMM(void);

/* Reset encoder count */
void Encoder_Reset(void);

#endif