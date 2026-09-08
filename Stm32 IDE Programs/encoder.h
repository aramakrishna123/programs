#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
void Encoder_Update(void);

int32_t Encoder_GetCount(void);
int32_t Encoder_GetPositionUM(void);
float Encoder_GetPositionMM(void);

void Encoder_Reset(void);

#endif