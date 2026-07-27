#ifndef _ENCODER_H
#define _ENCODER_H
#include <stdint.h>
#include "ti_msp_dl_config.h"

void Encoder_OnAEdge(uint32_t gpio_status_a);
void Encoder_OnBEdge(uint32_t gpio_status_b);
void Encoder_UpdateSample(void);
int Encoder_GetDeltaA(void);
int Encoder_GetDeltaB(void);
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms);
float Encoder_CountsToDistanceMm(int encoder_count_a, int encoder_count_b);
#endif
