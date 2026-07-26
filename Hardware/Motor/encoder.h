#ifndef _ENCODER_H
#define _ENCODER_H
#include "ti_msp_dl_config.h"
#include "board.h"

void Encoder_OnGpioIrq(uint32_t gpio_status_a, uint32_t gpio_status_b);
void Encoder_UpdateSample(void);
int Encoder_GetDeltaA(void);
int Encoder_GetDeltaB(void);
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms);
float Encoder_CountsToDistanceMm(int encoder_count_a, int encoder_count_b);
#endif
