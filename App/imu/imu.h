#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>

void IMU_Init(void);
void IMU_OnDataReadyIrq(void);
void IMU_Update(void);

uint8_t IMU_IsValid(void);
float IMU_GetYaw(void);
float IMU_GetPitch(void);
float IMU_GetRoll(void);

#endif
