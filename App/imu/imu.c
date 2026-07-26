#include "imu.h"

#include "MPU6050.h"
#include "bsp_siic.h"

#define MPU6050_GYRO_2000DPS_SCALE (16.4f)

static volatile uint8_t imu_data_ready = 0;
static uint8_t imu_valid = 0;

void IMU_Init(void)
{
    pIICInterface_t siic = &User_sIICDev;

    siic->init();
    MPU6050_initialize();
    DMP_Init();

    imu_data_ready = 0;
    imu_valid = 0;
}

void IMU_OnDataReadyIrq(void)
{
    imu_data_ready = 1;
}

void IMU_Update(void)
{
    if (!imu_data_ready)
    {
        return;
    }

    imu_data_ready = 0;
    Read_DMP();

    mpu6050.pitch = Roll;
    mpu6050.roll = Pitch;
    mpu6050.yaw = Yaw;
    mpu6050.gyro.x = gyro[0];
    mpu6050.gyro.y = gyro[1];
    mpu6050.gyro.z = gyro[2];
    mpu6050.accel.x = accel[0];
    mpu6050.accel.y = accel[1];
    mpu6050.accel.z = accel[2];

    imu_valid = 1;
}

uint8_t IMU_IsValid(void)
{
    return imu_valid;
}

float IMU_GetYaw(void)
{
    return mpu6050.yaw;
}

float IMU_GetPitch(void)
{
    return mpu6050.pitch;
}

float IMU_GetRoll(void)
{
    return mpu6050.roll;
}

float IMU_GetGyroZ(void)
{
    return mpu6050.gyro.z / MPU6050_GYRO_2000DPS_SCALE;
}
