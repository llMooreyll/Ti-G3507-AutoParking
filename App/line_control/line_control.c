#include "App/line_control/line_control.h"

#include "Hardware/LineSensor/line_sensor.h"

#define LINE_CONTROL_BASE_RPM (80.0f)

/*
 * Index bits are [PB16 PB17 PA22 PA24 PA27]. Based on the observed vehicle
 * response, positive commands turn left and negative commands turn right.
 * The original requested angles are reduced to 20% for gentler steering.
 * The table explicitly covers all 32 possible input combinations. For other
 * multi-sensor cases, the angle follows the geometric center of all active
 * sensors. Symmetric patterns therefore drive straight.
 */
static const float steering_angle_deg_by_mask[32] = {
    0.0f,   /* 00000: no line; handled as STOP below */
    -15.0f, /* 00001: PA27 */
    -6.0f,  /* 00010: PA24 */
    -9.0f,  /* 00011: PA24 + PA27 */
    0.0f,   /* 00100: PA22 */
    -6.0f,  /* 00101: PA22 + PA27 */
    -3.0f,  /* 00110: PA22 + PA24 */
    -6.0f,  /* 00111: PA22 + PA24 + PA27 */
    6.0f,   /* 01000: PB17 */
    -3.0f,  /* 01001: PB17 + PA27 */
    0.0f,   /* 01010: PB17 + PA24 */
    -4.0f,  /* 01011: PB17 + PA24 + PA27 */
    3.0f,   /* 01100: PB17 + PA22 */
    -2.0f,  /* 01101: PB17 + PA22 + PA27 */
    0.0f,   /* 01110: PB17 + PA22 + PA24 */
    -3.0f,  /* 01111: PB17 + PA22 + PA24 + PA27 */
    15.0f,  /* 10000: PB16 */
    0.0f,   /* 10001: PB16 + PA27 */
    3.0f,   /* 10010: PB16 + PA24 */
    -2.0f,  /* 10011: PB16 + PA24 + PA27 */
    6.0f,   /* 10100: PB16 + PA22 */
    0.0f,   /* 10101: PB16 + PA22 + PA27 */
    2.0f,   /* 10110: PB16 + PA22 + PA24 */
    -1.5f,  /* 10111: PB16 + PA22 + PA24 + PA27 */
    9.0f,   /* 11000: PB16 + PB17 */
    2.0f,   /* 11001: PB16 + PB17 + PA27 */
    4.0f,   /* 11010: PB16 + PB17 + PA24 */
    0.0f,   /* 11011: PB16 + PB17 + PA24 + PA27 */
    6.0f,   /* 11100: PB16 + PB17 + PA22 */
    1.5f,   /* 11101: PB16 + PB17 + PA22 + PA27 */
    3.0f,   /* 11110: PB16 + PB17 + PA22 + PA24 */
    0.0f    /* 11111: all five sensors */
};

static uint8_t last_sensor_mask;

void LineControl_GetCommand10ms(ChassisCommand *command)
{
    if (command == 0)
    {
        return;
    }

    last_sensor_mask = LineSensor_ReadMask();

    command->type = CHASSIS_COMMAND_STOP;
    command->base_rpm = 0.0f;
    command->relative_yaw_error_deg = 0.0f;
    command->distance_mm = 0.0f;
    command->delta_yaw_deg = 0.0f;
    command->action_id = 0U;

    /* All inputs low means that the line is lost, so stop safely. */
    if (last_sensor_mask == 0U)
    {
        return;
    }

    command->type = CHASSIS_COMMAND_CONTINUOUS_DRIVE;
    command->base_rpm = LINE_CONTROL_BASE_RPM;
    command->relative_yaw_error_deg =
        steering_angle_deg_by_mask[last_sensor_mask];
}

uint8_t LineControl_GetLastSensorMask(void)
{
    return last_sensor_mask;
}
