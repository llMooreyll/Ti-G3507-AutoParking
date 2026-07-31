#include "Hardware/LineSensor/line_sensor.h"

#include "ti_msp_dl_config.h"

uint8_t LineSensor_ReadMask(void)
{
    uint8_t sensor_mask = 0U;
    uint32_t porta_state;
    uint32_t portb_state;

    porta_state = DL_GPIO_readPins(
        LINE_SENSOR_A_PORT,
        LINE_SENSOR_A_RIGHT_OUTER_PIN |
            LINE_SENSOR_A_RIGHT_INNER_PIN |
            LINE_SENSOR_A_CENTER_PIN);
    portb_state = DL_GPIO_readPins(
        LINE_SENSOR_B_PORT,
        LINE_SENSOR_B_LEFT_INNER_PIN |
            LINE_SENSOR_B_LEFT_OUTER_PIN);

    if ((porta_state & LINE_SENSOR_A_RIGHT_OUTER_PIN) != 0U)
    {
        sensor_mask |= LINE_SENSOR_RIGHT_OUTER;
    }
    if ((porta_state & LINE_SENSOR_A_RIGHT_INNER_PIN) != 0U)
    {
        sensor_mask |= LINE_SENSOR_RIGHT_INNER;
    }
    if ((porta_state & LINE_SENSOR_A_CENTER_PIN) != 0U)
    {
        sensor_mask |= LINE_SENSOR_CENTER;
    }
    if ((portb_state & LINE_SENSOR_B_LEFT_INNER_PIN) != 0U)
    {
        sensor_mask |= LINE_SENSOR_LEFT_INNER;
    }
    if ((portb_state & LINE_SENSOR_B_LEFT_OUTER_PIN) != 0U)
    {
        sensor_mask |= LINE_SENSOR_LEFT_OUTER;
    }

    return sensor_mask;
}
