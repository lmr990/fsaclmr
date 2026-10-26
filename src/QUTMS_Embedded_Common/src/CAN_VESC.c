#include "CAN_VESC.h"

void Parse_VESC_CANPacketStatus(const uint8_t *data, int32_t *rpm, float *current, float *duty) {
    // Data[0..3]: RPM (int32 big-endian)
    *rpm = ((int32_t)data[0] << 24) |
           ((int32_t)data[1] << 16) |
           ((int32_t)data[2] << 8)  |
           (int32_t)data[3];

    // Data[4..5]: Current (int16 big-endian scaled by 10)
    int16_t current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    *current = current_raw / 10.0f;

    // Data[6..7]: Duty cycle (int16 big-endian scaled by 1000)
    int16_t duty_raw = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
    *duty = duty_raw / 1000.0f;
}