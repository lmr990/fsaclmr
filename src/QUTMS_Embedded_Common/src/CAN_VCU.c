#include "CAN_VCU.h"

void Parse_VCU_TransmitSteering(const uint8_t *data,
                                int16_t *steering_0_raw,
                                int16_t *steering_1_raw,
                                uint16_t *adc_0,
                                uint16_t *adc_1) {
    // Data[0..1]: steering_0_raw (int16 big-endian)
    *steering_0_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);

    // Data[2..3]: steering_1_raw (int16 big-endian)
    *steering_1_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);

    // Data[4..5]: adc_0 (uint16 big-endian)
    *adc_0 = ((uint16_t)data[4] << 8) | data[5];

    // Data[6..7]: adc_1 (uint16 big-endian)
    *adc_1 = ((uint16_t)data[6] << 8) | data[7];
}