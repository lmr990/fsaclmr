#ifndef CAN_VCU_H
#define CAN_VCU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// QUTMS VCU CAN frame IDs
#define VCU_TransmitSteering_ID         0x703

// Parse VCU steering feedback packet (8 bytes big-endian)
//   Data[0..1]: steering_0_raw  (int16 scaled by 10) -> deg = value / 10.0
//   Data[2..3]: steering_1_raw  (int16 scaled by 10) -> deg = value / 10.0
//   Data[4..5]: adc_0           (uint16)
//   Data[6..7]: adc_1           (uint16)
void Parse_VCU_TransmitSteering(const uint8_t *data,
                                int16_t *steering_0_raw,
                                int16_t *steering_1_raw,
                                uint16_t *adc_0,
                                uint16_t *adc_1);

#ifdef __cplusplus
}
#endif

#endif  // CAN_VCU_H