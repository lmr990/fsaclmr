#ifndef CAN_VESC_H
#define CAN_VESC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// VESC CAN packet types (from VESC firmware standard CAN protocol)
// The full CAN ID is constructed as: (packet_type << 8) | controller_id
#define VESC_CAN_PACKET_STATUS          0x09
#define VESC_CAN_PACKET_SET_DUTY        0x00
#define VESC_CAN_PACKET_SET_CURRENT     0x01
#define VESC_CAN_PACKET_SET_CURRENT_BRAKE 0x02
#define VESC_CAN_PACKET_SET_RPM         0x03
#define VESC_CAN_PACKET_SET_POS         0x04

// Parse VESC status packet (8 bytes big-endian)
//   Data[0..3]: RPM       (int32)
//   Data[4..5]: Current   (int16 scaled by 10)   -> current = value / 10.0
//   Data[6..7]: Duty      (int16 scaled by 1000) -> duty    = value / 1000.0
void Parse_VESC_CANPacketStatus(const uint8_t *data, int32_t *rpm, float *current, float *duty);

#ifdef __cplusplus
}
#endif

#endif  // CAN_VESC_H