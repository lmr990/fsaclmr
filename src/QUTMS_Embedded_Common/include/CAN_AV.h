#ifndef CAN_AV_H
#define CAN_AV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Generic CAN frame structure used by QUTMS embedded protocol helpers.
typedef struct {
    uint32_t id;
    uint8_t data[8];
} CANFrame_t;

// Autonomous vehicle control request (sent to DVL / VCU as a heartbeat).
typedef struct {
    float torque;    // -100 .. 100  (percentage of max motor torque)
    float steering;  // degrees
    float speed;     // m/s
} Request_t;

// QUTMS AV CAN frame IDs
#define AV_Request_Heartbeat_ID       0x700

// Serialise a Request_t into a CAN frame (8 bytes big-endian).
//   Data[0..1]: torque   (int16 raw percentage)
//   Data[2..3]: steering (int16 deg * 10)
//   Data[4..5]: speed    (int16 m/s * 10)
//   Data[6..7]: reserved
CANFrame_t Compose_Request_Heartbeat(const Request_t *request);

#ifdef __cplusplus
}
#endif

#endif  // CAN_AV_H