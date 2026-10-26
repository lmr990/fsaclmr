#include "CAN_AV.h"

CANFrame_t Compose_Request_Heartbeat(const Request_t *request) {
    CANFrame_t frame;
    frame.id = AV_Request_Heartbeat_ID;

    int16_t torque   = (int16_t)request->torque;
    int16_t steering = (int16_t)(request->steering * 10.0f);
    int16_t speed    = (int16_t)(request->speed * 10.0f);

    // big-endian packing
    frame.data[0] = (uint8_t)((torque >> 8) & 0xFF);
    frame.data[1] = (uint8_t)(torque & 0xFF);

    frame.data[2] = (uint8_t)((steering >> 8) & 0xFF);
    frame.data[3] = (uint8_t)(steering & 0xFF);

    frame.data[4] = (uint8_t)((speed >> 8) & 0xFF);
    frame.data[5] = (uint8_t)(speed & 0xFF);

    frame.data[6] = 0;
    frame.data[7] = 0;

    return frame;
}