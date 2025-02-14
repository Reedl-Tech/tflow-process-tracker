#pragma once

#include <stdint.h>

class TFlowImu {

public:

    TFlowImu() {
        ts = 0;

        roll = 0;
        pitch = 0;
        yaw = 0;
        altitude_baro = 0;

        is_valid = 0;
    }

// packet received from TFlow Capture or TFlowPlayer
#pragma pack(push,1)
    struct flyn_imu_v0 {
        uint32_t sign;
    };

#pragma pack(pop)
    uint32_t ts;

    float roll;
    float pitch;
    float yaw;
    float altitude_baro;

    int is_valid;

    void getIMU(uint8_t* aux_data, uint32_t aux_data_len) {};
    void getIMU_flyn_v0(const TFlowImu::flyn_imu_v0* imu_in) {};
};


