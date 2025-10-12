#pragma once

#include <stdint.h>

class TFlowImu {

public:

    TFlowImu() {
        ts = 0;
        memset(&ap_imu, 0, sizeof(ap_imu));
        is_valid = 0;
    }

// packet received from TFlow Capture or TFlowPlayer
#pragma pack(push,1)

    struct imu_milesi_v0 {
        uint32_t sign;        // IMU1   0x494D5531
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp

        /* Direct translation from Mavlink ATTITUDE_QUATERNION */
        float qw;
        float qx;
        float qy;
        float qz;
        float rollspeed;    /*< [rad/s] Roll angular speed*/
        float pitchspeed;   /*< [rad/s] Pitch angular speed*/
        float yawspeed;     /*< [rad/s] Yaw angular speed*/

        /* Direct translation from Mavlink ATTITUDE */ 
        float roll;         /*< [rad] Roll angle (-pi..+pi)*/
        float pitch;        /*< [rad] Pitch angle (-pi..+pi)*/
        float yaw;          /*< [rad] Yaw angle (-pi..+pi)*/

        /* TODO: Add camera position from AP or Sensor's IMU */
    } ap_imu;

#pragma pack(pop)
    uint32_t ts;

    int is_valid;

    int getData(const uint8_t* aux_data, uint32_t aux_data_len);
};
