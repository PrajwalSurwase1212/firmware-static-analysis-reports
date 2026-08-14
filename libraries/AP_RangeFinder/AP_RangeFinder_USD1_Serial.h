#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_USD1_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_USD1_Serial : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_USD1_Serial(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    // baudrate used during object construction:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    uint16_t rx_bufsize() const override { return 128; }
    uint16_t tx_bufsize() const override { return 128; }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial; // coverity[UNINIT_CTOR] // Reviewed: safe in context

    // detect USD1_Serial Firmware Version
    bool detect_version(void);

    // get a reading
    bool get_reading(float &reading_m) override;

    uint8_t  _linebuf[6]{};
    uint8_t  _linebuf_len{0};
    bool     _version_known{false};
    uint8_t  _header{0};
    uint8_t  _version{0};
};

#endif  // AP_RANGEFINDER_USD1_SERIAL_ENABLED
