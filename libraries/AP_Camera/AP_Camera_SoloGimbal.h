#pragma once

#include "AP_Camera_config.h"

#if AP_CAMERA_SOLOGIMBAL_ENABLED

#include "AP_Camera_Backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Camera_SoloGimbal : public AP_Camera_Backend
{
public:

    // Constructor
    using AP_Camera_Backend::AP_Camera_Backend; // coverity[UNINIT_CTOR] // Reviewed: safe in context

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_SoloGimbal);

    // entry point to actually take a picture.  returns true on success
    bool trigger_pic() override;

    // momentary switch to change camera between picture and video modes
    void cam_mode_toggle() override;

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg) override;

private:

    GOPRO_CAPTURE_MODE gopro_capture_mode = GOPRO_CAPTURE_MODE_VIDEO;
    GOPRO_HEARTBEAT_STATUS gopro_status = GOPRO_HEARTBEAT_STATUS_DISCONNECTED;
    bool gopro_is_recording = false;
    mavlink_channel_t heartbeat_channel = MAVLINK_COMM_0;
};

#endif // AP_CAMERA_SOLOGIMBAL_ENABLED
