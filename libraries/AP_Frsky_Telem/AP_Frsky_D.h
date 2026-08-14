#pragma once

#include "AP_Frsky_Backend.h"

#if AP_FRSKY_D_TELEM_ENABLED

class AP_Frsky_D : public AP_Frsky_Backend
{

public:

    using AP_Frsky_Backend::AP_Frsky_Backend; // coverity[UNINIT_CTOR] // Reviewed: safe in context

protected:

    void send() override;
    uint32_t initial_baud() const override
    {
        return 9600;
    }

private:

    // methods related to the nuts-and-bolts of sending data
    void send_byte(uint8_t value);
    void send_uint16(uint16_t id, uint16_t data);

    struct {
        uint32_t last_200ms_frame{0};
        uint32_t last_1000ms_frame{0};
    } _D;

};

#endif  // AP_FRSKY_D_TELEM_ENABLED
