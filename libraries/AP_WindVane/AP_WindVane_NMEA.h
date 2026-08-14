/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_WindVane_config.h"

#if AP_WINDVANE_NMEA_ENABLED

#include "AP_WindVane_Backend.h"

class AP_WindVane_NMEA : public AP_WindVane_Backend
{
public:
    // constructor
    using AP_WindVane_Backend::AP_WindVane_Backend; // coverity[UNINIT_CTOR] // Reviewed: safe in context

    // initialization
    void init(const AP_SerialManager& serial_manager) override;

    // update state
    void update_direction() override;
    void update_speed() override;

private:
    // pointer to serial uart
    AP_HAL::UARTDriver *uart = nullptr; 

    // See if we can read in some data
    void update();

    // try and decode NMEA message
    bool decode(char c);

    // decode each term
    bool decode_latest_term();

    // latest values read in
    float _speed_ms{0.0f};
    float _wind_dir_deg{0.0f};

    char _term[15]{};            // buffer for the current term within the current sentence
    uint8_t _term_offset{0};      // offset within the _term buffer where the next character should be placed
    uint8_t _term_number{0};      // term index within the current sentence
    uint8_t _checksum{0};         // checksum accumulator
    bool _term_is_checksum{false};    // current term is the checksum
    bool _sentence_valid{false};      // is current sentence valid so far
    bool _sentence_done{false};       // true if this sentence has already been decoded
};

#endif  // AP_WINDVANE_NMEA_ENABLED
