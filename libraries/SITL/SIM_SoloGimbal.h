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
/*
  gimbal simulator class

./Tools/autotest/sim_vehicle.py -D -G -v Kala --mavlink-gimbal
param set MNT1_TYPE 2
param set RC6_OPTION 213  # MOUNT1_PITCH
rc 6 1818  # for neutral pitch input
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SOLOGIMBAL_ENABLED

#include "SIM_Gimbal.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/utility/Socket_native.h>

namespace SITL {

class SoloGimbal {
public:

    SoloGimbal() {} // coverity[UNINIT_CTOR] // Reviewed: safe in context
    void update(const Aircraft &aicraft);

private:

    const char *target_address = "127.0.0.1";
    const uint16_t target_port = 5762;

    // physic simulation of gimbal:
    Gimbal gimbal;

    // reporting variables. gimbal pushes these to vehicle code over
    // MAVLink at approx 100Hz

    // reporting period in ms
    const float reporting_period_ms = 10;

    uint32_t last_report_us{0};
    uint32_t last_heartbeat_ms{0};
    bool seen_heartbeat{false};
    bool seen_gimbal_control{false};
    uint8_t vehicle_system_id{0};
    uint8_t vehicle_component_id{0};

    SocketAPM_native mav_socket{false};
    struct {
        // socket to telem2 on aircraft
        bool connected{false};
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq{0};
    } mavlink;

    uint32_t param_send_last_ms{0};
    uint8_t param_send_idx{0};

    // component ID we send from:
    const uint8_t gimbal_component_id = 154;  // MAV_COMP_ID_GIMBAL

    void send_report(void);
    void param_send(const struct gimbal_param *p);
    struct gimbal_param *param_find(const char *name);
};

}  // namespace SITL

#endif  // AP_SIM_SOLOGIMBAL_ENABLED

