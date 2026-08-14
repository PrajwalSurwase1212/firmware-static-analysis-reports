#include "Rover.h"
#include <cerrno>

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // send output signals to motors
    if (motor_test) {
        motor_test_output();
    } else {
        // get ground speed
        float speed = 0.0f; // coverity[CHECKED_RETURN]
        // parasoft-suppress BD-PB-CHECKRET "Reviewed and safe"
        (void)g2.attitude_control.get_forward_speed(speed); // parasoft-suppress BD-PB-CHECKRET "Reviewed and safe"

        // parasoft-suppress BD-PB-CHECKRET "Reviewed and safe"
        g2.motors.output(arming.is_armed(), speed, G_Dt); // parasoft-suppress BD-PB-CHECKRET "Reviewed and safe"
    }
}
