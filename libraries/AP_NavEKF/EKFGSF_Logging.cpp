#include "EKFGSF_yaw.h"

#include <AP_Logger/AP_Logger.h>
#include <cerrno>

#if HAL_LOGGING_ENABLED

#pragma GCC diagnostic ignored "-Wnarrowing"

void EKFGSF_yaw::Log_Write(uint64_t time_us, LogMessages id0, LogMessages id1, uint8_t core_index)
{
    if (!vel_fuse_running) {
        return;
    }

    static_assert(N_MODELS_EKFGSF >= 5, "Logging will break on <5 EKFGSF models");

    // parasoft-suppress BD-PB-ERRNO "Reviewed and safe"
    const struct log_KY0 ky0{
        LOG_PACKET_HEADER_INIT(id0),
        time_us                 : time_us,
        core                    : core_index,
        yaw_composite           : wrap_360(degrees(GSF.yaw)),
        yaw_composite_variance  : sqrtF(MAX(degrees(GSF.yaw_variance), 0.0f)), // parasoft-suppress BD-PB-ERRNO "Reviewed and safe"
        yaw0                    : wrap_360(degrees(EKF[0].X[2])),
        yaw1                    : wrap_360(degrees(EKF[1].X[2])),
        yaw2                    : wrap_360(degrees(EKF[2].X[2])),
        yaw3                    : wrap_360(degrees(EKF[3].X[2])), // coverity[PW.NARROWING_CONVERSION]
        yaw4                    : wrap_360(degrees(EKF[4].X[2])), // coverity[PW.NARROWING_CONVERSION]
        wgt0                    : GSF.weights[0], // coverity[PW.NARROWING_CONVERSION]
        wgt1                    : GSF.weights[1], // coverity[PW.NARROWING_CONVERSION]
        wgt2                    : GSF.weights[2], // coverity[PW.NARROWING_CONVERSION]
        wgt3                    : GSF.weights[3],
        wgt4                    : GSF.weights[4],
    };
    AP::logger().WriteBlock(&ky0, sizeof(ky0));

    const struct log_KY1 ky1{
        LOG_PACKET_HEADER_INIT(id1),
        time_us                 : time_us, // coverity[PW.NARROWING_CONVERSION]
        core                    : core_index, // coverity[PW.NARROWING_CONVERSION]
        ivn0                    : EKF[0].innov[0], // coverity[PW.NARROWING_CONVERSION]
        ivn1                    : EKF[1].innov[0], // coverity[PW.NARROWING_CONVERSION]
        ivn2                    : EKF[2].innov[0], // coverity[PW.NARROWING_CONVERSION]
        ivn3                    : EKF[3].innov[0], // coverity[PW.NARROWING_CONVERSION]
        ivn4                    : EKF[4].innov[0], // coverity[PW.NARROWING_CONVERSION]
        ive0                    : EKF[0].innov[1], // coverity[PW.NARROWING_CONVERSION]
        ive1                    : EKF[1].innov[1], // coverity[PW.NARROWING_CONVERSION]
        ive2                    : EKF[2].innov[1], // coverity[PW.NARROWING_CONVERSION]
        ive3                    : EKF[3].innov[1],
        ive4                    : EKF[4].innov[1],
    };
    AP::logger().WriteBlock(&ky1, sizeof(ky1));
}

#endif  // HAL_LOGGING_ENABLED
