#include "AP_Baro_MSP.h"

#if AP_BARO_MSP_ENABLED

AP_Baro_MSP::AP_Baro_MSP(AP_Baro &baro, uint8_t _msp_instance) :
    AP_Baro_Backend(baro),
    instance(0),
    msp_instance(_msp_instance),
    sum_pressure(0.0f),
    sum_temp(0.0f),
    count(0) // coverity[UNINIT_CTOR] // Reviewed: safe in context
{
    instance = _frontend.register_sensor();
    set_bus_id(instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_MSP,0,msp_instance,0));
}

// Read the sensor
void AP_Baro_MSP::update(void)
{
    if (count) {
        WITH_SEMAPHORE(_sem);
        _copy_to_frontend(instance, sum_pressure/count, sum_temp/count);
        sum_pressure = sum_temp = 0;
        count = 0;
    }
}

void AP_Baro_MSP::handle_msp(const MSP::msp_baro_data_message_t &pkt)
{
    if (pkt.instance != msp_instance) {
        // not for us
        return;
    }
    WITH_SEMAPHORE(_sem);
    sum_pressure += pkt.pressure_pa;
    sum_temp += pkt.temp*0.01;
    count++;
}

#endif // AP_BARO_MSP_ENABLED
