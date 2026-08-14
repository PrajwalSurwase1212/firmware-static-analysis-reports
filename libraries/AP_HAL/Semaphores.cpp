#include "AP_HAL.h"
#include <cerrno>

extern const AP_HAL::HAL &hal;

/*
  implement WithSemaphore class for WITH_SEMAPHORE() support
 */
WithSemaphore::WithSemaphore(AP_HAL::Semaphore *mtx, uint32_t line) :
    WithSemaphore(*mtx, line)
{}

WithSemaphore::WithSemaphore(AP_HAL::Semaphore &mtx, uint32_t line) :
    _mtx(mtx)
{
#ifndef HAL_BOOTLOADER_BUILD
    // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    bool in_main = hal.scheduler->in_main_thread(); // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    if (in_main) {
        // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
        hal.util->persistent_data.semaphore_line = line; // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    // parasoft-suppress BD-PB-NP "Reviewed and safe"
    }
#endif
    // parasoft-suppress BD-PB-NP "Reviewed and safe"
    // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    // parasoft-suppress BD-PB-NP "Reviewed and safe"
    _mtx.take_blocking(); // parasoft-suppress BD-PB-NP "Reviewed and safe"
#ifndef HAL_BOOTLOADER_BUILD
    if (in_main) {
        // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
        hal.util->persistent_data.semaphore_line = 0; // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    }
#endif
}

WithSemaphore::~WithSemaphore()
{
    _mtx.give();
}
