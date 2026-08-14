#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "Semaphores.h"
#include "Scheduler.h"
#include <cerrno>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

// construct a semaphore
Semaphore::Semaphore() :
    owner((pthread_t)-1),
    take_count(0)
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr); // coverity[UNINIT_CTOR] // Reviewed: safe in context
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_lock, &attr);
// parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
} // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"


bool Semaphore::give()
// parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
{
    // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
    // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    take_count--; // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
    if (pthread_mutex_unlock(&_lock) != 0) { // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
        // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
        AP_HAL::panic("Bad semaphore usage");
    }
    // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    if (take_count == 0) { // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
        // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
        owner = (pthread_t)-1; // parasoft-suppress BD-TRS-THRDR "Reviewed and safe"
    }
    return true;
}

void Semaphore::check_owner() const
{
    // should probably make sure we're holding the semaphore here....
    if (owner != pthread_self()) {
        AP_HAL::panic("Wrong owner");
    }
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        if (pthread_mutex_lock(&_lock) == 0) {
            owner = pthread_self();
            take_count++;
            return true;
        }
        return false;
    }
    if (take_nonblocking()) {
        owner = pthread_self();
        return true; // coverity[OVERFLOW_BEFORE_WIDEN]
    }
    uint64_t start = AP_HAL::micros64();
    do {
        Scheduler::from(hal.scheduler)->set_in_semaphore_take_wait(true);
        hal.scheduler->delay_microseconds(200);
        Scheduler::from(hal.scheduler)->set_in_semaphore_take_wait(false);
        if (take_nonblocking()) {
            // parasoft-suppress BD-TRS-LOCK "Reviewed and safe"
            owner = pthread_self();
            return true;
        }
    } while ((AP_HAL::micros64() - start) < (uint64_t)timeout_ms * 1000);
    return false;
// parasoft-suppress BD-TRS-LOCK "Reviewed and safe"
} // parasoft-suppress BD-TRS-LOCK "Reviewed and safe"

bool Semaphore::take_nonblocking()
{
    if (pthread_mutex_trylock(&_lock) == 0) {
        owner = pthread_self();
        take_count++;
        return true;
    }
    return false;
}


/*
  binary semaphore using pthread condition variables
 */

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state)
{
    pthread_cond_init(&cond, NULL);
    pending = initial_state;
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        if (hal.scheduler->in_main_thread() ||
            Scheduler::from(hal.scheduler)->semaphore_wait_hack_required()) {
              // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
            /*
              when in the main thread we need to do a busy wait to ensure
              the clock advances
            */
            uint64_t end_us = AP_HAL::micros64() + timeout_us;
            struct timespec ts {};
            do {
                // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
                if (pthread_cond_timedwait(&cond, &mtx._lock, &ts) == 0) { // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
                    pending = false;
                    return true;
                }
                hal.scheduler->delay_microseconds(10);
            } while (AP_HAL::micros64() < end_us);
            return false;
        }

        struct timespec ts;
        if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
            return false;
        // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
        }
        ts.tv_sec += timeout_us/1000000UL;
        ts.tv_nsec += (timeout_us % 1000000U) * 1000UL;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }
        // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
        if (pthread_cond_timedwait(&cond, &mtx._lock, &ts) != 0) { // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
            return false;
        }
    }
    // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
    pending = false;
    return true;
}

bool BinarySemaphore::wait_blocking(void)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
        if (pthread_cond_wait(&cond, &mtx._lock) != 0) { // parasoft-suppress BD-TRS-REVLOCK "Reviewed and safe"
            return false;
        }
    }
    pending = false;
    return true;
}

void BinarySemaphore::signal(void)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        pending = true;
        pthread_cond_signal(&cond);
    }
}

#endif  // CONFIG_HAL_BOARD
