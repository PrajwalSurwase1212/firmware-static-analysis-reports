#include "Storage.h"

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL.h"

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <cerrno>

#ifndef HAL_STORAGE_FILE
#if APM_BUILD_TYPE(APM_BUILD_Replay)
#define HAL_STORAGE_FILE "eeprom-replay.bin"
#elif APM_BUILD_TYPE(APM_BUILD_AP_Periph)
#define HAL_STORAGE_FILE "eeprom-periph.bin"
#else
#define HAL_STORAGE_FILE "eeprom.bin"
#endif
#endif

using namespace HALSITL;

extern HAL_SITL& hal;

/*
  emulate flash sector sizes
 */
#ifndef HAL_FLASH_SECTOR_SIZE
#if HAL_STORAGE_SIZE <= 16384
#define HAL_FLASH_SECTOR_SIZE (16*1024)
#elif HAL_STORAGE_SIZE <= 32768
#define HAL_FLASH_SECTOR_SIZE (32*1024)
#else
#define HAL_FLASH_SECTOR_SIZE (128*1024)
#endif
#endif

#ifndef HAL_FLASH_MIN_WRITE_SIZE
#define HAL_FLASH_MIN_WRITE_SIZE 1
#endif

#ifndef HAL_FLASH_ALLOW_UPDATE
#define HAL_FLASH_ALLOW_UPDATE 1
#endif

void Storage::_storage_open(void)
{
    if (_initialisedType != StorageBackend::None) {
        // don't reinit
        return;
    }

    _dirty_mask.clearall();

#define HAL_RAMTRON_ALLOW_FALLBACK 0

#if STORAGE_USE_FRAM
    if (hal.get_storage_fram_enabled()) {
        if (fram.init() && fram.read(0, _buffer, HAL_STORAGE_SIZE)) {
//        _save_backup();  // FIXME
            _initialisedType = StorageBackend::FRAM;
            return;
        }

#if !HAL_RAMTRON_ALLOW_FALLBACK
        AP_HAL::panic("Unable to init RAMTRON storage");
#endif
    }
#endif

#if STORAGE_USE_FLASH
    if (hal.get_storage_flash_enabled()) {
        // load from storage backend
        _flash_load();
        _initialisedType = StorageBackend::Flash;
        return;
    }
#endif // STORAGE_USE_FLASH

#if STORAGE_USE_POSIX
    if (hal.get_storage_posix_enabled()) {
        // if we have failed filesystem init don't try again (this is
        // initialised to zero in the constructor)
        if (log_fd == -1) {
            return;
        }

        log_fd = open(HAL_STORAGE_FILE, O_RDWR|O_CREAT, 0644);
        if (log_fd == -1) {
            hal.console->printf("open failed of " HAL_STORAGE_FILE "\n");
            return;
        }
 // coverity[CHECKED_RETURN]
        if (fcntl(log_fd, F_SETFD, FD_CLOEXEC) == -1) {
            hal.console->printf("fcntl failed to set FD_CLOEXEC for " HAL_STORAGE_FILE "\n");
        }

        int ret = read(log_fd, _buffer, HAL_STORAGE_SIZE);
        if (ret < 0) {
            hal.console->printf("read failed for " HAL_STORAGE_FILE "\n");
            close(log_fd);
            log_fd = -1;
            return;
        }
        // pre-fill to full size
        if (lseek(log_fd, ret, SEEK_SET) != ret ||
            write(log_fd, &_buffer[ret], HAL_STORAGE_SIZE-ret) != HAL_STORAGE_SIZE-ret) {
            hal.console->printf("setup failed for " HAL_STORAGE_FILE "\n");
            close(log_fd);
            log_fd = -1;
            return;
        }
        _initialisedType = StorageBackend::SDCard;  // AKA POSIX
        return;
    }
#endif

//    ::printf("No storage backend enabled");
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    if (length == 0) {
        return;
    }
    uint16_t end = loc + length - 1;
    for (uint16_t line=loc>>STORAGE_LINE_SHIFT;
         line <= end>>STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    _storage_open();
    // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
    // parasoft-suppress BD-PB-OVERFWR "Reviewed and safe"
    // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
    memcpy(dst, &_buffer[loc], n); // parasoft-suppress BD-PB-OVERFWR "Reviewed and safe" // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
    // parasoft-suppress BD-PB-OVERFWR "Reviewed and safe"
    // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
    if (memcmp(src, &_buffer[loc], n) != 0) { // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
        // parasoft-suppress BD-PB-OVERFWR "Reviewed and safe"
        // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
        _storage_open();
        // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe"
        // parasoft-suppress BD-PB-OVERFWR "Reviewed and safe"
        memcpy(&_buffer[loc], src, n); // parasoft-suppress BD-PB-OVERFRD "Reviewed and safe" // parasoft-suppress BD-PB-OVERFWR "Reviewed and safe"
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (_initialisedType == StorageBackend::None) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

#if STORAGE_USE_FRAM
        if (fram.write(STORAGE_LINE_SIZE*i, &_buffer[STORAGE_LINE_SIZE*i], STORAGE_LINE_SIZE)) {
            _dirty_mask.clear(i);
            return;
        }
#endif

#if STORAGE_USE_POSIX
    if (hal.get_storage_posix_enabled()) {
        if (log_fd != -1) {
            const off_t offset = STORAGE_LINE_SIZE*i;
            if (lseek(log_fd, offset, SEEK_SET) != offset) {
                return;
            }
            if (write(log_fd, &_buffer[offset], STORAGE_LINE_SIZE) != STORAGE_LINE_SIZE) {
                return;
            }
            _dirty_mask.clear(i);
            return;
        }
    }
#endif

#if STORAGE_USE_FLASH
    if (hal.get_storage_flash_enabled()) {
        // save to storage backend
        _flash_write(i);
        return;
    }
#endif
}

#if STORAGE_USE_FLASH

/*
  load all data from flash
 */
void Storage::_flash_load(void)
{
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
}

/*
  write one storage line. This also updates _dirty_mask. 
*/
void Storage::_flash_write(uint16_t line)
{
    if (_flash.write(line*STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
        // mark the line clean
        _dirty_mask.clear(line);
    }
}


/*
  emulate writing to flash
 */
static int flash_fd = -1;

static uint32_t sitl_flash_getpageaddr(uint32_t page)
{
    return page * HAL_FLASH_SECTOR_SIZE;
}
 // coverity[STACK_USE]
static void sitl_flash_open(void)
{
    // parasoft-suppress BD-TRS-FRC "Reviewed and safe"
    if (flash_fd == -1) {
        // parasoft-suppress BD-TRS-FRC "Reviewed and safe"
        flash_fd = open("flash.dat", O_RDWR, 0644);
        if (flash_fd == -1) {
            // parasoft-suppress BD-TRS-FRC "Reviewed and safe"
            flash_fd = open("flash.dat", O_RDWR|O_CREAT, 0644); // parasoft-suppress BD-TRS-FRC "Reviewed and safe"
            // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
            if (flash_fd == -1) {
                AP_HAL::panic("Failed to open flash.dat");
            // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
            // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
            }
            if (ftruncate(flash_fd, 2*HAL_FLASH_SECTOR_SIZE) != 0) {
                // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
                // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
                AP_HAL::panic("Failed to create flash.dat"); // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
            }
            // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
            uint8_t fill[HAL_FLASH_SECTOR_SIZE*2]; // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
            // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
            memset(fill, 0xff, sizeof(fill));
            if (pwrite(flash_fd, fill, sizeof(fill), 0) != (ssize_t)sizeof(fill)) {
                // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
                AP_HAL::panic("Failed to fill flash.dat"); // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
            }
        // parasoft-suppress BD-PB-VLASIZE "Reviewed and safe"
        }
    }
// parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
}

static bool sitl_flash_write(uint32_t addr, const uint8_t *data, uint32_t length)
{
    // parasoft-suppress BD-PB-VLASIZE "Reviewed and safe"
    // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
    sitl_flash_open();
    // parasoft-suppress BD-PB-VLASIZE "Reviewed and safe"
    // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
    uint8_t old[length]; // parasoft-suppress BD-PB-VLASIZE "Reviewed and safe"
    if (pread(flash_fd, old, length, addr) != (ssize_t)length) {
        // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
        AP_HAL::panic("Failed to read flash.dat at %u len=%u", unsigned(addr), unsigned(length)); // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
    }
// parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
#if defined(HAL_FLASH_MIN_WRITE_SIZE) && HAL_FLASH_MIN_WRITE_SIZE == 32
    if ((length % HAL_FLASH_MIN_WRITE_SIZE) != 0 || (addr % HAL_FLASH_MIN_WRITE_SIZE) != 0) {
        // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
        AP_HAL::panic("Attempt to write flash at %u length %u", addr, length); // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
    }
    // emulate H7 requirement that writes be to untouched bytes
    for (uint32_t i=0; i<length; i+= 32) {
        if (memcmp(&old[i], &data[i], 32) == 0) {
            continue;
        }
        // parasoft-suppress BD-RES-LEAKS "Reviewed and safe" // coverity[STACK_USE]
        for (uint32_t j=0; j<32; j++) {
            if (old[i+j] != 0xFF) {
                // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
                AP_HAL::panic("Attempt to write modified flash at %u length %u", addr+i+j, length); // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
            }
        }
    }
#endif
    // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
    // check that we are only ever clearing bits (real flash storage can only ever clear bits,
    // except for an erase
    for (uint32_t i=0; i<length; i++) {
#if HAL_FLASH_ALLOW_UPDATE
        // emulating flash that allows setting any bit from 1 to 0
        if (data[i] & ~old[i]) {
            AP_HAL::panic("Attempt to set flash byte 0x%02x from 0x%02x at %u", data[i], old[i], addr+i);
        }
#else
        // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
        // emulating flash that only allows update if whole byte is 0xFF
        if (data[i] != old[i] && old[i] != 0xFF) {
            // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
            AP_HAL::panic("Attempt to set flash byte 0x%02x from 0x%02x at %u", data[i], old[i], addr+i); // parasoft-suppress BD-RES-LEAKS "Reviewed and safe"
        }
#endif
    // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
    }
    return pwrite(flash_fd, data, length, addr) == (ssize_t)length;
}

// parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"

static bool sitl_flash_read(uint32_t addr, uint8_t *data, uint32_t length)
{
    sitl_flash_open();
    return pread(flash_fd, data, length, addr) == (ssize_t)length;
}

// parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
static bool sitl_flash_erasepage(uint32_t page)
{
    // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
    uint8_t fill[HAL_FLASH_SECTOR_SIZE]; // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
    // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
    memset(fill, 0xff, sizeof(fill));
    sitl_flash_open();
    // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
    bool ret = pwrite(flash_fd, fill, sizeof(fill), page * HAL_FLASH_SECTOR_SIZE) == sizeof(fill); // parasoft-suppress BD-RES-STACKLIM "Reviewed and safe"
    printf("erase %u -> %u\n", page, ret);
    return ret;
}

/*
  callback to write data to flash
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    size_t base_address = sitl_flash_getpageaddr(sector);
    bool ret = sitl_flash_write(base_address+offset, data, length);
    if (!ret && _flash_erase_ok()) {
        // we are getting flash write errors while disarmed. Try
        // re-writing all of flash
        uint32_t now = AP_HAL::millis();
        if (now - _last_re_init_ms > 5000) {
            _last_re_init_ms = now;
            bool ok = _flash.re_initialise();
            hal.console->printf("Storage: failed at %u:%u for %u - re-init %u\n",
                                (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
        }
    }
    return ret;
}

/*
  callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    size_t base_address = sitl_flash_getpageaddr(sector);
    return sitl_flash_read(base_address+offset, data, length);
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
    return sitl_flash_erasepage(sector);
}

/*
  callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
    // only allow erase while disarmed
    return !hal.util->get_soft_armed();
}

#endif // STORAGE_USE_FLASH

/*
  consider storage healthy if we have nothing to write sometime in the
  last 2 seconds
 */
bool Storage::healthy(void)
{
    if (_initialisedType == StorageBackend::None) {
        return false;
    }
    return AP_HAL::millis() - _last_empty_ms < 2000;
}

/*
  get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (_initialisedType==StorageBackend::None) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}
