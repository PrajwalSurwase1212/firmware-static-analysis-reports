#pragma once

#include "AP_Generator_Backend.h"

#if AP_GENERATOR_IE_ENABLED

#include <AP_Logger/AP_Logger_config.h>
#include <GCS_MAVLink/GCS.h>

class AP_Generator_IE_FuelCell : public AP_Generator_Backend
{

public:
    // Constructor
    using AP_Generator_Backend::AP_Generator_Backend; // coverity[UNINIT_CTOR] // Reviewed: safe in context

    // Initialize the fuel cell driver
    __INITFUNC__ void init(void) override;

    // Check if readings are healthy
    bool healthy(void) const override { return _healthy; }

    // Check for arming
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override;

    // Update fuel cell, expected to be called at 20hz
    void update(void) override;

protected:

    // Pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr;

    // IE fuel cell running states
    enum class State {
        STARTING       = 0,
        READY          = 1,
        RUNNING        = 2,
        FAULT          = 3,
        BATTERY_ONLY   = 8,
    };

    // State enum to string lookup
    struct Lookup_State {
        State option;
        const char *msg_txt;
    };

    static const Lookup_State lookup_state[];

    uint32_t _err_code = 0;       // The error code from fuel cell
    uint32_t _last_err_code = 0;  // The previous error code from fuel cell
    uint32_t _sub_err_code = 0;       // The sub error code from fuel cell
    uint32_t _last_sub_err_code = 0;  // The previous sub error code from fuel cell
    State _state = State::STARTING;          // The PSU state
    State _last_state = State::STARTING;     // The previous PSU state
    uint32_t _last_time_ms = 0;   // Time we got a reading
    bool _healthy = false;        // Is the driver working
    uint32_t _health_warn_last_ms = 0; // Time to persist warning message without spamming

    // Temporary state params
    struct ParsedValue {
        float tank_pct = 0.0f;
        uint16_t tank_bar = 0;
        float battery_pct = 0.0f;
        float battery_volt = 0.0f;
        int16_t pwr_out = 0;
        uint16_t spm_pwr = 0;
        int16_t battery_pwr = 0;
        uint8_t state = 0;
        uint32_t err_code = 0;
        uint32_t sub_err_code = 0;
    } _parsed {};

    // Constants
    static const uint8_t TERM_BUFFER = 33; // Max length of term we expect
    static const uint16_t HEALTHY_TIMEOUT_MS = 5000; // Time for driver to be marked un-healthy

    // Decoding vars
    char _start_char = '\0';           // inital sentence character giving sentence type
    char _term[TERM_BUFFER] {};    // Term buffer
    bool _sentence_valid = false;       // Is current sentence valid
    bool _data_valid = false;           // Is data within expected limits
    uint8_t _term_number = 0;       // Term index within the current sentence
    uint8_t _term_offset = 0;       // Offset within the _term buffer where the next character should be placed
    bool _in_string = false;            // True if we should be decoding
    uint8_t _checksum = 0;          // Basic checksum used by V2 protocol

    // Assigns the unit specific measurements once a valid sentence is obtained
    virtual void assign_measurements(const uint32_t now) = 0;

#if HAL_LOGGING_ENABLED
    virtual void log_write(void) {}
#endif

    // Add a single character to the buffer and attempt to decode.
    // Returns true if a complete sentence was successfully decoded or if the buffer is full.
    bool decode(char c);

    // Unit specific decoding to process characters received and build sentence
    virtual void decode_latest_term(void) = 0;

    // Check if we should notify on any change of fuel cell state
    virtual void check_status(const uint32_t now);

    // Check error codes and populate message with error code
    virtual bool check_for_err_code(char* msg_txt, uint8_t msg_len) const = 0;

    // Check if we have received an warning code and populate message with warning code
    virtual bool check_for_warning_code(char* msg_txt, uint8_t msg_len) const { return false; }

    // Only check the error code if it has changed since we last checked
    void check_for_err_code_if_changed();

    // Return true is fuel cell is in running state suitable for arming
    virtual bool is_running() const;

    // Print msg to user updating on state change
    virtual void update_state_msg();

#if HAL_GCS_ENABLED
    // Get the MAV_SEVERITY level of a given error code
    virtual MAV_SEVERITY get_mav_severity(uint32_t err_code) const { return MAV_SEVERITY_ALERT; }
#endif

};
#endif  // AP_GENERATOR_IE_ENABLED
