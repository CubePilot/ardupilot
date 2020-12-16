#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#define AP_BATTMONITOR_BQ_I2C_ADDR               0x08

class AP_BattMonitor_BQ769x0 : public AP_BattMonitor_Backend
{
public:

    // Constructor
    AP_BattMonitor_BQ769x0(AP_BattMonitor &mon,
                            AP_BattMonitor::BattMonitor_State &mon_state,
                            AP_BattMonitor_Params &params,
                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool has_cell_voltages() const override { return true; }

    // all smart batteries are expected to provide current
    bool has_current() const override { return true; }

    void init(void) override;

    bool power_on() override;
    bool power_off() override;
    bool start_charging() override;
    bool stop_charging() override;
    bool is_balancing() override { return (balancing_status != 0); }
    bool is_charging() override { return charging; }

private:

    void read(void) override;

    // void handle_alert(uint8_t pin, bool high, uint32_t timestamp_us);
    void timer(void);

    bool read_byte(uint8_t reg, uint8_t& val);
    bool write_byte(uint8_t reg, uint8_t val);

    bool read_adc_gain();
    bool read_adc_offset();
    bool read_OVTrip(float& over);
    bool write_OVTrip(float trip_voltage);
    bool read_UVTrip(float& under);
    bool write_UVTrip(float tripVoltage);
    uint8_t tripCalculator(float tripVoltage);
    bool read_cell_voltage(uint8_t cell_number, uint16_t &voltage);
    bool read_temp(float& temp);
    float thermistor_lookup(float resistance);
    bool read_current(float &current_A);
    bool read_pack_voltage(float &pack_voltage);
    bool check_status(uint8_t sys_stat);
    void do_cell_balance();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    // read_block - returns number of characters read if successful, zero if unsuccessful
    uint8_t read_block(uint8_t reg, uint8_t* data, uint8_t max_len) const;

    AP_HAL::Device::PeriodicHandle timer_handle;
    enum bq_type {
        BQ76920 = 0,
    };
    const bq_type _bq_type = BQ76920; // Currently only BQ76920 supported
    const uint8_t _num_cells = 4; // Currently only four cells supported

    // Battery Monitor Vars
    float _adc_gain;
    int16_t _adc_offset;
    uint32_t _start_tempread_wait;
    uint32_t idle_start_ms;
    bool charging;
    uint8_t balancing_status;
};
