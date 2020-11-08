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
#include <AP_HAL/AP_HAL.h>
#if defined(HAL_BOARD_AP_PERIPH_HEREPRO)
#include "../AP_Periph.h"
#include "GCS_Mavlink.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_RTC/AP_RTC.h>

#define NUM_CAN_IFACES 3

class HerePro_FW : public AP_Periph_FW {
    GCS_HerePro _gcs;
    AP_Notify notify;
    AP_InertialSensor ins;
    
    AP_Logger logger;
    static const struct LogStructure log_structure[];

    AP_AHRS_NavEKF ahrs;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AP_Scheduler scheduler;

    AP_Scripting scripting;

    AP_RTC rtc;

    AP_HAL::AnalogSource *_adc0;
    AP_HAL::AnalogSource *_adc1;
    AP_HAL::AnalogSource *_adc2;
    AP_HAL::AnalogSource *_adc3;

    bool start_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }
    void exit_mission_callback() { return; }
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }
    void log_init(void);
    static void scheduler_delay_callback();
    uint64_t vehicle_state;
    float yaw_earth;
    //TODO: Remove dependencies on initialisation
    // of following classes in GCS
    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&HerePro_FW::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&HerePro_FW::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&HerePro_FW::exit_mission_callback, void)};
    
    static HerePro_FW* _singleton;

    uint32_t last_vehicle_state;

    uint32_t send_next_node_id_allocation_request_at_ms[NUM_CAN_IFACES]; ///< When the next node ID allocation request should be sent
    uint8_t node_id_allocation_unique_id_offset[NUM_CAN_IFACES];         ///< Depends on the stage of the next request

public:
    HerePro_FW();

    /* Do not allow copies */
    HerePro_FW(const HerePro_FW &other) = delete;
    HerePro_FW &operator=(const HerePro_FW&) = delete;

    static HerePro_FW *get_singleton() { return _singleton; }
    void init() override;
    void update() override;
    static void pps_irq_event();
    void can_imu_update();
    void can_voltage_update(uint32_t index, float value);
    void can_gps_update() override;
    void can_mag_update() override;
    void handle_RTCMStreamSend();
    void handle_lightscommand(CanardInstance* isns, CanardRxTransfer* transfer) override;
    void handle_herepro_notify(CanardInstance* isns, CanardRxTransfer* transfer) override;

    // Methods overriden for multi interface support
    void can_start() override;
    void processTx(void) override;
    void processRx(void) override;
    int16_t canard_broadcast(uint64_t data_type_signature,
                                        uint16_t data_type_id,
                                        uint8_t priority,
                                        const void* payload,
                                        uint16_t payload_len) override;
    uint16_t pool_peak_percent(void) override;
    void cleanup_stale_trx(uint64_t &timestamp_usec) override;
    bool can_do_dna(uint8_t iface);
    void handle_allocation_response(CanardInstance* isns, CanardRxTransfer* transfer) override;

    // Handled under LUA script to control LEDs
    float get_yaw_earth() { return yaw_earth; }
    uint32_t get_vehicle_state() { return vehicle_state; }
    // setup the var_info table
    AP_Param param_loader{var_info};
    static const AP_Param::Info var_info[];
};

extern HerePro_FW periph;
#else
class HerePro_FW {
public:
    HerePro_FW() {}
    static HerePro_FW *get_singleton() { return nullptr; }
    float get_yaw_earth() { return 0; }
    uint32_t get_vehicle_state() { return 0; }
};

#endif