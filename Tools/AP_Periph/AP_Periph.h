#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include "../AP_Bootloader/app_comms.h"
#include "hwing_esc.h"
#include "canard.h"
#include <uavcan/protocol/NodeStatus.h>

#if defined(HAL_PERIPH_NEOPIXEL_COUNT) || defined(HAL_PERIPH_ENABLE_NCP5623_LED)
#define AP_PERIPH_HAVE_LED
#endif

#include "Parameters.h"
#include "ch.h"

/*
  app descriptor compatible with MissionPlanner
 */
extern const struct app_descriptor app_descriptor;

class AP_Periph_FW {
private:
    uint32_t send_next_node_id_allocation_request_at_ms; ///< When the next node ID allocation request should be sent
    uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request

public:
    virtual void init();
    virtual void update();
    Parameters g;

    uavcan_protocol_NodeStatus node_status;

    static void fix_float16(float &f);
    virtual uint16_t pool_peak_percent();
    static void can_printf(const char *fmt, ...);
    static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer);
    static uint16_t get_random_range(uint16_t range);
    static void readUniqueID(uint8_t* out_uid);

    virtual void processTx();
    virtual void processRx();
    virtual void cleanup_stale_trx(uint64_t &timestamp_usec);
    void process1HzTasks(uint64_t timestamp_usec);
    void can_wait_node_id(void);
    virtual int16_t canard_broadcast(uint64_t data_type_signature,
                        uint16_t data_type_id,
                        uint8_t priority,
                        const void* payload,
                        uint16_t payload_len);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id);
    virtual void handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_get_node_info(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_param_getset(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_param_executeopcode(CanardInstance* ins, CanardRxTransfer* transfer);
    virtual void handle_herepro_notify(CanardInstance* isns, CanardRxTransfer* transfer) {}
#ifdef HAL_PERIPH_ENABLE_BUZZER
    void handle_beep_command(CanardInstance* ins, CanardRxTransfer* transfer);
#endif
#ifdef HAL_GPIO_PIN_SAFE_LED
    void handle_safety_state(CanardInstance* ins, CanardRxTransfer* transfer);
#endif
#ifdef HAL_PERIPH_ENABLE_GPS
    void handle_RTCMStream(CanardInstance* ins, CanardRxTransfer* transfer);
#endif
#ifdef AP_PERIPH_HAVE_LED
    virtual void handle_lightscommand(CanardInstance* ins, CanardRxTransfer* transfer);
#endif

    virtual void can_start();
    virtual void can_update();
    virtual void can_mag_update();
    virtual void can_gps_update();
    virtual void can_baro_update();
    virtual void can_airspeed_update();
    virtual void can_rangefinder_update();
#ifdef HAL_GPIO_PIN_SAFE_BUTTON
    virtual void can_safety_button_update();
#endif
    virtual void load_parameters();

    AP_SerialManager serial_manager;

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_GPS gps;
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    Compass compass;
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Baro baro;
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    struct {
        AP_MSP msp;
        MSP::msp_port_t port;
        uint32_t last_gps_ms;
        uint32_t last_baro_ms;
        uint32_t last_mag_ms;
    } msp;
    void msp_init(AP_HAL::UARTDriver *_uart);
    void msp_sensor_update(void);
    void send_msp_packet(uint16_t cmd, void *p, uint16_t size);
    void send_msp_GPS(void);
    void send_msp_compass(void);
    void send_msp_baro(void);
#endif
    
#ifdef HAL_PERIPH_ENABLE_ADSB
    virtual void adsb_init();
    virtual void adsb_update();
    virtual void can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg);
    struct {
        mavlink_message_t msg;
        mavlink_status_t status;
    } adsb;
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    AP_Airspeed airspeed;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    RangeFinder rangefinder;
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    virtual void pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);
    virtual void pwm_hardpoint_init();
    virtual void pwm_hardpoint_update();
    struct {
        uint8_t last_state;
        uint32_t last_ts_us;
        uint32_t last_send_ms;
        uint16_t pwm_value;
        uint16_t highest_pwm;
    } pwm_hardpoint;
#endif

#ifdef HAL_PERIPH_ENABLE_HWESC
    HWESC_Telem hwesc_telem;
    virtual void hwesc_telem_update();
#endif
    
#if !defined(HAL_BOARD_AP_PERIPH_HEREPRO)
    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];
#endif

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
    uint32_t last_airspeed_update_ms;
};

#if !defined(HAL_BOARD_AP_PERIPH_HEREPRO)
extern AP_Periph_FW periph;
#endif

extern "C" {
void can_printf(const char *fmt, ...);
}

