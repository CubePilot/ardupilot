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
/*
  HerePro main firmware

  To flash this firmware on Linux use:

     st-flash write build/f103-periph/bin/AP_Periph.bin 0x8006000

 */
#include <AP_HAL/AP_HAL.h>
#include "hal.h"
#include <stdio.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include "HerePro.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <canard.h>
#include <uavcan/equipment/ahrs/RawIMU.h>
#include <uavcan/equipment/power/CircuitStatus.h>
#include <uavcan/equipment/ahrs/Solution.h>
#include <uavcan/equipment/indication/LightsCommand.h>
#include <com/hex/equipment/herepro/NotifyState.h>
#include <uavcan/equipment/gnss/RTCMStream.h>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.h>
#include <uavcan/protocol/dynamic_node_id/Allocation.h>
#include <uavcan/equipment/gnss/Fix2.h>
#include <uavcan/equipment/gnss/Auxiliary.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CANManager/AP_SLCANIface.h>

#ifndef CAN_PROBE_CONTINUOUS
#define CAN_PROBE_CONTINUOUS 0
#endif
#ifndef HAL_CAN_POOL_SIZE
#define HAL_CAN_POOL_SIZE 4000
#endif
extern const AP_HAL::HAL &hal;

AP_Vehicle& vehicle = *AP_Vehicle::get_singleton();
HerePro_FW* HerePro_FW::_singleton = nullptr;

static ChibiOS::CANIface hw_can1_iface(0);
static ChibiOS::CANIface hw_can2_iface(1);
static SLCAN::CANIface slcan_iface;

#define NUM_CAN_IFACES 3

static AP_HAL::CANIface *can_iface[] = {&hw_can1_iface, &hw_can2_iface, &slcan_iface};

static CanardInstance canard[NUM_CAN_IFACES];

static uint32_t canard_memory_pool[NUM_CAN_IFACES][HAL_CAN_POOL_SIZE/sizeof(uint32_t)];
#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID CANARD_BROADCAST_NODE_ID
#endif
uint8_t PreferredNodeID[NUM_CAN_IFACES] = {HAL_CAN_DEFAULT_NODE_ID,
#if NUM_CAN_IFACES > 1
HAL_CAN_DEFAULT_NODE_ID,
#elif NUM_CAN_IFACES > 2
HAL_CAN_DEFAULT_NODE_ID,
#endif
};

uint8_t transfer_id[NUM_CAN_IFACES];


HerePro_FW::HerePro_FW() :
    logger(g.log_bitmask)
{ 
    if (_singleton != nullptr) {
        AP_HAL::panic("HerePro_FW must be singleton");
    }
    _singleton = this;
}

void HerePro_FW::init()
{
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
    stm32_watchdog_init();

    stm32_watchdog_pat();

    load_parameters();

    stm32_watchdog_pat();

    AP::rtc().set_utc_usec(hal.util->get_hw_rtc(), AP_RTC::SOURCE_HW);

    // initialise serial manager as early as sensible to get
    // diagnostic output during boot process.  We have to initialise
    // the GCS singleton first as it sets the global mavlink system ID
    // which may get used very early on.
    gcs().init();

    // initialise serial ports
    serial_manager.init();
    gcs().setup_console();

    // Register delay callback
    hal.scheduler->register_delay_callback(scheduler_delay_callback, 1);

    can_start();

    stm32_watchdog_pat();

    printf("Booting %08x:%08x %u/%u len=%u 0x%08x\n",
           app_descriptor.image_crc1,
           app_descriptor.image_crc2,
           app_descriptor.version_major, app_descriptor.version_minor,
           app_descriptor.image_size,
           app_descriptor.git_hash);

    if (hal.util->was_watchdog_reset()) {
        printf("Reboot after watchdog reset\n");
    }


	while(g.serialpass > 0)
	{
        static uint32_t currentbaud = 0;
        stm32_watchdog_pat();

        // follow the usb baudrate request - allows firmware update etc
        uint32_t baud = hal.util->get_usb_baud(2); //*((uint32_t*)linecoding2.dwDTERate);
        if(currentbaud != baud) {
            can_printf("new baud %lu", baud);
            hal.uartB->end();
            hal.uartB->begin(baud);
            currentbaud = baud;
        }

	    // send characters received from the otg2 to the GPS
	    while (hal.uartC->available()) {
	        hal.uartB->write(hal.uartC->read());
	    }
	    // send GPS characters to the otg2
	    while (hal.uartB->available()) {
	        hal.uartC->write(hal.uartB->read());
	    }
	    can_update();
        // 1ms = 56kb at 460800 baud
        hal.scheduler->delay(1);
	}    

    if (gps.get_type(0) != AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        gps.init(serial_manager);
    }

    if (compass.enabled()) {
        compass.init();
    }

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();

    // power leds
    palWriteLine(HAL_GPIO_PIN_LED, 1); // 4 leds left/right
    palWriteLine(HAL_GPIO_PIN_LED_2, 1); // the rest

    // setup handler for rising edge of IRQ pin - GPIO(100)
    hal.gpio->pinMode(100, HAL_GPIO_INPUT);
    hal.gpio->attach_interrupt(100, pps_irq_event, AP_HAL::GPIO::INTERRUPT_RISING);

    {
        // get analog for loop
        _adc0 = hal.analogin->channel(4);
        _adc1 = hal.analogin->channel(13);
        _adc2 = hal.analogin->channel(14);
        _adc3 = hal.analogin->channel(17);
    }

    // Initialise logging
    log_init();

    notify.init();
    scripting.init();
}

/*
the pps pin outputs a high accuracy time pulse every 1 second. using this we can calculate the position processing delay/serial buffer delay
*/
void HerePro_FW::pps_irq_event()
{
    static uint64_t last_irq;
    static uint8_t setcounter;
    uint64_t tnow = AP_HAL::micros64();

    uint64_t tepoch_us = AP::gps().time_epoch_usec();
    setcounter++;
    uint64_t epoch_ms =  tepoch_us % 1000000;
    tepoch_us = tepoch_us - epoch_ms;
    if(epoch_ms > 500000)
         tepoch_us = tepoch_us + 1000000;

    if(tepoch_us > 0 && setcounter > 10) {
        //can_printf("PCI TimePulse set %llu", tepoch_us);
        AP::rtc().set_utc_usec(tepoch_us, AP_RTC::SOURCE_GPS);
    }

    uint64_t utc_usec=0;
    if (AP::rtc().get_utc_usec(utc_usec)) {
        can_printf("PCI TimePulse %f ms %llu gps %llu set %llu", (tnow - last_irq) * 0.001, utc_usec, AP::gps().time_epoch_usec(), tepoch_us);
    }
    last_irq = tnow;
}


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void HerePro_FW::scheduler_delay_callback()
{
    static uint32_t last_1hz, last_50hz;

    AP_Logger &logger = AP::logger();

    // don't allow potentially expensive logging calls:
    logger.EnableWrites(false);

    const uint32_t tnow = AP_HAL::millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_SYS_STATUS);
    }

    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs().update_receive();
        gcs().update_send();
    }

    logger.EnableWrites(true);
}

void HerePro_FW::update()
{
    static uint32_t last_1hz;
    static uint32_t last_50hz;
    static uint32_t last_100hz;
    uint32_t tnow = AP_HAL::millis();

     if (g.testmode > 0 && tnow - last_1hz >= 1000)
     {
         last_1hz = tnow;

         hal.gpio->pinMode(1, HAL_GPIO_INPUT);
         uint8_t gpio1 = hal.gpio->read(1);
         hal.gpio->pinMode(2, HAL_GPIO_INPUT);
         uint8_t gpio2 = hal.gpio->read(2);
         //can_printf("gpio %u %u\r\n",gpio1,gpio2);

         float adc1 = _adc0->voltage_average();
         float adc2 = _adc1->voltage_average();
         float adc3 = _adc2->voltage_average();
         float adc4 = _adc3->voltage_average();

         //can_printf("analog vcc(5v) %f vcc2(1.8v) %f bat1 %f bat2 %f\r\n",adc1,adc2,adc3,adc4);

         can_voltage_update(0,adc3);
         can_voltage_update(1,adc4);
         can_voltage_update(2,adc1);
         can_voltage_update(3,adc2);
         can_voltage_update(4,gpio1);
         can_voltage_update(5,gpio2);
         can_imu_update();
     }

    if (AP_HAL::millis() - last_vehicle_state > 500 ) {
        // We haven't heard from Ardupilot for a while go down
        vehicle_state = 1;
        yaw_earth = 0;
    }

    if (tnow - last_50hz >= 20) {
        last_50hz = tnow;
        notify.update();          
    }
    
    // testmode raw rtcm feed or we are a moving base and need to forward rtcm messages to can
    if (g.testmode >= 2 || gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_UBLOX_RTK_BASE){
        handle_RTCMStreamSend();
    }

    if (tnow - last_100hz >= 10) {
        last_100hz = tnow;  

        // update INS immediately to get current gyro data populated
        ins.update();

        // run EKF state estimator (expensive)
        // --------------------
        // we tell AHRS to skip INS update as we have already done it in fast_loop()
        ahrs.update(true);

        // Inertial Nav
        // --------------------
        //inertial_nav.update(vibration_check.high_vibes);      
    }
    bool dna_finished = false;
    for (uint8_t i = 0; i < NUM_CAN_IFACES; i++) {
        if (can_do_dna(i)) {
            dna_finished = true;
        } else {
            processTx();
            processRx();
        }
    }
    if (dna_finished) {
        can_update();
    }
    hal.scheduler->delay(2);
}


void HerePro_FW::handle_RTCMStreamSend()
{
    const uint8_t *rtcm_data;
    uint16_t rtcm_len;
    
    if (periph.gps.get_RTCMV3(0, rtcm_data, rtcm_len)) {   
        for (int a=0; a < rtcm_len; a+=128) {
            uint8_t size = MIN(rtcm_len - a, 128);

            uavcan_equipment_gnss_RTCMStream pkt {};
            pkt.protocol_id = 3;
            pkt.data.len = size;
            pkt.data.data = (uint8_t*)&rtcm_data[a];

            uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_DATA_MAX_LENGTH];
            uint16_t total_size = uavcan_equipment_gnss_RTCMStream_encode(&pkt, buffer);

            canard_broadcast(UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_SIGNATURE,
                            UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,                                
                            &buffer[0],
                            total_size);
        }

        periph.gps.clear_RTCMV3(0);
    }
}

void HerePro_FW::can_voltage_update(uint32_t index, float value)
{
    uint8_t buffer[UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_MAX_SIZE];

    uavcan_equipment_power_CircuitStatus power1;

    power1.circuit_id = index;
    power1.voltage = value;         

    uint32_t len = uavcan_equipment_power_CircuitStatus_encode(&power1, buffer);

    canard_broadcast(UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

void HerePro_FW::can_mag_update(void)
{
#ifdef HAL_PERIPH_ENABLE_MAG
    if (!compass.enabled()) {
        return;
    }
    compass.read();
#if CAN_PROBE_CONTINUOUS
    if (compass.get_count() == 0) {
        static uint32_t last_probe_ms;
        uint32_t now = AP_HAL::millis();
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            compass.init();
        }
    }
#endif

    if (last_mag_update_ms == compass.last_update_ms()) {
        return;
    }

    if (g.testmode > 0 && AP_HAL::millis() - last_mag_update_ms < 1000) {
        return;
    }

    last_mag_update_ms = compass.last_update_ms();
    const Vector3f &field = compass.get_field();
    uavcan_equipment_ahrs_MagneticFieldStrength pkt {};

    // the canard dsdl compiler doesn't understand float16
    for (uint8_t i=0; i<3; i++) {
        pkt.magnetic_field_ga[i] = field[i] * 0.001;
        fix_float16(pkt.magnetic_field_ga[i]);
    }

    uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_ahrs_MagneticFieldStrength_encode(&pkt, buffer);

    canard_broadcast(UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE,
                    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_MAG
}

void HerePro_FW::can_imu_update(void)
{
    {
        static uint64_t last_imu_time_us = 0;

        uint64_t time_us = AP_HAL::micros64();

        const Vector3f &gyro = ins.get_gyro(0);
        const Vector3f &accel = ins.get_accel(0);
        float int_int = (float)(time_us - last_imu_time_us) / 1000.0;
        uavcan_equipment_ahrs_RawIMU pkt { };
        pkt.integration_interval= int_int;

        last_imu_time_us = time_us;
        pkt.timestamp.usec = time_us;

            // the canard dsdl compiler doesn't understand float16
        for (uint8_t i=0; i<3; i++) {
            pkt.accelerometer_latest[i] = accel[i] ;
            fix_float16(pkt.accelerometer_latest[i]);
        }

            // the canard dsdl compiler doesn't understand float16
        for (uint8_t i=0; i<3; i++) {
            pkt.rate_gyro_latest[i] = gyro[i] ;
            fix_float16(pkt.rate_gyro_latest[i]);
        }

        uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_RAWIMU_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_ahrs_RawIMU_encode(&pkt, buffer);

        canard_broadcast(UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE,
                        UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    {
        uavcan_equipment_ahrs_Solution _att_state {};
        uavcan_Timestamp ts {};
        ts.usec = AP_HAL::micros64();
        _att_state.timestamp = ts;

        Quaternion qt;
        Matrix3f rot = ahrs.get_rotation_body_to_ned();
        qt.from_rotation_matrix(rot);
        _att_state.orientation_xyzw[0] = qt.q1;
        _att_state.orientation_xyzw[1] = qt.q2;
        _att_state.orientation_xyzw[2] = qt.q3;
        _att_state.orientation_xyzw[3] = qt.q4;
        fix_float16(_att_state.orientation_xyzw[0]);
        fix_float16(_att_state.orientation_xyzw[1]);
        fix_float16(_att_state.orientation_xyzw[2]);
        fix_float16(_att_state.orientation_xyzw[3]);
        // TODO: extract from EKF
        //_att_state.orientation_covariance

        Vector3f av = ahrs.get_gyro();
        _att_state.angular_velocity[0] = av.x;
        _att_state.angular_velocity[1] = av.y;
        _att_state.angular_velocity[2] = av.z;
        fix_float16(_att_state.angular_velocity[0]);
        fix_float16(_att_state.angular_velocity[1]);
        fix_float16(_att_state.angular_velocity[2]);
        
        // TODO: extract from EKF
        //_att_state.angular_velocity_covariance

        Vector3f la = ahrs.get_accel_ef();
        _att_state.linear_acceleration[0] = la.x;
        _att_state.linear_acceleration[1] = la.y;
        _att_state.linear_acceleration[2] = la.z;
        fix_float16(_att_state.linear_acceleration[0]);
        fix_float16(_att_state.linear_acceleration[1]);
        fix_float16(_att_state.linear_acceleration[2]);

        // TODO: extract from EKF
        //_att_state.linear_acceleration_covariance

        uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_ahrs_Solution_encode(&_att_state, buffer);

        canard_broadcast(UAVCAN_EQUIPMENT_AHRS_SOLUTION_SIGNATURE,
                        UAVCAN_EQUIPMENT_AHRS_SOLUTION_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

/*
  update CAN GPS
 */
void HerePro_FW::can_gps_update(void)
{
#ifdef HAL_PERIPH_ENABLE_GPS
    if (gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        return;
    }
    gps.update();
    if (last_gps_update_ms == gps.last_message_time_ms()) {
        return;
    }
    if (g.testmode > 0 && AP_HAL::millis() - last_gps_update_ms < 1000) {
        return;
    }
    last_gps_update_ms = gps.last_message_time_ms();

    {
        /*
          send Fix2 packet
        */
        uavcan_equipment_gnss_Fix2 pkt {};
        const Location &loc = gps.location();
        const Vector3f &vel = gps.velocity();

        pkt.timestamp.usec = AP_HAL::micros64();
        pkt.gnss_timestamp.usec = gps.time_epoch_usec();
        if (pkt.gnss_timestamp.usec == 0) {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_NONE;
        } else {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC;
        }
        pkt.longitude_deg_1e8 = uint64_t(loc.lng) * 10ULL;
        pkt.latitude_deg_1e8 = uint64_t(loc.lat) * 10ULL;
        pkt.height_ellipsoid_mm = loc.alt * 10;
        pkt.height_msl_mm = loc.alt * 10;
        for (uint8_t i=0; i<3; i++) {
            pkt.ned_velocity[i] = vel[i];
        }
        pkt.sats_used = gps.num_sats();
        switch (gps.status()) {
        case AP_GPS::GPS_Status::NO_GPS:
        case AP_GPS::GPS_Status::NO_FIX:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_SBAS;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
            break;
        }

        float cov[6] {};
        pkt.covariance.data = &cov[0];
        pkt.covariance.len = 6;

        float hacc;
        if (gps.horizontal_accuracy(hacc)) {
            cov[0] = cov[1] = sq(hacc);
        }
    
        float vacc;
        if (gps.vertical_accuracy(vacc)) {
            cov[2] = sq(vacc);
        }

        float sacc;
        if (gps.speed_accuracy(sacc)) {
            float vc3 = sq(sacc);
            cov[3] = cov[4] = cov[5] = vc3;
        }

        for (uint8_t i=0; i<6; i++) {
            fix_float16(cov[i]);
        }

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_gnss_Fix2_encode(&pkt, buffer);

        canard_broadcast(UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
    
    /*
      send aux packet
     */
    {
        uavcan_equipment_gnss_Auxiliary aux {};
        aux.hdop = gps.get_hdop() * 0.01;
        aux.vdop = gps.get_vdop() * 0.01;
        fix_float16(aux.hdop);
        fix_float16(aux.vdop);

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_AUXILIARY_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_gnss_Auxiliary_encode(&aux, buffer);
        canard_broadcast(UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
#endif // HAL_PERIPH_ENABLE_GPS
}

const struct LogStructure HerePro_FW::log_structure[] = {
    LOG_COMMON_STRUCTURES,
};

void HerePro_FW::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

void HerePro_FW::handle_lightscommand(CanardInstance* isns, CanardRxTransfer* transfer)
{
    uavcan_equipment_indication_LightsCommand req;
    uint8_t arraybuf[UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_MAX_SIZE];
    uint8_t *arraybuf_ptr = arraybuf;
    if (uavcan_equipment_indication_LightsCommand_decode(transfer, transfer->payload_len, &req, &arraybuf_ptr) < 0) {
        return;
    }
    for (uint8_t i=0; i<req.commands.len; i++) {
        uavcan_equipment_indication_SingleLightCommand &cmd = req.commands.data[i];
        // to get the right color proportions we scale the green so that is uses the
        // same number of bits as red and blue
        uint8_t red = cmd.color.red<<3;
        uint8_t green = (cmd.color.green>>1)<<3;
        uint8_t blue = cmd.color.blue<<3;
        if (periph.g.led_brightness != 100 && periph.g.led_brightness >= 0) {
            float scale = periph.g.led_brightness * 0.01;
            red = constrain_int16(red * scale, 0, 255);
            green = constrain_int16(green * scale, 0, 255);
            blue = constrain_int16(blue * scale, 0, 255);
        }
        notify.handle_rgb_id(red, green, blue, cmd.light_id);
    }
}

void HerePro_FW::handle_herepro_notify(CanardInstance* isns, CanardRxTransfer* transfer)
{
    com_hex_equipment_herepro_NotifyState msg;
    uint8_t arraybuf[COM_HEX_EQUIPMENT_HEREPRO_NOTIFYSTATE_MAX_SIZE];
    uint8_t *arraybuf_ptr = arraybuf;
    if (com_hex_equipment_herepro_NotifyState_decode(transfer, transfer->payload_len, &msg, &arraybuf_ptr) < 0) {
        return;
    }
    vehicle_state = msg.vehicle_state;
    yaw_earth = msg.yaw_earth;
    last_vehicle_state = AP_HAL::millis();
}


void HerePro_FW::can_start()
{
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    // initialise Can ifaces
    hw_can1_iface.init(1000000, AP_HAL::CANIface::NormalMode);
    hw_can2_iface.init(1000000, AP_HAL::CANIface::NormalMode);
    hal.uartA->begin(115200);
    slcan_iface.set_port(hal.uartA);

    for (uint8_t i = 0; i < NUM_CAN_IFACES; i++) {
        canardInit(&canard[i], (uint8_t *)canard_memory_pool[i], sizeof(canard_memory_pool[i]),
                onTransferReceived, shouldAcceptTransfer, NULL);

        if (PreferredNodeID[i] != CANARD_BROADCAST_NODE_ID) {
            canardSetLocalNodeID(&canard[i], PreferredNodeID[i]);
        }
    }
}

void HerePro_FW::processTx(void)
{
    static uint8_t fail_count;
    for (uint8_t i = 0; i < NUM_CAN_IFACES; i++) {
        if (can_iface[i] == NULL) {
            continue;
        }
        for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard[i])) != NULL;) {
            AP_HAL::CANFrame txmsg {};
            txmsg.dlc = txf->data_len;
            memcpy(txmsg.data, txf->data, 8);
            txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
            // push message with 1s timeout
            if (can_iface[i]->send(txmsg, AP_HAL::micros64() + 1000000, 0) > 0) {
                canardPopTxQueue(&canard[i]);
                fail_count = 0;
            } else {
                // just exit and try again later. If we fail 8 times in a row
                // then start discarding to prevent the pool filling up
                if (fail_count < 8) {
                    fail_count++;
                } else {
                    canardPopTxQueue(&canard[i]);
                }
                return;
            }
        }
    }
}

void HerePro_FW::processRx(void)
{
    AP_HAL::CANFrame rxmsg;
    for(uint8_t iface = 0; iface < NUM_CAN_IFACES; iface++) {
        if (can_iface[iface] == NULL) {
            continue;
        }
        while (true) {
            bool read_select = true;
            bool write_select = false;
            can_iface[iface]->select(read_select, write_select, nullptr, 0);
            if (!read_select) {
                break;
            }
            CanardCANFrame rx_frame {};

            //palToggleLine(HAL_GPIO_PIN_LED);
            uint64_t timestamp;
            AP_HAL::CANIface::CanIOFlags flags;
            can_iface[iface]->receive(rxmsg, timestamp, flags);
            memcpy(rx_frame.data, rxmsg.data, 8);
            rx_frame.data_len = rxmsg.dlc;
            rx_frame.id = rxmsg.id;
            canardHandleRxFrame(&canard[iface], &rx_frame, timestamp);
        }
    }
}

int16_t HerePro_FW::canard_broadcast(uint64_t data_type_signature,
                                    uint16_t data_type_id,
                                    uint8_t priority,
                                    const void* payload,
                                    uint16_t payload_len)
{
    int16_t cache = 0;
    bool success = false;
    for (uint8_t i = 0; i < NUM_CAN_IFACES; i++) {
        if (canardGetLocalNodeID(&canard[i]) == CANARD_BROADCAST_NODE_ID) {
            continue;
        }
        cache = canardBroadcast(&canard[i],
                        data_type_signature,
                        data_type_id,
                        &transfer_id[i],
                        priority,
                        payload,
                        payload_len);
        if (cache == 0) {
            // we had atleast one interface where transaction was successful
            success = true;
        }
    }
    return success?0:cache;
}

uint16_t HerePro_FW::pool_peak_percent(void)
{
    uint16_t peak_percent = 0;
    for (uint8_t i = 0; i < NUM_CAN_IFACES; i++) {
        const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard[i]);
        peak_percent = MAX(peak_percent, (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks));
    }
    return peak_percent;
}

void HerePro_FW::cleanup_stale_trx(uint64_t &timestamp_usec)
{
    for (uint8_t i = 0; i < NUM_CAN_IFACES; i++) {
        canardCleanupStaleTransfers(&canard[i], timestamp_usec);
    }
}


/*
  wait for dynamic allocation of node ID
 */
bool HerePro_FW::can_do_dna(uint8_t iface)
{
    if (canardGetLocalNodeID(&canard[iface]) != CANARD_BROADCAST_NODE_ID) {
        return true;
    }
    uint8_t node_id_allocation_transfer_id = 0;

    // printf("Waiting for dynamic node ID allocation... (pool %u)\n", pool_peak_percent());

    stm32_watchdog_pat();
    uint32_t now = AP_HAL::millis();

    if (send_next_node_id_allocation_request_at_ms[iface] > now) {
        return false;
    }

    send_next_node_id_allocation_request_at_ms[iface] =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PreferredNodeID[iface] << 1U);

    if (node_id_allocation_unique_id_offset[iface] == 0)
    {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    readUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH - node_id_allocation_unique_id_offset[iface]);
    if (uid_size > MaxLenOfUniqueIDInRequest)
    {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    // Paranoia time
    assert(node_id_allocation_unique_id_offset[iface] < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
    assert(uid_size <= MaxLenOfUniqueIDInRequest);
    assert(uid_size > 0);
    assert((uid_size + node_id_allocation_unique_id_offset[iface]) <= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);

    memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset[iface]], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = canardBroadcast(&canard[iface],
                                                UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                                UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                                &node_id_allocation_transfer_id,
                                                CANARD_TRANSFER_PRIORITY_LOW,
                                                &allocation_request[0],
                                                (uint16_t) (uid_size + 1));
    if (bcast_res < 0)
    {
        printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    node_id_allocation_unique_id_offset[iface] = 0;

    printf("Dynamic node ID allocation complete [%d]\n", canardGetLocalNodeID(&canard[iface]));
    return false;
}


void HerePro_FW::handle_allocation_response(CanardInstance* isns, CanardRxTransfer* transfer)
{
    uint8_t iface = 0;
    for (uint8_t i = 0; i < NUM_CAN_IFACES; i++) {
        if (isns == &canard[i]) {
            iface = i;
        }
    }
    // Rule C - updating the randomized time interval
    send_next_node_id_allocation_request_at_ms[iface] =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
        node_id_allocation_unique_id_offset[iface] = 0;
        return;
    }

    // Copying the unique ID from the message
    static const uint8_t UniqueIDBitOffset = 8;
    uint8_t received_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    uint8_t received_unique_id_len = 0;
    for (; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U)); received_unique_id_len++) {
        assert(received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
        const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
        (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(received_unique_id, my_unique_id, received_unique_id_len) != 0) {
        printf("Mismatching allocation response\n");
        node_id_allocation_unique_id_offset[iface] = 0;
        return;         // No match, return
    }

    if (received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation_unique_id_offset[iface] = received_unique_id_len;
        send_next_node_id_allocation_request_at_ms[iface] -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", received_unique_id_len);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        uint8_t allocated_node_id = 0;
        (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
        assert(allocated_node_id <= 127);

        canardSetLocalNodeID(isns, allocated_node_id);
        printf("Node ID allocated: %d\n", allocated_node_id);
    }
}
