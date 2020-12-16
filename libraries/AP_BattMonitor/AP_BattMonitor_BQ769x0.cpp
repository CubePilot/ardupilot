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

   Reference: https://github.com/LibreSolar/bq769x0_mbed_lib/blob/master/bq769x0.cpp
   https://www.ti.com/lit/ds/symlink/bq76920.pdf
 */

#include "AP_BattMonitor_BQ769x0.h"
#include <GCS_MAVLink/GCS.h>

// Register addresses
#define BQ769x0_SYS_STAT    0x00
#define BQ769x0_CELLBAL1    0x01
#define BQ769x0_CELLBAL2    0x02
#define BQ769x0_CELLBAL3    0x03
#define BQ769x0_SYS_CTRL1   0x04
#define BQ769x0_SYS_CTRL2   0x05
#define BQ769x0_PROTECT1    0x06
#define BQ769x0_PROTECT2    0x07
#define BQ769x0_PROTECT3    0x8
#define BQ769x0_OV_TRIP     0x09
#define BQ769x0_UV_TRIP     0x0A
#define BQ769x0_CC_CFG      0x0B

//Read-only
#define BQ769x0_VC1_HI  0x0C
#define BQ769x0_VC1_LO  0x0D
//Other VC registers are done with an offset in software
#define BQ769x0_BAT_HI  0x2A
#define BQ769x0_BAT_LO  0x2B
#define BQ769x0_TS1_HI  0x2C
#define BQ769x0_TS1_LO  0x2D
//Other TS registers are done with an offset in software
#define BQ769x0_CC_HI  0x32
#define BQ769x0_CC_LO  0x33
#define BQ769x0_ADCGAIN1  0x50
#define BQ769x0_ADCOFFSET  0x51
#define BQ769x0_ADCGAIN2  0x59

//SYS_STAT bit masks
#define BQ769x0_CC_READY  (1<<7)
#define BQ769x0_DEVICE_XREADY (1<<5)
#define BQ769x0_OVRD_ALERT (1<<4)
#define BQ769x0_UV (1<<3)
#define BQ769x0_OV (1<<2)
#define BQ769x0_SCD (1<<1)
#define BQ769x0_OCD (1<<0)

//SYS_CTRL1 bit masks
#define BQ769x0_LOAD_PRESENT (1<<7)
#define BQ769x0_ADC_EN (1<<4)
#define BQ769x0_TEMP_SEL (1<<3)
#define BQ769x0_SHUT_A (1<<1)
#define BQ769x0_SHUT_B (1<<0) 

//SYS_CTRL2 bit masks
#define BQ769x0_DELAY_DIS (1<<7)
#define BQ769x0_CC_EN (1<<6)
#define BQ769x0_CC_ONESHOT (1<<5)
#define BQ769x0_DSG_ON (1<<1)
#define BQ769x0_CHG_ON (1<<0)

#define AP_BATTMONITOR_BQ769x0_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

#define IDLE_CURRENT_THRESHOLD 0.10f // 100mA

#define CELL_BAL_START_IDLE_MS 60000 // 1min
#define CELL_BAL_START_VOLTAGE 4000 // 4V
#define CELL_BAL_STOP_VDIFF 4 // 4mv

#ifdef HAL_BUILD_AP_PERIPH
#include "../../Tools/AP_Periph/AP_Periph.h"
#define Debug(fmt, args ...) do { can_printf(fmt, ## args); } while (0)
#endif

struct thermistor_lookup_t {
    float deg;
    float resistance;
};

// Lookup table for NXFT15XV103 https://www.murata.com/~/media/webrenewal/support/library/catalog/products/thermistor/ntc/r44e.ashx
static const thermistor_lookup_t thermistor_lookup_table[] =
{{-40.f,337.503f}, {-35.f,243.332f}, {-30.f,177.496f}, {-25.f,130.859f}, {-20.f,97.428f}, {-15.f,73.230f}, {-10.f,55.529f}, {-5.f,42.467f}, {0,32.747f}, {5.f,25.450f}, {10.f,19.932f}, {15.f,15.727f}, {20.f,12.498f}, {25.f,10.000f}, {30.f,8.054f}, {35.f,6.529f}, {40.f,5.324f}, {45.f,4.366f}, {50.f,3.601f}, {55.f,2.985f}, {60.f,2.488f}, {65.f,2.083f}, {70.f,1.752f}, {75.f,1.480f}, {80.f,1.256f}, {85.f,1.070f}, {90.f,0.916f}, {95.f,0.787f}, {100.f,0.679f}, {105.f,0.588f}, {110.f,0.512f}, {115.f,0.446f}, {120.f,0.391f}, {125.f,0.343f}};

extern const AP_HAL::HAL& hal;

AP_BattMonitor_BQ769x0::AP_BattMonitor_BQ769x0(AP_BattMonitor &mon,
                                           AP_BattMonitor::BattMonitor_State &mon_state,
                                           AP_BattMonitor_Params &params,
                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
        : AP_BattMonitor_Backend(mon, mon_state, params),
        _dev(std::move(dev))
{}

void AP_BattMonitor_BQ769x0::init(void)
{
    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();

    if (!bus_sem) {
        hal.console->printf("BQ769x0: Unable to get bus semaphore\n");
        return;
    }
    
    WITH_SEMAPHORE(bus_sem);

    // Begin Initialisation
    //"For optimal performance, [CC_CFG] should be programmed to 0x19 upon device startup." page 40
    if (!write_byte(BQ769x0_CC_CFG, 0x19)) {
        return;
    }

    // Configure measurements
    uint8_t val = 0;
    val = BQ769x0_ADC_EN | BQ769x0_TEMP_SEL;
    write_byte(BQ769x0_SYS_CTRL1, val);

    val = BQ769x0_CC_EN;
    write_byte(BQ769x0_SYS_CTRL2, val);

    if (!read_adc_gain()) {
        hal.console->printf("BQ769x0: Failed Can't read ADC Gain\n");
        return;
    }

    if (!read_adc_offset()) {
        hal.console->printf("BQ769x0: Failed Can't read ADC Offset\n");
        return;
    }

    hal.console->printf("BQ769x0: Gain: %f uV/LSB   Offset: %d mv\n", _adc_gain, _adc_offset);

    bool stats_ok = false;
    //Read the system status register
    for (uint8_t i = 0; i < 2; i++) {
        if (!read_byte(BQ769x0_SYS_STAT, val)) {
            hal.console->printf("BQ769x0: Failed to read system status\n");
        }
        if(val & BQ769x0_DEVICE_XREADY)
        {
            hal.console->printf("BQ769x0: Device X Ready Error");
            //Try to clear it
            write_byte(BQ769x0_SYS_STAT, BQ769x0_DEVICE_XREADY);
        } else {
            stats_ok = true;
            break;
        }
        // Sleep for some time
        hal.scheduler->delay(500);
    }

    if (!stats_ok) {
        return;
    }

    //Set any other settings such as OVTrip and UVTrip limits
    float over, under;
    if (!read_OVTrip(over)) {
        hal.console->printf("BQ769x0: Failed to read Overvoltage trip\n");
        return;
    }
    if (!read_UVTrip(under)) {
        hal.console->printf("BQ769x0: Failed to read Undervoltage trip\n");
        return;
    }
    hal.console->printf("BQ769x0: Current Overvoltage trip: %f V\n", over);
    hal.console->printf("BQ769x0: Current Undervoltage trip: %f V\n", under);

    if(under != 3.32f) {
        write_UVTrip(3.32f); //Set undervoltage to 3.32V
    }

    if(over != 4.27f) {
        write_OVTrip(4.27f); //Set overvoltage to 4.27V
    }

    // Finally register a timer callback for every 100ms or 10Hz
    if (_dev) {
        timer_handle = _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_BQ769x0::timer, void));
    }
}

//Reads the gain registers and calculates the system's factory trimmed gain
//GAIN = 365uV/LSB + (ADCGAIN<4:0>) * 1uV/LSB
//ADC gain comes from two registers that have to be moved around and combined.
bool AP_BattMonitor_BQ769x0::read_adc_gain()
{
    uint8_t val1, val2;
    if (!read_byte(BQ769x0_ADCGAIN1, val1)) {
        return false;
    }
    if (!read_byte(BQ769x0_ADCGAIN2, val2)) {
        return false;
    }
    val1 &= 0b00001100; //There are some unknown reserved bits around val1 that need to be cleared

    //Recombine the bits into one ADCGAIN
    uint16_t adc_gain = (val1 << 1) | (val2 >> 5);

    _adc_gain = (365 + adc_gain)/ 1000.0f;

    return true;
}

//Returns the factory trimmed ADC offset
//Offset is -127 to 128 in mV
bool AP_BattMonitor_BQ769x0::read_adc_offset()
{
    uint8_t adc_offset;
    if (!read_byte(BQ769x0_ADCOFFSET, adc_offset)) {
        return false;
    }

    //Here we need to convert a 8bit 2's compliment to a 16 bit int
    _adc_offset = (int8_t)adc_offset; // convert value to int8 from uint8 and then hold it in int16
    return true;
}


//Returns the over voltage trip threshold
//Default is 0b.10.OVTRIP(0xAC).1000 = 0b.10.1010.1100.1000 = 0x2AC8 = 10,952
//OverVoltage = (OV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (10,952 * 0.370) + 56mV = 4.108V
bool AP_BattMonitor_BQ769x0::read_OVTrip(float& over)
{
    uint8_t val;
    if (!read_byte(BQ769x0_OV_TRIP, val)) {
        return false;
    }
    uint16_t trip = val;

    trip <<= 4; //Shuffle the bits to align to 0b.10.XXXX.XXXX.1000
    trip |= 0x2008;

    over = (((float)trip * _adc_gain) + _adc_offset)/1000.f;
    return true;
}

//Given a voltage (4.22 for example), set the over voltage trip register
//Example: voltage = 4.2V = (4200mV - 56mV) / 0.370mv = 11,200
//11,200 = 0x2BC0 = 
bool AP_BattMonitor_BQ769x0::write_OVTrip(float trip_voltage)
{
    uint8_t val = tripCalculator(trip_voltage); //Convert voltage to an 8-bit middle value
    return write_byte(BQ769x0_OV_TRIP, val); //address, value
}

//Returns the under voltage trip threshold
//Default is 0b.01.UVTRIP(0x97).0000 = 0x1970 = 6,512
//UnderVoltage = (UV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (6,512 * 0.370) + 56mV = 2.465V
bool AP_BattMonitor_BQ769x0::read_UVTrip(float& under)
{
    uint8_t val;
    if (!read_byte(BQ769x0_UV_TRIP, val)) {
        return false;
    }
    uint16_t trip = val;

    trip <<= 4; //Shuffle the bits to align to 0b.10.XXXX.XXXX.1000
    trip |= 0x2008;

    under = (((float)trip * _adc_gain) + _adc_offset)/1000.f;
    return true;
}

//Given a voltage (2.85V for example), set the under voltage trip register
bool AP_BattMonitor_BQ769x0::write_UVTrip(float trip_voltage)
{
    uint8_t val = tripCalculator(trip_voltage); //Convert voltage to an 8-bit middle value
    return write_byte(BQ769x0_UV_TRIP, val); //address, value
}

//Under voltage and over voltage use the same rules for calculating the 8-bit value
//Given a voltage this function uses gain and offset to get a 14 bit value
//Then strips that value down to the middle-ish 8-bits
//No registers are written, that's up to the caller
uint8_t AP_BattMonitor_BQ769x0::tripCalculator(float trip_voltage)
{
    trip_voltage *= 1000; //Convert volts to mV
    trip_voltage -= _adc_offset;
    trip_voltage /= _adc_gain;

    uint16_t trip_value = (uint16_t)trip_voltage; //We only want the integer - drop decimal portion.

    trip_value >>= 4; //Cut off lower 4 bits
    trip_value &= 0x00FF; //Cut off higher bits

    return trip_value;
}


//Given a cell number, return the cell voltage
//Vcell = GAIN * ADC(cell) + OFFSET
//Conversion example from datasheet: 14-bit ADC = 0x1800, Gain = 0x0F, Offset = 0x1E = 2.365V
bool AP_BattMonitor_BQ769x0::read_cell_voltage(uint8_t cell_number, uint16_t &voltage)
{
    if (cell_number > 15) {
        return false; //Return error
    }

    // Refer Table 24 https://www.ti.com/lit/ds/symlink/bq76920.pdf
    if (cell_number > 2 && _num_cells == 4) {
        cell_number++;
    }

    uint8_t registerNumber = BQ769x0_VC1_HI + (cell_number * 2);

    int16_t cell_value = 0;
    uint8_t *cell_data = (uint8_t*)&cell_value;
    if (!read_byte(registerNumber, cell_data[1])) {
        return false;
    }
    if (!read_byte(registerNumber+1, cell_data[0])) {
        return false;
    }

    if(cell_value == 0) {
        return false;
    }

    voltage = (cell_value * _adc_gain + _adc_offset); //0x1800 * 0.37 + 60 = 3,397mV

    return true;
}

//Given a thermistor number return the temperature in C
bool AP_BattMonitor_BQ769x0::read_temp(float& temp)
{
    if ((AP_HAL::millis() - _start_tempread_wait) < 2000) {
        // we need to wait until we can start reading temperature again
        return false;
    }

    uint8_t val;
    read_byte(BQ769x0_SYS_CTRL1, val);

    //See if we need to switch between internal die temp and external thermistor
    if((val & BQ769x0_TEMP_SEL) == 0) {
        //Bad news, we have to do a switch and wait 2 seconds
        //Set the TEMP_SEL bit
        val |= BQ769x0_TEMP_SEL;
        write_byte(BQ769x0_SYS_CTRL1, val); //address, value
        _start_tempread_wait = AP_HAL::millis();
    }

    int16_t therm_value;
    uint8_t *therm_bytes = (uint8_t *)&therm_value;
    if (!read_byte(BQ769x0_TS1_HI, therm_bytes[1])) {
        return false;
    }
    if (!read_byte(BQ769x0_TS1_LO, therm_bytes[0])) {
        return false;
    }

    //Therm value should now contain a 14 bit value
    float therm_voltage = therm_value * (float)382.f; //0xC89 * 382 = 1,225,838uV. 0x233C * 382uV/LSB = 3,445,640uV
    therm_voltage /= 1000000.f; //Convert to V
    
    float therm_resistance = ((float)10000.f * therm_voltage) / (3.3f - therm_voltage);

    //We now have therm voltage and resistance. With a datasheet for the NXFT15XV103 thermistor we could
    //calculate temperature. 
    temp = thermistor_lookup(therm_resistance);
    _state.temperature = temp;
    _state.temperature_time = AP_HAL::millis();
    return true;
}

//Given a resistance on a NXFT15XV103 thermistor, return a temperature in C
//From: https://www.murata.com/~/media/webrenewal/support/library/catalog/products/thermistor/ntc/r44e.ashx
float AP_BattMonitor_BQ769x0::thermistor_lookup(float resistance)
{
  //Resistance is coming in as Ohms, this lookup table assume kOhm
    resistance /= 1000.f; //Convert to kOhm
    const uint8_t lookup_table_length = ARRAY_SIZE(thermistor_lookup_table);
    if (resistance > thermistor_lookup_table[0].resistance) {
        return thermistor_lookup_table[lookup_table_length-1].deg;
    } else if(resistance < thermistor_lookup_table[lookup_table_length-1].resistance) {
        return thermistor_lookup_table[lookup_table_length-1].deg;
    }

    for (uint8_t i=1; i < lookup_table_length-1; i++) {
        if (resistance > thermistor_lookup_table[i].resistance) {
            return linear_interpolate(thermistor_lookup_table[i-1].deg, thermistor_lookup_table[i].deg, resistance, 
                                      thermistor_lookup_table[i].resistance, thermistor_lookup_table[i-1].resistance);
        }
    }

    // we should never reach here
    return 0.0f;
}


//Returns the coulomb counter value in microVolts
//Example: 84,400uV 
//Coulomb counter is enabled during bqInit(). We do not use oneshot.
//If the counter is enabled in ALWAYS ON mode it will set the ALERT pin every 250ms. You can respond to this however you want.
//Host may clear the CC_READY bit or let it stay at 1.
bool AP_BattMonitor_BQ769x0::read_current(float &current_A)
{
    int16_t count;
    uint8_t* count_bytes = (uint8_t*)&count;
    if (!read_byte(BQ769x0_CC_HI, count_bytes[1])) {
        return false;
    }
    if (!read_byte(BQ769x0_CC_LO, count_bytes[0])) {
        return false;
    }

    current_A = (count * 8.44f)/(0.25f); //count should be naturally in 2's compliment.
    current_A /= 1000.0f;
    return true;
}

//Returns the pack voltage in volts
//Vbat = 4 * GAIN * ADC(cell) + (# of cells * offset)
bool AP_BattMonitor_BQ769x0::read_pack_voltage(float &pack_voltage)
{
    int16_t packADC = 0;
    uint8_t *packADC_bytes = (uint8_t*)&packADC;
    if (!read_byte(BQ769x0_BAT_HI, packADC_bytes[1])) {
        return false;
    }
    if (!read_byte(BQ769x0_BAT_LO, packADC_bytes[0])) {
        return false;
    }
    
    pack_voltage = 4 * _adc_gain * (uint16_t)packADC;
    pack_voltage += (_num_cells * _adc_offset); //Should be in mV
    pack_voltage /= 1000.0f;
    return true; //Convert to volts
}


bool AP_BattMonitor_BQ769x0::read_byte(uint8_t reg, uint8_t &val) {
    uint8_t data[2] {};
    bool ret = _dev->read_registers(reg, data, ARRAY_SIZE(data));
    val = data[0];
    return ret;
}

bool AP_BattMonitor_BQ769x0::write_byte(uint8_t reg, uint8_t val) {
    uint8_t data[3] {};
    data[0] = (AP_BATTMONITOR_BQ_I2C_ADDR << 1) | 0;
    data[1] = reg;
    data[2] = val;
    data[2] = crc8_ccitt(data, 3);
    data[0] = reg;
    data[1] = val;
    return _dev->transfer(data, ARRAY_SIZE(data), NULL, 0);
}

void AP_BattMonitor_BQ769x0::read(void) {

}

bool AP_BattMonitor_BQ769x0::power_on()
{
    uint8_t sys_ctrl2;
    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    
    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!read_byte(BQ769x0_SYS_CTRL2, sys_ctrl2)) {
        return false;
    }
    // switch DSG on
    if (!write_byte(BQ769x0_SYS_CTRL2, sys_ctrl2 | BQ769x0_DSG_ON)) {
        return false;
    }

    Debug("Powering ON...\n");
    return true;
}

bool AP_BattMonitor_BQ769x0::power_off()
{
    uint8_t sys_ctrl2;
    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    
    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!read_byte(BQ769x0_SYS_CTRL2, sys_ctrl2)) {
        return false;
    }
    // switch DSG off
    if (!write_byte(BQ769x0_SYS_CTRL2, sys_ctrl2 & ~BQ769x0_DSG_ON)) {
        return false;
    }
    Debug("Powering Off...\n");
    return true;
}

bool AP_BattMonitor_BQ769x0::start_charging()
{
    uint8_t sys_ctrl2;
    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!read_byte(BQ769x0_SYS_CTRL2, sys_ctrl2)) {
        return false;
    }
    // switch CHG on
    if (!write_byte(BQ769x0_SYS_CTRL2, sys_ctrl2 | BQ769x0_CHG_ON | BQ769x0_DSG_ON)) {
        return false;
    }

    Debug("Charging Started...\n");
    charging = true;
    return true;
}

bool AP_BattMonitor_BQ769x0::stop_charging()
{
    uint8_t sys_ctrl2 = 0;
    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!read_byte(BQ769x0_SYS_CTRL2, sys_ctrl2)) {
        return false;
    }
    // switch CHG off
    if (!write_byte(BQ769x0_SYS_CTRL2, sys_ctrl2 & ~(BQ769x0_CHG_ON | BQ769x0_DSG_ON))) {
        return false;
    }

    Debug("Charging Stopped...\n");
    charging = false;
    return true;
}


void AP_BattMonitor_BQ769x0::timer(void)
{
    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();

    if (!bus_sem) {
        return;
    }
    
    WITH_SEMAPHORE(bus_sem);

    uint32_t timestamp_us = AP_HAL::micros();
    uint8_t sys_stat = 0;
    if (!read_byte(BQ769x0_SYS_STAT, sys_stat)) {
        return;
    }

    if (!check_status(sys_stat)) {
        return;
    }

    // read coulomb counts
    if (sys_stat & BQ769x0_CC_READY ) {
        float current;
        if (read_current(current)) {
            _state.current_amps = current;
            _state.consumed_mah += current/4.0f/3.6f; // convert to mAh
        }
        write_byte(BQ769x0_SYS_STAT, BQ769x0_CC_READY); //address, val
    }

    // read cell voltages
    for (uint8_t i = 0; i < _num_cells; i++) {
        read_cell_voltage(i, _state.cell_voltages.cells[i]);
    }

    if (read_pack_voltage(_state.voltage)) {    
        _state.last_time_micros = timestamp_us;
        _state.healthy = true;
    } else {
        return;
    }

    read_temp(_state.temperature);
    _state.temperature_time = AP_HAL::millis();

    // Do Cell Balancing
    do_cell_balance();
}

void AP_BattMonitor_BQ769x0::do_cell_balance()
{
    if (abs(_state.current_amps) > IDLE_CURRENT_THRESHOLD) {
        idle_start_ms = AP_HAL::millis();
    }
    
    uint8_t maxV_cell_idx = 0, minV_cell_idx = 0;
    for (uint8_t i = 1; i < _num_cells; i++) {
        if (_state.cell_voltages.cells[i] > _state.cell_voltages.cells[maxV_cell_idx]) {
            maxV_cell_idx = i;
        }
        if (_state.cell_voltages.cells[i] < _state.cell_voltages.cells[minV_cell_idx]) {
            minV_cell_idx = i;
        }
    }
    
    if (charging &&
        (AP_HAL::millis() - idle_start_ms > CELL_BAL_START_IDLE_MS) &&
        (_state.cell_voltages.cells[maxV_cell_idx] > CELL_BAL_START_VOLTAGE )&&
        (_state.cell_voltages.cells[maxV_cell_idx] - _state.cell_voltages.cells[minV_cell_idx] > CELL_BAL_STOP_VDIFF)) {
        // do cell balancing
        uint8_t updated_balancing_status = 0, tgt_updated_balancing_status = 0;
        uint8_t cell_ids[_num_cells] {};
        uint8_t cell_cntr = 0;
        // find cells which should be balanced and sort them by voltage descending
        for (uint8_t i = 0; i < _num_cells; i++) {
            if ((_state.cell_voltages.cells[i] - _state.cell_voltages.cells[minV_cell_idx]) > CELL_BAL_STOP_VDIFF) {
                uint8_t j = cell_cntr;
                while (j > 0 && _state.cell_voltages.cells[cell_ids[j - 1]] < _state.cell_voltages.cells[i]) {
                    cell_ids[j] = cell_ids[j - 1];
                    j--;
                }
                cell_ids[j] = i;
                cell_cntr++;
            }
        }
        // setup updated balancing flags
        for (uint8_t i = 0; i < cell_cntr; i++) {
            // try to enable balancing of current cell
            tgt_updated_balancing_status = updated_balancing_status | (1 << cell_ids[i]);

            // check if attempting to balance adjacent cells
            bool adjacentCellCollision =
                ((tgt_updated_balancing_status << 1) & updated_balancing_status) ||
                ((updated_balancing_status << 1) & tgt_updated_balancing_status);

            if (adjacentCellCollision == false) {
                updated_balancing_status = tgt_updated_balancing_status;
            }
        }

        if (balancing_status != updated_balancing_status) {
            Debug("Switching Balancing: %x -> %x", balancing_status, updated_balancing_status);
        }

        // Finally try setting updated flags
        if (write_byte(BQ769x0_CELLBAL1, updated_balancing_status)) {
            balancing_status = updated_balancing_status;
        }
    } else if (balancing_status) {
        //disable cell balancing
        if (write_byte(BQ769x0_CELLBAL1, 0)) {
            Debug("Balancing Stopped");
            balancing_status = 0;
        }
    }
}

bool AP_BattMonitor_BQ769x0::check_status(uint8_t sys_stat) {
    uint8_t sys_write = 0;
    bool ret = true;

    // timeout after 5 seconds
    if ((AP_HAL::micros() - _state.last_time_micros) > AP_BATTMONITOR_BQ769x0_TIMEOUT_MICROS) {
        _state.healthy = false;
        ret = false;
    }

    //Internal fault
    if(sys_stat & BQ769x0_DEVICE_XREADY) {
        Debug("BQ769x0: Internal fault");
        sys_write |= BQ769x0_DEVICE_XREADY; //Clear this status bit by writing a one into this spot
        ret = false;
    }
    
    //Alert pin is being pulled high externally?
    if(sys_stat & BQ769x0_OVRD_ALERT) {
        Debug("BQ769x0: Override alert");
        sys_write |= BQ769x0_OVRD_ALERT; //Clear this status bit by writing a one into this spot
        ret = false;
    }

    //Under voltage
    if(sys_stat & BQ769x0_UV) {
        Debug("BQ769x0: Under voltage alert!");
        sys_write |= BQ769x0_UV; //Clear this status bit by writing a one into this spot
        ret = false;
    }

    //Over voltage
    if(sys_stat & BQ769x0_OV) {
        Debug("BQ769x0: Over voltage alert!");
        sys_write |= BQ769x0_OV; //Clear this status bit by writing a one into this spot
        ret = false;
    }

    //Short circuit detect
    if(sys_stat & BQ769x0_SCD) {
        Debug("BQ769x0: Short Circuit alert!");
        //sys_write |= BQ769x0_SCD; //Clear this status bit by writing a one into this spot
        ret = false;
    }

    //Over current detect
    if(sys_stat & BQ769x0_OCD) {
        Debug("BQ769x0: Over current alert!");
        //sys_write |= BQ769x0_OCD; //Clear this status bit by writing a one into this spot
        ret = false;
    }

    //Update the SYS_STAT with only the ones we want, only these bits will clear to zero
    if (sys_write != 0) {
        write_byte(BQ769x0_SYS_STAT, sys_write); //address, val
    }
    return ret;
}
