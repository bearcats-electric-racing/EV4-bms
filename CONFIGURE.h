

#ifndef CONFIGURE_H  
#define CONFIGURE_H

#include <stdint.h>

bool debug = 1;

//Safe operating conditions
const float max_temp = 58;
const float min_temp = 0;
const float OV = 4.20;       //over-voltage limit (spelled with an "oh" not zero) (V)
const float UV = 2.5;       //under-voltage limit (V)
const float max_diff = 0.75;   //max difference between min and max cell for open parallel cell detection. Set to ~0.1v.
const int watchdog_timeout = 0;  //watchdog timeout (in seconds). setting to 0 will DISABLE timer. Watchdog timer must be greater than 6 seconds
const float open_wire_threshold = 0.30; // % voltage attenuation tolerance during open wire check - attenuation exceeding this value will flag an open wire

//architecture
const int num_boards = 10;
const int num_cells = 14;       //cells per board
const int num_parrallel = 1;

//BMS operation mode. Leave as empty string to determine mode during runtime
String mode = "";     //"", "charge", "standby", "drive", "debug"

//CAN Bus Parameters
uint16_t BMS_ID = 0x123;             //standard ID of BMS TX messages
uint32_t INV_TX_ID = 0x0A7;         //CAN Message ID of message send from inverter of DC Bus Voltage (100 Hz frequency).
uint32_t CHG_TX_ID = 0x18FF50E5;    //CAN Message ID of messages sent from charger

// Charging parameters
const uint16_t CHG_voltage = 588;
const uint16_t CHG_current1 = 2; // up to 80% SOC
const uint16_t CHG_current2 = 2; // 80% to 90% SOC
const uint16_t CHG_current3 = 2; // 90% to 100% SOC

//balancing parameters
const float balance_threshold = 3.0;       // will not balance cells below this threshold (V)
const float balance_precision = 0.01;      // Will balance cells to this tolerance

//sense board parameters
int ADC_mode = 0;     //integer 0-7 to set ADC sampling frequency
#define wake_delay 2    //wake delay per board (milliseconds) to bring up power supply to voltage. Depends on Linear voltage regulator capacitance
#define cell_RC 0.0001  //C pin filter RC time constant in milliseconds (R*C*1000)


const int time_step = 10;      //timestep in milliseconds
const int volt_interval = 5;
const int temp_interval = 50;
const int current_interval = 1;
const int CAN_interval = 50;
const int SD_interval = 100;           //This needs to be the longest interval

//SD Card
float SD_card_size = 16;   //SD card size in Gb
int file_read_begin = 0;  //Starting file number of file to dump through serial


//Power Calculations
const int FULL_POWER_LEVEL_KW = 80; //Peak Power level
const int FULL_ENDURANCE_CURRENT = 40;
const int FULL_POWER_TEMP_C = 55; // Max temperature that has full power
const int ZERO_POWER_TEMP_C = 57; // Power linearly decreases from full power to zero power at this temperature
#endif