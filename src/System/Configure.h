#ifndef CONFIGURE_H  
#define CONFIGURE_H

#include <stdint.h>

typedef struct config_t {
    bool debug;
    float balance_threshold = 3.0;       // will not balance cells below this threshold (V)
    float balance_precision = 0.01;      // will balance cells to this tolerance
    float _qt = 12.6 * 60;               // total capacity (coulumbs): total capacity (Ah) * 60s/1hr
    float max_difference = 0.3;          // will not continue charging if max-min cell exceeds this threshold
    uint8_t adc_mode;                    // integer 0-7 to set ADC sampling frequency
} config_t;

// Safe operating conditions
const float MAX_TEMP = 58;                  // 2C max measurement error (due to thermistor + ADBMS6830B)
const float MIN_TEMP = 0;
const float GPIO_OV = 2.8125;               // Max expected GPIO voltage. Exceeding this value assumes broken thermistor, triggers shutdown
const float OV = 4.19;                      // over-voltage limit (spelled with an "oh" not zero) (V)
const float UV = 2.5;                       // under-voltage limit (V)
const float MAX_DIFF = 1.7;                 // max difference between min and max cell for open parallel cell detection
const uint8_t WATCHDOG_TIMEOUT = 10;        // watchdog timeout (in seconds). setting to 0 will DISABLE timer. Watchdog timer must be grester thatn 6 secibds

// Architecture
const uint8_t NUM_BOARDS = 10;
const uint8_t NUM_CELLS = 14;               // cells per board
const uint8_t NUM_PARALLEL = 4;
const uint8_t NUM_THERMISTORS = 10;
const uint8_t NUM_TIMERS = 2;

#define OFF 0
#define ON 1

#define CLEAR_REG   (0b00000000)
#define FULL_REG    (0b11111111)

// SPI pins
#define CS          (10)      // chip select pin isoSPI
#define CS2         (38)      // 3nd chip select pin isoSPI
#define SC          (20)      // shutdown circuit pin

// CAN pins
#define CRX3        (23)
#define CTX3        (22)
#define STBY        (21)      // CAN Transceiver Standby

#define CS1         (0)       // chip select for ADC

// CAN Bus Parameters
const uint16_t BMS_ID = 0x123;                // standard ID of BMS TX messages
const uint32_t INV_TX_ID = 0x0A7;             // CAN Message ID of message send from inverter of DC Bus Voltage (100 Hz frequency).
const uint32_t CHG_TX_ID = 0x18FF50E5;        // CAN Message ID of messages sent from charger

// BMS operation mode. Leave as Init string to determine mode during runtime
enum Mode {
    Init, // ""
    Charge,
    Standby,
    Drive,
    Balance,
    Debug
};

// Charging parameters
const uint16_t CHG_VOLTAGE = 588;
const uint16_t CHG_CURRENT1 = 9; // up to 80% SOC
const uint16_t CHG_CURRENT2 = 6; // 80% to 90% SOC
const uint16_t CHG_CURRENT3 = 4; // 90% to 100% SOC

// Sense board parameters
#define WAKE_DELAY (2)                      // wake delay per board (milliseconds) to bring up power supply to voltage. Depends on Linear voltage regulator capacitance
#define CELL_RC (0.0001)                    // C pin filter RC time constant in milliseconds (R*C*1000)

const uint8_t TIME_STEP = 10;               // timestep in milliseconds
const uint8_t VOLT_INTERVAL = 5;
const uint8_t CELL_TEMP_INTERVAL = 50;
const uint8_t PCB_TEMP_INTERVAL = 50;
const uint8_t CURRENT_INTERVAL = 1;
const uint8_t CAN_INTERVAL = 50;
const uint8_t SD_INTERVAL = 100;            // This needs to be the longest interval

// SD Card
const float SD_CARD_SIZE = 16;               // SD card size in Gb
const uint8_t FILE_READ_BEGIN = 0;          // Starting file number of file to dump through serial

// Power Calculations
const uint8_t FULL_POWER_LEVEL_KW = 80;     // Peak Power level
const uint8_t FULL_ENDURANCE_CURRENT = 40;
const uint8_t FULL_POWER_TEMP_C = 55;       // Max temperature that has full power
const uint8_t ZERO_POWER_TEMP_C = 57;       // Power linearly decreases from full power to zero power at this temperature


#endif

