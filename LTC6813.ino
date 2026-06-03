#include <SPI.h>
#include <cmath>
#include <string>

#include "COMMANDS.h"
#include "CONFIGURE.h"
#include "HEADER.h"
#include "LUTS.h"

#include "Watchdog_t4.h"
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <algorithm>

#include <SD.h>
#include <EEPROM.h>

//CAN pins
#define CRX3 23
#define CTX3 22
#define STBY 21  //CAN Transceiver Standby

//SD card pins
const int chipSelect = BUILTIN_SDCARD;

//SPI pins
#define CS 10   //chip select pin isoSPI
#define CS2 38  //3nd chip select pin isoSPI
#define CS1 0   //chip select for ADC

//Charger Pins
#define CHG_OUT 35    // Charger output pin for E-stop interlock
#define CHG_IN 34     // Charger input pin for E-stop interlock

//counters
unsigned int start_time = millis();
unsigned int sense_watchdog_timer;  //senseboard watchdog timer. Sense boards will go to sleep after 2 seconds if no valid command with correct PEC is sent from master.
bool new_voltage = false;
bool new_temp = false;

//faults
bool volt_sense_fault = 0;    // Configured
bool isoSPI_fault = 0;        // Configured
bool curr_sense_fault = 0;    // Configured
bool overvolt_fault = 0;      // Configured
bool undervolt_fault = 0;     // Configured
bool overtemp_fault = 0;      // Configured
bool undertemp_fault = 0;     // Configured
bool fusible_link_fault = 0;  // Configured

bool memory_fault = 0;
String serialBuffer = "";     //Serial buffer used for clearing faults

//fault positions (cell number / temp number)
uint8_t pos_volt_sense_fault = 0;   // Configured
uint8_t pos_overvolt = 0;   // Configured
uint8_t pos_undervolt = 0;   // Configured
uint8_t pos_overtemp = 0;      // Configured
uint8_t pos_undertemp = 0;      // Configured
uint8_t pos_fusible_link_fault = 0; // Not Configured

int wire_cut = 0;             // Not Configured
bool watchdog_callback = 0;   // Not Configured
bool watchdog_reset = 0;      // Not Configured
bool charger_fault = 0;       // Not Configured

bool CHG_EN = 0;  //0: enable charging, 1: disable charging

// Fault Memory Map (EEPROM)
// Byte 0:  76543210
//          0 = volt_sense_fault
//          1 = isoSPI_fault
//          2 = curr_sense_fault
//          3 = overvolt_fault
//          4 = undervolt_fault
//          5 = overtemp_fault
//          6 = undertemp_fault
//          7 = fusible_link_fault
// Byte 1:  pos_volt_sense_fault
// Byte 2:  pos_overvolt
// Byte 3:  pos_undervolt
// Byte 4:  pos_overtemp
// Byte 5:  pos_undertemp
// Byte 6:  pos_fusible_link_fault


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;  //    https://github.com/tonton81/FlexCAN_T4/tree/master

WDT_T4<WDT1> wdt;  //watchdog 1 holds output pin low until power-on-reset. This is desired for a shutdown circuit

// // Shared variables
float current = 0;

float current_offset = 0;

//state of charge
float soc = 0.0;

// data.csv enumeration
int data_file_num = 0;

//inverter voltage read fromc CAN
float inv_voltage = 0;

//LTC6813 minimum supply voltage is 16V
float cell_voltage[num_boards][num_cells];  //most recent cell voltages
float open_circuit_voltage[num_boards][num_cells];
float pack_voltage = 0;          //sum of cell voltages
float cell_temp[num_boards][10];  //most recent cell temperatures. Contans raw voltage data for the duration of open wire checks
float die_temps[num_boards];     //most recent sense board LTC6813 die temps
float min_cell_voltage = 0;
float max_cell_voltage = 0;
float min_cell_temp = 0;
float max_cell_temp = 0;
float min_die_temp = 0;
float max_die_temp = 0;

//sense board flags
float GPIO_open_wire[num_boards][9];
bool overvoltage_flag[18];
bool undervoltage_flag[18];

//measurement buffers
unsigned int time_buffer[SD_interval];
float voltage_buffer[SD_interval / volt_interval][num_boards][num_cells];
float temp_buffer[SD_interval / temp_interval][num_boards][9];
float current_buffer[SD_interval / current_interval];
float currentbuffer_stat = 0;
// RMS calc values
long current_count = 0;
long current_sum = 0;
int RMS_Current = 0;


void setup() {
  //open shutdown circuit
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);


  //Configure charger interlock
  // pinMode(CHG_OUT, OUTPUT);
  // pinMode(CHG_IN, INPUT);
  // digitalWrite(CHG_OUT, HIGH);

  delay(5000);  //startup delay should be use to make it easier to recover the teensy when runtime errors occurs

  //start timers
  sense_watchdog_timer = start_time - 5000;  //initial sense_watchdog timer with expired watchdog time (T - 2000 milliseconds)

  Serial.begin(9600);
  Serial.println("startup");
  Serial.print("Start Time: ");
  Serial.println(start_time);

  //SPI (isoSPI)
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  //SPI1 (ADC)
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));


  /*
  //EV3 CAN Setup
  pinMode(CRX3, INPUT);
  pinMode(CTX3, OUTPUT);
  pinMode(STBY, OUTPUT);
  can.begin();
  can.setBaudRate(250000);
  can.setMaxMB(3);  //number of CAN message mailboxes
  digitalWrite(STBY, LOW);
  //    https://github.com/tonton81/FlexCAN_T4/blob/master/examples/mailbox_filtering_example_with_interrupts/mailbox_filtering_example_with_interrupts.ino
  // Mailboxes must be configured for all messages - both TX and RX
  can.setMB((FLEXCAN_MAILBOX)0, RX, STD);  //Standard mailbox for Inverter ID
  can.setMB((FLEXCAN_MAILBOX)1, RX, EXT);  //Extended id for charger
  can.setMB((FLEXCAN_MAILBOX)2, TX, EXT);  //BMS TX -> charger id
  can.setMBFilter(MB0, INV_TX_ID);         //Mailbox for Inverter CAN messages
  can.setMBFilter(MB1, CHG_TX_ID);         //Mailbox for Charger CAN Messages
  can.setMBFilter(MB2, 0x1806E5F4);        //Mailbox for Charger CAN Messages
  */

  //EV2 CAN
  pinMode(CRX3, INPUT);
  pinMode(CTX3, OUTPUT);
  pinMode(STBY, OUTPUT);
  can.begin();
  can.setBaudRate(250000);
  can.enableFIFO();

  //Watchdog
  if (watchdog_timeout != 0) {  //callback function is having some issues
    WDT_timings_t config;
    int watchdog_trigger = watchdog_timeout - 1;
    if (watchdog_trigger < 1) {
      watchdog_trigger = 1;
    }
    config.trigger = 11; /* in seconds, 0->128 */                //time until watchdog callback function is triggered.
    config.timeout = watchdog_timeout; /* in seconds, 0->128 */  //time until watchdog reset
    config.pin = 20;                                             //pin to be driven low upon reset. WDT1 holds low, WDT2 pulses low
    config.callback = myCallback;
    wdt.begin(config);
  }

  //Bring up ADC
  initialize_ADC();

  //current offset compensation
  measure_current();
  current_offset = current;

  check_memory();  //must be called to use SD card

  //voltage poll and temperature poll take 16 and 24 milliseconds. The rest of the measure functions only take 1 or two milliseconds

  if (mode == "") {
    measure_voltage();
    measure_current();
    update_SOC();

    CAN_message_t msg;
    bool CAN_baud_alt = true;
    while (1) {
      Serial.println("Setup");
      measure_current();
      measure_voltage();
      measure_temp();
      update_SOC();
      TX_CAN();       //wrong baud rate every other message
      print_min_max();
      reset_watchdog();
      //update_faults();
      //print_faults();
      msg = RX_CAN();

      //Alternate CAN baud rate (250000 for charger, 500000 for vehicle)
      if(CAN_baud_alt){
        can.setBaudRate(500000);
        CAN_baud_alt = false;
      }
      else{
        can.setBaudRate(250000);
        CAN_baud_alt = true;
      }
      
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      
      if (msg.id == INV_TX_ID && false) {  //Always check msg id. Stdby has not yet been tested
        mode = "standy";
        can.setBaudRate(500000);
        //can.setMBFilter(MB1, 0);  //Disable Charger Mailbox
        break;
      } else if (msg.id == CHG_TX_ID) {
        mode = "charge";
        can.setBaudRate(250000);
        //can.setMBFilter(MB0, 0);  //Disable Inverter Mailbox
        break;
      } else if (current >= 0.5) {    //enter directly into drive mode if current is detected
        can.setBaudRate(500000);
        mode = "drive";
        break;
      } else if (input == "debug") {
        mode = "debug";
        break;
      }
    
      
      delay(20);
      Serial.println("End of Setup Loop");
    }
  }
}

void loop() {
  if (mode == "charge") {
    Serial.println("Charge Mode Entered");
    CAN_message_t msg;
    float charger_voltage = 0;
    float charger_current = 0;
    String filename = "data" + String(data_file_num) + ".csv";  //create data file
    File file = SD.open(filename.c_str(), FILE_WRITE);
    file.close();

    delay(6000);  //cause comm fault on charger. Power cycling the BMS without ensuring the charger fully powers down would otherwise can cause the BMS to enter the charge cycle agian.

    while (1) {  //precharge cycle
      measure_voltage();
      measure_temp();
      reset_watchdog();
      charger_enable(true);  //send charge-disable message and clear comm fault on charger
      msg = RX_CAN();
      charger_voltage = ((uint16_t)msg.buf[0] << 8 | (uint16_t)msg.buf[1]) / 10;
      charger_current = ((uint16_t)msg.buf[2] << 8 | (uint16_t)msg.buf[3]) / 10;
      Serial.println(charger_voltage);
      Serial.println(pack_voltage);
      //if (msg.id == CHG_TX_ID && msg.buf[4] == 0 || true){ //&& charger_voltage >= pack_voltage * 0.80) {  //if can id matches charger AND there are no charger faults AND precharge is complete
      break;
      //}
    }
    //00100 low ac power on charger flag
    delay(1000);  //delay so that another Charger CAN message is sent to the BMS (so that an empty CAN buffer is not read which would indicate a charger error)

    unsigned int charge_start_time = millis();
    while (1) {  //charge cycle
      Serial.print("Time (minutes): "); Serial.println((float)(millis() - charge_start_time)/60000);
      Serial.print("charge fault status: ");
      Serial.println(msg.buf[4]);
      measure_voltage();
      measure_temp();
      measure_current();
      Serial.print("current: ");
      Serial.println(current);
      Serial.print("Pack Voltage: ");
      Serial.println(pack_voltage);
      print_min_max();
      if (!memory_fault) {
        SD_data_write();
      }
      if (reset_watchdog()) {
        msg = RX_CAN();
        charger_voltage = ((uint16_t)msg.buf[0] << 8 | (uint16_t)msg.buf[1]) / 10;
        charger_current = ((uint16_t)msg.buf[2] << 8 | (uint16_t)msg.buf[3]) / 10;
        Serial.print("charger voltage: ");
        Serial.println(charger_voltage);
        Serial.print("charger current: ");
        Serial.println(charger_current);
        if (msg.id == CHG_TX_ID && msg.buf[4] == 0 || true) {
          charger_enable(false);
        } else {  //charger error
          digitalWrite(20, LOW);
          charger_enable(true);
          Serial.println("Charger Error");
          charger_fault = 1;
        }
      }
      delay(1000);
    }

    //charger fault
    while (1) {
      Serial.println("Charger Fault");
      delay(1000);
    }
  }

  if (mode == "standby") {  //waiting to drive. Still provides rules-compliant monitering in case CAN is lost
    Serial.println("Standby Mode Entered");

    if (!memory_fault) {
      String filename = "data" + String(data_file_num) + ".csv";  //create data file
      File file = SD.open(filename.c_str(), FILE_WRITE);
      file.close();
      SD_data_write();  //write initial conditions to data file once
    }

    while (1) {
      Serial.println(mode);
      CAN_message_t msg;
      measure_current();
      measure_voltage();
      measure_temp();
      reset_watchdog();
      msg = RX_CAN();
      if (msg.id == INV_TX_ID) {
        inv_voltage = float(msg.buf[0] * 256 + msg.buf[1]);
      }
      if (inv_voltage >= pack_voltage * 0.8) {  //checks inverter voltage to see if precharge is occuring
        mode = "drive";                         //enter drive mode if precharging
        break;
      }
      delay(10);
    }
  }

  else if (mode == "drive") {
    Serial.println("Drive Mode Entered");
    int n = 0;  //time step number

    CAN_message_t msg;

    while (1) {
      time_buffer[n] = millis() - start_time;
      if (n % current_interval == 0) {
        measure_current();
        current_buffer[int(n / current_interval)] = current;
      }
      if (n % volt_interval == 0) {
        measure_voltage();
        Serial.println("New volt");
        for (int i = 0; i < num_boards; i++) {
          for (int j = 0; j < num_cells; j++) {
            voltage_buffer[int(n / volt_interval)][i][j] = cell_voltage[i][j];
          }
        }
      }
      if (n % temp_interval == 0) {
        Serial.println("New Temp");
        measure_temp();
        for (int i = 0; i < num_boards; i++) {
          for (int j = 0; j < 9; j++) {
            temp_buffer[int(n / temp_interval)][i][j] = cell_temp[i][j];
          }
        }
      }
      if (new_voltage && new_temp) {
        print_min_max();
        reset_watchdog();
      }
      if (n % SD_interval == 0 && !memory_fault) {
        SD_data_write();
      }
      if (n % CAN_interval == 0) {
        Serial.println("Send CAN");
        update_SOC();
        TX_CAN();
      }

      // msg = RX_CAN();
      // if(msg.id == INV_TX_ID){
      //   inv_voltage = float(msg.buf[0]*256 + msg.buf[1]);
      // }
      // if(inv_voltage < pack_voltage * 0.5){    //checks inverter voltage to see if tractive system voltage is dropping
      //   mode = "standby";                         //enter standby mode if ready to drive is exited
      //   //data_file_num = 0;
      //   //check_memory();         //assign a new data file number in case RTD is entered agian
      //   break;
      // }

      while (millis() - start_time <= time_buffer[n] + time_step) {  //this needs checked
      }
      if (n < SD_interval - 1) {
        n++;
      } else {
        n = 0;
      }
      //Serial.println(n);
    }

  }

  else {                    //debug mode
    digitalWrite(20, LOW);  //open shutdown circuit in debug mode
    Serial.println("Debug Mode Entered");
    while (1)
      dumpDataToSerial();
  }
}

void initialize_ADC() {
  //ADC sampling time constant (without external filter) = 50 ohms * 40 pF
  //CFR.B6 = 0 : uses external voltage reference
  //CFR.B9 = 0, FSR_ADC_A = 0 to VREF_A and FSR_ADC_B = 0 to VREF_B
  //CFR.B7 = 0 : single ended measurements
  //CFR.B11 = 0 and CFR.B10 = 1  Single-SDO Mode
  //CFR.B15:B11 = 1000 (write) or 0011 (read)
  uint8_t CFR_reg_MSB;
  uint8_t CFR_reg_LSB;
  uint8_t CFR_readback_MSB;
  uint8_t CFR_readback_LSB;
  CFR_reg_MSB = 0b10000100;  //CFR [B15:B8]
  //CFR_reg_LSB = 0b01000000;  //CFR [B7:B0]
  CFR_reg_LSB = 0b00000000;

  //Send write CFR register command
  digitalWrite(CS1, LOW);
  SPI1.transfer(CFR_reg_MSB);
  SPI1.transfer(CFR_reg_LSB);
  for (int i = 0; i < 6; i++) {  //clock ADC
    SPI1.transfer(0b00000000);
  }
  digitalWrite(CS1, HIGH);
  delay(2);

  //Send read CFR register command while clocking the write config
  digitalWrite(CS1, LOW);
  SPI1.transfer(0b00110000);
  for (int i = 0; i < 6; i++) {
    SPI1.transfer(0b00000000);  //clock ADC
  }
  digitalWrite(CS1, HIGH);
  delay(2);

  //readback the CFR configuration
  digitalWrite(CS1, LOW);
  CFR_readback_MSB = SPI1.transfer(0b00000000);
  CFR_readback_LSB = SPI1.transfer(0b00000000);
  for (int i = 0; i < 6; i++) {
    SPI1.transfer(0b00000000);  //clock ADC
  }
  digitalWrite(CS1, HIGH);

  //the 4 MSBs of the CFR register (read/write command bits) are cleared in Frame F+2 which is not consistant with the datasheet
  if ((uint8_t)(CFR_reg_MSB << 4) != (uint8_t)(CFR_readback_MSB << 4) || (uint8_t)(CFR_reg_LSB << 4) != (uint8_t)(CFR_readback_LSB << 4)) {  //bit-shifts to mask the 4 MSBs
    Serial.println("ADC_initialization ERROR");
    Serial.println((CFR_reg_MSB << 4), BIN);
    Serial.println((CFR_reg_LSB << 4), BIN);
    curr_sense_fault = 1;
  }
}

void read_ADC() {
  uint16_t ADC_A;
  uint16_t ADC_B;
  float A_volt;
  float B_volt;
  uint16_t temp;
  digitalWrite(CS1, LOW);
  for (int i = 0; i < 2; i++) {
    temp = SPI1.transfer(0b00000000);  //clock ADC
    Serial.print("temp: ");
    Serial.println(temp, BIN);
  }
  ADC_A = SPI1.transfer(0b00000000);
  ADC_A = ADC_A << 8;
  ADC_A = ADC_A | SPI1.transfer(0b00000000);
  Serial.print("ADC_A: ");
  Serial.println(ADC_A);
  ADC_B = SPI1.transfer(0b00000000);
  ADC_B = ADC_B << 8;
  ADC_B = ADC_B | SPI1.transfer(0b00000000);
  Serial.print("ADC_B: ");
  Serial.println(ADC_B);
  temp = SPI1.transfer(0b00000000);
  Serial.print("temp: ");
  Serial.println(temp, BIN);
  digitalWrite(CS1, HIGH);
  delay(2);
  A_volt = (float)(ADC_A) / 65535 * 5;
  B_volt = (float)(ADC_B) / 65535 * 5;
  Serial.print("A Voltage: ");
  Serial.println(A_volt);
  Serial.print("B Voltage: ");
  Serial.println(B_volt);
}


void print_min_max() {  //This function prints the min and max parameters
  Serial.print("Max cell voltage: ");
  Serial.println(max_cell_voltage);
  Serial.print("Min cell_voltage: ");
  Serial.println(min_cell_voltage);
  Serial.print("Max cell_temp: ");
  Serial.println(max_cell_temp);
  Serial.print("Min cell_temp: ");
  Serial.println(min_cell_temp);
  Serial.print("Max die temp: ");
  Serial.println(max_die_temp);
  Serial.print("Min die temp: ");
  Serial.println(min_die_temp);
}


void dumpDataToSerial() {
  while (1) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "begin") {
      break;
    }
  }

  File root = SD.open("/");
  File entry = root.openNextFile();
  int file_num = 0;
  while (entry) {
    if (file_num >= file_read_begin) {  //begin with file at file_read_begin
      Serial.println(entry.name());
      while (entry.available()) {
        char character = entry.read();
        Serial.write(character);

        while (character == '\n') {  //block until confirmation from Python that line has been read
          String input = Serial.readStringUntil('\n');
          input.trim();
          if (input == "next line") {
            break;
          }
        }
      }
      Serial.println("done");

      while (1) {  //block until Python is ready to read next file
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "next file") {
          break;
        }
      }
    }
    entry.close();
    entry = root.openNextFile();
    file_num++;
  }
  root.close();

  Serial.println("serial dump done");
}

void check_memory() {  //this should check all files
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    memory_fault = 1;
    return;
  }

  if (!SD.exists("state.txt")) {
    File file = SD.open("state.txt", FILE_WRITE);
    file.close();
  }

  File root = SD.open("/");
  File entry = root.openNextFile();
  uint64_t memory_usage = 0;
  int num_files = 0;
  while (entry) {
    num_files++;
    Serial.print(entry.name());
    Serial.print("\t");
    Serial.print(entry.size());
    Serial.println(" bytes");

    memory_usage += entry.size();
    entry.close();
    //SD.remove(entry.name());
    entry = root.openNextFile();
  }
  root.close();
  Serial.print("Memory Usage: ");
  Serial.print(100 * memory_usage / (SD_card_size * 1e9));
  Serial.println("%\n");
  if (memory_usage > 0.9 * (SD_card_size * 1e9)) {
    Serial.println("SD card over 90% full");
    memory_fault = 1;
    return;
  }
  if (data_file_num == 0) {
    for (int i = 1; i < num_files + 100; i++) {
      String filename = "data" + String(i) + ".csv";
      if (!SD.exists(filename.c_str())) {
        data_file_num = i;
        break;
      }
    }
  }
}

void get_SOC() {  //SOC should be written in the state.txt file as: "SOC:100"

  if (SD.exists("state.txt")) {
    File file = SD.open("state.txt", FILE_READ);
    if (file) {
      String line;
      while (file.available()) {
        line = file.readStringUntil('\n');
        Serial.println(line);
        int delim_index = line.indexOf(':');
        String name = line.substring(0, delim_index);
        String value = line.substring(delim_index + 1);
        map_text2var(name, value);
      }
      file.close();
    }
  } else {
    File file = SD.open("state.txt", FILE_WRITE);
    Serial.println("Initializing state of charge to 100%");
    file.close();
    Serial.println("state.txt initialized.");
  }
}

void map_text2var(String name, String value) {  //map text name and value to a variable
  if (name == "SOC:") {
    soc = value.toFloat();
    Serial.println(soc);
  }
}

float update_SOC() { 

    const int discharge_curve_length = sizeof(discharge_points) / sizeof(discharge_points[0]);  //length of each discharge curve
    const float max_capacity = discharge_points[0];                                             //maximum capacity of a single cell
    const int num_current_curves = sizeof(discharge_currents) / sizeof(discharge_currents[0]);  //number of discharge curves @ different currents

    float min_OC_cell_voltage = open_circuit_voltage[0][0];  //funct. min_max requires that the min and max values are initalized within the range of the min max values
    float max_OC_cell_voltage = open_circuit_voltage[0][0];
    min_max<num_boards, num_cells>(open_circuit_voltage, &min_OC_cell_voltage, &max_OC_cell_voltage);
    float discharged = interpolate<discharge_curve_length>(discharge_curves[0], discharge_points, min_OC_cell_voltage);  //capacity which has already been discharged (mAh)
    soc = 100 - ((max_capacity - discharged) / max_capacity * 100);
    
  Serial.print("SOC: ");
  Serial.println(soc);
  return soc;
}

void upadate_current_limit() {
  const int discharge_curve_length = sizeof(discharge_points) / sizeof(discharge_points[0]);  //length of each discharge curve
  const float max_capacity = discharge_points[0];                                             //maximum capacity of a single cell
  const int num_current_curves = sizeof(discharge_currents) / sizeof(discharge_currents[0]);  //number of discharge curves @ different currents
}

void SD_data_write() {
  String filename = "data" + String(data_file_num) + ".csv";
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  if (dataFile) {
    dataFile.print("Mode: ");
    dataFile.println(mode);
    for (int n = 0; n < SD_interval; n++) {
      dataFile.print("Voltage:\n");
      if (n % volt_interval == 0 || mode != "drive") {
        for (int i = 0; i < num_boards; i++) {
          for (int j = 0; j < num_cells; j++) {
            if (mode == "drive")
              dataFile.print(voltage_buffer[int(n / volt_interval)][i][j], 4);
            else
              dataFile.print(cell_voltage[i][j], 4);
            dataFile.print(", ");
          }
          dataFile.print("\n");
        }
      }
      if (n % temp_interval == 0 || mode != "drive") {
        dataFile.print("\nTemperature:\n");
        for (int i = 0; i < num_boards; i++) {
          for (int j = 0; j < 9; j++) {
            if (mode == "drive")
              dataFile.print(temp_buffer[int(n / temp_interval)][i][j], 2);
            else
              dataFile.print(cell_temp[i][j], 2);
            dataFile.print(", ");
          }
          dataFile.print("\n");
        }
      }
      if (n % current_interval == 0 || mode != "drive") {
        dataFile.print("Current: ");
        if (mode == "drive")
          currentbuffer_stat = current_buffer[int(n / current_interval)];
        dataFile.print(currentbuffer_stat);
      } else {
        dataFile.print(current);
        dataFile.print("\n");
      }
      dataFile.print("Time:\n");

      //time stamp
      if (mode == "drive")
        dataFile.println(time_buffer[n]);
      else
        dataFile.println(millis() - start_time);
      break;
    }
    dataFile.close();
  }
}

void send_command(uint16_t command) {
  uint8_t comm_arr[2];
  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t cmd0;
  uint8_t cmd1;

  cmd0 = command >> 8;
  cmd1 = command >> 0;

  if (millis() - sense_watchdog_timer >= 1800) {
    wakeup_sleep(num_boards + 1);
    sense_watchdog_timer = millis();
  } else {
    wakeup_idle(num_boards);
    sense_watchdog_timer = millis();
  }

  digitalWrite(CS, LOW);

  comm_arr[0] = cmd0;
  comm_arr[1] = cmd1;

  pec = pec15_calc(2, comm_arr);
  pec1 = pec >> 0;
  pec0 = pec >> 8;

  SPI.transfer(cmd0);
  SPI.transfer(cmd1);
  SPI.transfer(pec0);
  SPI.transfer(pec1);
}

void read_register_group(uint16_t command, uint8_t response[num_boards][6]) {  //register group is always 6 bytes

  uint8_t ccmd; // command counter
  uint16_t rx_pec10; // Recieved and parsed 10 bit data PEC
  uint16_t calc_pec10; // Calculated 10 bit data PEC
  uint8_t response_pec0;
  uint8_t response_pec1;

  send_command(command);

  for (int i = 0; i < num_boards; i++) {
      for (int j = 0; j < 6; j++) {
          response[i][j] = SPI.transfer(0b11111111); // Send dummy byte to receive data
          // Serial.println(response[i][j], BIN);
      }

      response_pec0 = SPI.transfer(0xFF); // response PEC = command counter + PEC, needs to parsed
      response_pec1 = SPI.transfer(0xFF);
      
      // Extract command counter and received 10-bit PEC from the ADBMS6830B readback format:
      // PEC0 = [CCNT5..0 | PEC9..8], PEC1 = [PEC7..0]
      ccmd = (response_pec0 >> 2) & 0x3F;
      rx_pec10 = ((uint16_t)(response_pec0 & 0x03) << 8) | response_pec1;

      calc_pec10 = pec10_calc_data_ccnt(response[i], ccmd);

      /*
      if (rx_pec10 != (calc_pec10 & 0x3FF)) {
          Serial.println("PEC Error - Data PEC Mismatch");
          wakeup_sleep(NUM_BOARDS + 1);
      }
      */

      //Debug Print
      // if(debug){
      //   Serial.print("response PEC ");
      //   Serial.println(rx_pec10, HEX);
      //   Serial.print("calculated PEC ");
      //   Serial.println(calc_pec10, HEX);
      // }

      if(rx_pec10 != calc_pec10){
        isoSPI_fault = 1;
      }

  }

  

  digitalWrite(CS, HIGH);
}

void write_register_group(uint16_t command, uint8_t data[num_boards][6]) {

  uint8_t return_data;
  uint8_t data_pec0;
  uint8_t data_pec1;
  uint16_t data_pec;

  send_command(command);

  for (int i = num_boards - 1; i >= 0; i--) {
    data_pec = pec15_calc(6, data[i]);
    data_pec1 = data_pec >> 0;
    data_pec0 = data_pec >> 8;

    for (int j = 0; j < 6; j++) {
      SPI.transfer(data[i][j]);
    }
    SPI.transfer(data_pec0);
    SPI.transfer(data_pec1);
  }
  digitalWrite(CS, HIGH);
}

void poll_ADC(uint16_t command, bool curr_measure) {
  uint8_t return_data = 0;
  send_command(command);

  if (!curr_sense_fault && curr_measure) {
    measure_current();
  }

  int num_polls = 0;
  uint32_t adc_poll_start_time = millis();

  while (return_data == 0) {
    return_data = SPI.transfer(0b11111111); // Send dummy byte to receive data
    num_polls++;
    if(millis() - adc_poll_start_time > 1000){          // 1000ms is a made up number and should be reviewed. This previously had no timeout.
        Serial.println("ADBMS6830B ADC TImeout Error");
        break;
    }
}

  // Serial.println("ADC Conversion Done!");
  // Serial.println(num_polls);

  digitalWrite(CS, HIGH);
}

void measure_voltage() {  //18 millisecond execution time
  uint8_t response[num_boards][6];
  uint16_t cell_comm[6] = { RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF };  //read cell voltage registers A through E commands

  ////cell voltage measurement algorithm outlined in INTERNAL PROTECTION AND FILTERING section of LTC6813 datasheet////
  //poll_ADC(ADCV | 0b1);   //measure cells 1,7,13 to allow MUX voltage to settle
  //delay(cell_RC * 6);

  poll_ADC(ADCV);  //initiate and wait for voltage measurement

  pack_voltage = 0;
  for (int i = 0; i * 3 < num_cells; i++) {  //i: cell group
    //Serial.print('i');
    //Serial.println(i);
    uint16_t curr_comm = cell_comm[i];  //each command reads a sequential set of three cells from each board
    read_register_group(curr_comm, response);
    for (int j = 0; j < num_boards; j++) {  //j:board number
      //Serial.print('j');
      //Serial.println(j);
      for (int k = 0; k < 3 && i * 3 + k < num_cells; k++) {                                                         //cell number within register group
        int16_t adc_code = (int16_t)(((uint16_t)response[j][k * 2 + 1] << 8) | response[j][k * 2]);
        cell_voltage[j][i * 3 + k] = (float)adc_code * 0.00015f + 1.5f; // LSB represents 150 uV, +1.5v offset
        pack_voltage += cell_voltage[j][i * 3 + k];
      }
    }
  }

  // Update open circuit voltage if applicable
  if (current < 0.2 and current > -0.2) {
    for (int i = 0; i < num_boards; ++i) {
      for (int j = 0; j < num_cells; ++j) {
        open_circuit_voltage[i][j] = cell_voltage[i][j];
      }
    }
  }

  // Update min, max voltage
  min_cell_voltage = cell_voltage[0][0];
  max_cell_voltage = cell_voltage[0][0];
  min_max<num_boards, num_cells>(cell_voltage, &min_cell_voltage, &max_cell_voltage);

  new_voltage = true;

  if (debug) {
    Serial.println("Voltages:");
    int g = 0;
    for (int i = 0; i < num_boards; i++) {
      Serial.print("board: ");
      Serial.println(i + 1);
      for (int j = 0; j < num_cells; j++) {
        Serial.print(cell_voltage[i][j]);
        Serial.print(" ");
        g++;
      }
      Serial.println("");
    }
  }
}

float map_temp(float V) {
    int const size = sizeof(NTC_LUT) / sizeof(NTC_LUT[0]);
    float R_bias = 10000;
    float V_ref = 3.00;
    
    if (V_ref == V) // divide by zero case
    return -55;
    
    float NTC_res = (V / V_ref * R_bias) / (1 - V / V_ref);
    int i = search<size>(NTC_LUT, NTC_res);
    float temperature = float(i) / float(size) * (150 + 55) - 55;
    return (temperature);
}

void measure_temp(bool open_wire_check) {  //25 millisecond execution time

    uint8_t response[num_boards][6];
    uint16_t aux_comm[4] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD}; // read aux registers A through D commands
    int thermistor_idx = 0; // thermistor index 0-9
    int command_idx = 0; // command index within aux_comm array

    poll_ADC(ADAX);

    while (thermistor_idx < 10) {
        uint16_t curr_comm = aux_comm[command_idx];     // each command reads a sequential set of
                                                        // three GPIO from each board (RDAUXB is an
                                                        // exception with just 2 GPIO)
        read_register_group(curr_comm, response);
        for (int reading = 0; reading < 3 && thermistor_idx < 10; reading++) { // GPIO reading within group (~3 per group)
            if (command_idx == 3 && reading > 0) // Group register D has 1 reading only
                continue;

            for (int b = 0; b < num_boards; b++) {
                int16_t adc_code = (int16_t)(((uint16_t)response[b][reading * 2 + 1] << 8) | response[b][reading * 2]);
                // if(!thermistor_idx){
                //   Serial.print("ADC Code: ");
                //   Serial.println(adc_code);
                //   Serial.print("Converted Voltage: ");
                //   Serial.println((float)adc_code*0.00015f + 1.5f);
                // }
                cell_temp[b][thermistor_idx] = (float)adc_code * 0.00015f + 1.5f; // LSB represents 150 uV, +1.5v offset
            }
            thermistor_idx++;
        }
        command_idx++;
    }

    // Map cell temps
    for (int i = 0; i < num_boards; i++)
        for (int j = 0; j < 10; j++)
            cell_temp[i][j] = map_temp(cell_temp[i][j]);
    
    // Filer out unused thermistors
    // Sense boards wired rear --> front --> rear (Module slot 1 --> 2 --> 3...)
    // Bad thermistors:
    // Module 1
    //    board 0: GPIO5
    //    board 1
    // Module 2
    //    board 2
    //    board 3
    // Module 3 
    //    board 4: GPIO1
    //    board 5 
    // Module 4
    //    board 6: GPIO1
    //    board 7
    // Module 5
    //    board 8
    //    board 9: GPIO2, 4, 6, 8, 10

    
    // cell_temp[0][4] = -55.0f;
    // cell_temp[4][0] = -55.0f;
    // cell_temp[6][0] = -55.0f;
    // cell_temp[9][1] = -55.0f;
    // cell_temp[9][3] = -55.0f;
    // cell_temp[9][5] = -55.0f;
    // cell_temp[9][7] = -55.0f;
    // cell_temp[9][9] = -55.0f;

    // Update min / max
    min_cell_temp = 160.0f;
    max_cell_temp = -60.0f;

    for (int i = 0; i < num_boards; i++){
      for (int j = 0; j < 10; j++){
        if(cell_temp[i][j] == -55.0f) continue;
        if(cell_temp[i][j] < min_cell_temp){
          min_cell_temp = cell_temp[i][j];
        }
        if(cell_temp[i][j] > max_cell_temp){
          max_cell_temp = cell_temp[i][j];
        }
      }
    }
      

    new_temp = true;

    if (debug) {
        Serial.println("Temperatures:");
        for (int i = 0; i < num_boards; i++) {
            print_with_args("\tBoard: %d\n\t", i + 1);
            
            for (int j = 0; j < 10; j++) {
                print_with_args("%f ", cell_temp[i][j]);
            }
            Serial.println("");
        }
    }
}

// 37ms execution time
void cell_open_wire_check() {
    uint8_t response[num_boards][6];                    // ADBMS6830B response
    float s_voltage_open[num_boards][num_cells];        // S voltage values, OW switch open (baseline)
    float s_voltage_closed[num_boards][num_cells];      // S voltage values, OW switch closed
    bool open_wire_flags[num_boards][num_cells] = {false};
    bool open_wire = 0;

    uint16_t cell_comm[6] = {
        RDSVA, RDSVB, RDSVC,
        RDSVD, RDSVE, RDSVF
    }; // read S voltage registers A through F commands

    /*
    Open Wire Check Sequence:
        1.) poll ADC for baseline s voltage, read register groups
        2.) poll ADC with even open-wire check, read register groups
        3.) poll ADC with odd open-wire check, read register groups
        After Loop: Check for out of tolerance differences between open / closed wire
    */

    for (uint8_t i = 0; i < 3; i++){
        if(i == 0){
            poll_ADC(ADSV);                // S-ADC poll, OW switches open
        }
        else if(i == 1){
            poll_ADC(ADSV | OW_EVEN);      // S-ADC poll, even OW switches closed
        }
        else if(i == 2){
            poll_ADC(ADSV | OW_ODD);       // S-ADC poll, odd OW switches closed
        }

        for (int j = 0; j * 3 < num_cells; j++) { // i: cell group (3 cells per register group)
            // Serial.print('j');
            // Serial.println(j);
            uint16_t curr_comm = cell_comm[j]; // each command reads a sequential set
                                            // of three cells from each board
            read_register_group(curr_comm, response);
            for (int k = 0; k < num_boards; k++) { // j: board number
                // Serial.print('k');
                // Serial.println(k);
                for (int l = 0; l < 3 && j * 3 + l < num_cells; l++) { // cell within cell group
                    // Serial.print('l');
                    // Serial.println(l);
                    int16_t adc_code = (int16_t)(((uint16_t)response[k][l * 2 + 1] << 8) | response[k][l * 2]);
                    
                    // Initial loop, write all baseline voltages to s_voltage_open
                    if(i == 0){
                        s_voltage_open[k][j * 3 + l] = (float)adc_code * 0.00015f + 1.5f; // LSB represents 150 uV, +1.5v offset
                    }
                    // Loop 1, write even cells only (note: index 0 = cell 1)
                    else if(i == 1){
                        if((j * 3 + l) % 2){
                            s_voltage_closed[k][j * 3 + l] = (float)adc_code * 0.00015f + 1.5f;
                        }
                    }
                    // Loop 2, write odd cells only
                    else if(i == 2){
                        if( !((j * 3 + l) % 2)) {
                            s_voltage_closed[k][j * 3 + l] = (float)adc_code * 0.00015f + 1.5f;
                        }
                    }
                }
            }
        }
    }

    // Check for out of tolerance voltage drops
    for (int i = 0; i < num_boards; i++){
        for (int j = 0; j < num_cells; j++){
            if(s_voltage_closed[i][j] < (s_voltage_open[i][j] * (1.00f - open_wire_threshold)) ){
                open_wire_flags[i][j] = 1;
                open_wire = 1;
                if(debug){
                    Serial.println("Open Voltage Sense Lead!");
                    print_with_args("Board: %d Cell: %d", i + 1, j + 1);
                    pos_volt_sense_fault = i*num_cells + j;
                }
            }
        }
    }

    if(open_wire){
        digitalWrite(20, LOW);
        delay(1000); // delay to overcome debounce of shutdown circuit
        wire_cut = 1;
        Serial.println("Open voltage sense lead detected");
    }


}

bool reset_watchdog() {  //this needs to clear the voltage and temperature measurements after reading them
  new_voltage = false;
  new_temp = false;

  // Check cell voltages
  for (int i = 0; i < num_boards; i++) {
    for (int j = 0; j < num_cells; j++) {
      if (cell_voltage[i][j] < OV && cell_voltage[i][j] > UV) {
        cell_voltage[i][j] = 0;
        continue;
      } else {
        digitalWrite(20, LOW);
        delay(1000);  //delay to overcome debounce of shutdown circuit
        Serial.println("invalid voltage");
        Serial.println(cell_voltage[i][j]);

        if(cell_voltage[i][j] > OV){
          overvolt_fault = 1;
          pos_overvolt = i*num_cells + j;
        }
        else{
          undervolt_fault = 1;
          pos_undervolt = i*num_cells + j;
        }


        return false;
      }
    }
  }

  // Check temperatures
  for (int i = 0; i < num_boards; i++) {
    for (int j = 0; j < 9; j++) {
      //if (cell_temp[i][j] > min_temp && cell_temp[i][j] < max_temp) {   temporarily changed for floating GPIOs
      if (cell_temp[i][j] < max_temp) {
        cell_temp[i][j] = min_temp;
        continue;
      } else {
        digitalWrite(20, LOW);
        delay(1000);  //delay to overcome debounce of shutdown circuit
        Serial.print("invalid temp Board:  ");
        Serial.print(i + 1);
        Serial.print("Num: ");
        Serial.println(j + 1);
        if(cell_temp[i][j] > max_temp){
          overtemp_fault;
          pos_overtemp = i*10 + j;
        }
        else{
          undertemp_fault;
          pos_undertemp = i*10 + j;
        }

        return false;
      }
    }
  }

  // Check for open fusible links
  if (max_cell_voltage - min_cell_voltage > max_diff){
      digitalWrite(20, LOW);
      delay(1000); // delay to overcome debounce of shutdown circuit
      fusible_link_fault = 1;
      Serial.println("Open fusible link detected - max voltage differential exceeded");
      return false;
  }

  // Check charger interlock
  // if (mode == "charge" && digitalRead(CHG_IN)){
  //   digitalWrite(20, LOW);
  //   delay(1000);
  //   Serial.println("Charger E-Stop Active");
  //   return false;
  // }


  digitalWrite(20, HIGH);
  wdt.feed();
  //Serial.println("Watchdog fed");
  return true;
}

void measure_current() {
  uint16_t ADC;
  float volt;
  digitalWrite(CS1, LOW);

  for (int i = 0; i < 2; i++) {
    SPI1.transfer(0b01100100);  //clock ADC
  }

  ADC = SPI1.transfer(0b00000000);
  ADC = ADC << 8;
  ADC = ADC | SPI1.transfer(0b00000000);
  volt = (float)(ADC) / 65535 * 5;
  current = (volt - 2.5) / .0267 - current_offset;  //this needs checked

  if (current > 50) {  //so does this
    ADC = SPI1.transfer(0b00000000);
    ADC = ADC << 8;
    ADC = ADC | SPI1.transfer(0b00000000);
    volt = (float)(ADC) / 65535 * 5;
    current = (volt - 2.5) / .004 - current_offset;  //this needs checked
  }

  if(1){
    Serial.print("Hall Effect ADC Voltage: "); Serial.println(volt);
    Serial.print("current: "); Serial.println(current);
  }
  current_count = current_count + 1;
  digitalWrite(CS1, HIGH);
}


void charger_enable(bool enable) {
  uint16_t chg_current;
  if (soc < 80)
        chg_current = CHG_current1;
    else if (soc < 90)
        chg_current = CHG_current2;
    else
        chg_current = CHG_current3; 

  digitalWrite(STBY, LOW);
  digitalWrite(CTX3, HIGH);
  delay(1);
  CAN_message_t CHGR_EN;
  //CHGR_EN.id = 0x1806E5F4;  // Set the CAN message ID     //datasheet
  CHGR_EN.id = 0x1806E5F4;  // Set the CAN message ID     //datasheet
  //CHGR_EN.id = 0x18FF50E5;  //charger send can id?
  CHGR_EN.flags.extended = 1;
  CHGR_EN.len = 8;  // Set the data length
  //7FF max CAN ID

  uint16_t voltage_int = (uint16_t)(CHG_voltage * 10);
  uint16_t current_int = (uint16_t)(chg_current * 10);


  CHGR_EN.buf[0] = (uint8_t)(voltage_int >> 8);  // High byte
  CHGR_EN.buf[1] = (uint8_t)(voltage_int);       // Low byte
  CHGR_EN.buf[2] = (uint8_t)(current_int >> 8);  // High byte
  CHGR_EN.buf[3] = (uint8_t)(current_int);       // Low byte
  CHGR_EN.buf[4] = (uint8_t)(enable);
  CHGR_EN.buf[5] = 0;
  CHGR_EN.buf[6] = 0;
  CHGR_EN.buf[7] = 0;

  bool message_sent = can.write(CHGR_EN);
  for (int i = 0; i < CHGR_EN.len; i++) {
    Serial.print(CHGR_EN.buf[i], BIN);
    Serial.print(" ");
  }
  digitalWrite(CTX3, LOW);

  if (!message_sent) {
  }
}

void TX_CAN() {
  measure_voltage();
  measure_temp();
  uint8_t inst_power_limit = power_limit(max_cell_temp);
  Serial.print("Power Limit: ");
  Serial.println(inst_power_limit);

  digitalWrite(STBY, LOW);
  digitalWrite(CTX3, HIGH);
  delay(1);

  CAN_message_t BMS_data;
  //BMS_data.id = BMS_ID;
  //BMS_data.id = BMS_ID;
  BMS_data.id = 0x00000007;
  BMS_data.flags.extended = 0;
  BMS_data.len = 8;  // Set the data length

  BMS_data.buf[0] = float_2_uint8_t(soc, 0, 100);                       // SOC
  BMS_data.buf[1] = float_2_uint8_t(currentbuffer_stat, 0, 250);        // current
  BMS_data.buf[2] = float_2_uint8_t(max_cell_voltage, 2.00, 4.50);      // max cell voltage
  BMS_data.buf[5] = float_2_uint8_t(max_cell_temp, 0, 75);              // max cell temp
  BMS_data.buf[4] = float_2_uint8_t(min_cell_voltage, 2.00, 4.50);      // min cell voltage
  BMS_data.buf[3] = float_2_uint8_t(min_cell_temp, 0, 75);              // min cell temp
  BMS_data.buf[6] = inst_power_limit;                                   // BMS Suggested Power Limit
  BMS_data.buf[7] = float_2_uint8_t(pack_voltage, 280, 600);            // Pack voltage

  if (can.write(BMS_data)) {
    Serial.println("CAN message sent 2");
  } else {
    Serial.println("CAN message TX Failed");
  }
  digitalWrite(CTX3, LOW);
}

uint8_t power_limit(float max_cell_temp) {
  if (max_cell_temp <= FULL_POWER_TEMP_C) {
    return FULL_POWER_LEVEL_KW;
  } else {
    return 4;
  }
  // else {
  //   float slope = -1.0 / (ZERO_POWER_TEMP_C - FULL_POWER_TEMP_C);
  //   return float_2_uint8_t(FULL_POWER_LEVEL_KW * (slope * (max_cell_temp - FULL_POWER_TEMP_C) + 1.0), 0, 80);
  // }
}

uint8_t float_2_uint8_t(float float_val, float min, float max) {  //float to uint8_t, clips values under/over min or max
  if (max == min) { return (0); }                                 //divide by zero case
  if (float_val >= max) {                                         //overflow case
    return max;
  }
  if (float_val <= min) {  //overflow case
    return min;
  }
  uint8_t scaled = (uint8_t)(((float_val - min) / (max - min)) * 255.0);  //float decoded = ((float)scaled / 255.0) * (max - min) + min;
  return (scaled);
}

CAN_message_t RX_CAN() {  //grabs the first message in the FIFO.
  static int curr_time = 0;

  //left bit in charger flag is highest bit (bit 4)
  CAN_message_t msg = {};
  digitalWrite(STBY, LOW);
  bool recieved = false;
  can.read(msg);
  //can.readMB(msg);
  if (msg.id != 0 && debug) {
    Serial.print("ID: ");
    Serial.print(msg.id, HEX);
    Serial.println(" Data: ");
    //msg.len = 8;
    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.print('\n');
  }
  return msg;  //always check the ID of the returned message. No messages in buffer returns 0 ID with 8 byte of zero data
}

void configure_sense() {
  uint8_t data[6];
  uint8_t data_arr[num_boards][6];  //contains identicle copies of data for each board
  uint16_t VUV;
  uint16_t VOV;
  VUV = UV / (16 * 0.0001) - 1;  //Comparison Voltage = (VUV + 1) • 16 • 100μV  (pg. 68 in datasheet)
  VOV = OV / (16 * 0.0001);      //Comparison Voltage = VOV • 16 • 100μV        (pg. 68 in datasheet)

  Serial.println(VUV, BIN);
  Serial.println(VOV, BIN);

  data[0] = 0b11111100;  //GPIO1-5 = 1 (pull-down off), REFON=1, DTEN=0, ADCOPT=0
  data[1] = (uint8_t)VUV;
  data[2] = (uint8_t)(VOV & 0b11110000) | (VUV >> 8 & 0b00001111);
  data[3] = (uint8_t)VOV >> 4;
  data[4] = 0b00000000;
  data[5] = 0b00000000;

  // data[0] = 0b11111110;     //GPIO1-5 = 1 (pull-down off), REFON=1, DTEN=0, ADCOPT=0
  // data[1] = (uint8_t) VUV;
  // data[2] = (uint8_t) (VOV & 0b11110000) | (VUV>>8 & 0b00001111);
  // data[3] = (uint8_t) VOV>>4;
  // data[4] = 0b11111111;
  // data[5] = 0b11111111;

  for (int i = 0; i < num_boards; i++) {
    std::copy(data, data + 6, data_arr[i]);
  }
  write_register_group(WRCFGA, data_arr);
}

void update_faults(){
  // Check for "clear_faults" input
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer == "clear_faults") {
        Serial.println("Clearing faults");
        volt_sense_fault = 0;
        isoSPI_fault = 0;
        curr_sense_fault = 0;
        overvolt_fault = 0;
        undervolt_fault = 0;
        overtemp_fault = 0;
        undertemp_fault = 0;
        fusible_link_fault = 0;
        pos_volt_sense_fault = 0;
        pos_overvolt = 0;
        pos_undervolt = 0;
        pos_overtemp = 0;
        pos_undertemp = 0;
        pos_fusible_link_fault = 0;
        for(int i = 0; i < 7; i++){
          EEPROM.write(i, 0);
        }
        serialBuffer = "";
        return;
      }
      //Clear buffer even if input is not "clear_faults"
      serialBuffer = "";
    } 
    else {
      serialBuffer += c;
    }
  }

  // No clear_faults input, check / update stored faults
  uint8_t stored_faults = EEPROM.read(0);
  uint8_t current_faults =  
                    (fusible_link_fault << 7) |
                    (undertemp_fault << 6) | 
                    (overtemp_fault << 5) | 
                    (undervolt_fault << 4) | 
                    (overvolt_fault << 3) |
                    (curr_sense_fault << 2) |
                    (isoSPI_fault << 1) |
                    (volt_sense_fault << 0);

  // Check for new faults, do not clear stored faults
  if(stored_faults != (current_faults | stored_faults) ){
    EEPROM.write(0, (current_faults | stored_faults) );
  }

  // Store fault positions if none exist
  if( (EEPROM.read(1) == 0) && (pos_volt_sense_fault != 0)){
    EEPROM.write(1, pos_volt_sense_fault);
  }
  if( (EEPROM.read(2) == 0) && (pos_overvolt != 0)){
    EEPROM.write(2, pos_overvolt);
  }
  if( (EEPROM.read(3) == 0) && (pos_undervolt != 0)){
    EEPROM.write(3, pos_undervolt);
  }
  if( (EEPROM.read(4) == 0) && (pos_overtemp != 0)){
    EEPROM.write(4, pos_overtemp);
  }
  if( (EEPROM.read(5) == 0) && (pos_undertemp != 0)){
    EEPROM.write(5, pos_undertemp);
  }
  if( (EEPROM.read(6) == 0) && (pos_fusible_link_fault != 0)){
  EEPROM.write(6, pos_fusible_link_fault);
  }
}

void print_faults(){
  uint8_t stored_faults = EEPROM.read(0);
  if(!stored_faults){
    Serial.println("No Stored Faults");
    return;
  }
  
  if(stored_faults & 0b10000000){
    Serial.print("Fusible link fault, position: ");
    Serial.println(EEPROM.read(6));
  }
  if(stored_faults & 0b01000000){
    Serial.print("Undertemp fault, position: ");
    Serial.println(EEPROM.read(5));
  }
  if(stored_faults & 0b00100000){
    Serial.print("Overtemp fault, position: ");
    Serial.println(EEPROM.read(4));
  }
  if(stored_faults & 0b00010000){
    Serial.print("Undervolt fault, position: ");
    Serial.println(EEPROM.read(3));
  }
  if(stored_faults & 0b00001000){
    Serial.print("Overvolt fault, position: ");
    Serial.println(EEPROM.read(2));
  }
  if(stored_faults & 0b00000100){
    Serial.println("Current sense fault (stored)");
  }
  if(stored_faults & 0b00000010){
    Serial.println("IsoSPI fault (stored)");
  }
  if(stored_faults & 0b00000001){
    Serial.print("Voltage sense fault, position ");
    Serial.println(EEPROM.read(1));
  }

}

/*
void balance(bool keep_going) {
  bool discharge[num_boards][18] = { 0 };  //'1': needs dischaged, '0': does not need discharged
  float min = cell_voltage[0][0];
  float max = cell_voltage[0][0];

  sense_status();
  ////mark cells to be discharged////
  min_max<num_boards, num_cells>(cell_voltage, &min, &max);
  Serial.print("min cell voltage: ");
  Serial.println(min);
  Serial.print("max cell voltage: ");
  Serial.println(max);
  Serial.println("Cells to be discharged");
  if (keep_going) {
    for (int i = 0; i < num_boards; i++) {
      for (int j = 0; j < num_cells; j++) {
        discharge[i][j] = cell_voltage[i][j] > min && cell_voltage[i][j] > balance_threshold && die_temps[i] < 60.0f;
        if (discharge[i][j]) {
          Serial.print("Board: ");
          Serial.print(i + 1);
          Serial.print("  Cell: ");
          Serial.print(j + 1);
          Serial.print(" Volt: ");
          Serial.println(cell_voltage[i][j]);
          Serial.print("die temp: ");
          Serial.println(die_temps[i]);
        }
      }
    }
  }
  discharge_cells(discharge);
  if (balance_threshold < min) {
    balance_threshold = min;
  }
}
*/

void sense_status() {  //really should be the measure die temp function
  uint8_t response[num_boards][6];
  poll_ADC(ADSTAT);
  read_register_group(RDSTATA, response);
  Serial.println("die_temps");
  for (int i = 0; i < num_boards; i++) {
    die_temps[i] = (response[i][2] | response[i][3] << 8) * (0.0001 / .0076) - 276;
    Serial.println(die_temps[i]);
  }
  Serial.println();
}

// void sense_status(){
//   uint8_t response[num_boards][6];
//   read_register_group(RDSTATB , response);

//   undervoltage_flag[0] = response[2]>>0 & 0b1;
//   undervoltage_flag[1] = response[2]>>2 & 0b1;
//   undervoltage_flag[2] = response[2]>>4 & 0b1;
//   undervoltage_flag[3] = response[2]>>6 & 0b1;
//   undervoltage_flag[4] = response[3]>>0 & 0b1;
//   undervoltage_flag[5] = response[3]>>2 & 0b1;
//   undervoltage_flag[6] = 0;
//   undervoltage_flag[7] = 0;
//   undervoltage_flag[8] = 0;
//   undervoltage_flag[9] = 0;
//   undervoltage_flag[10] = 0;
//   undervoltage_flag[11] = 0;
//   undervoltage_flag[12] = 0;
//   undervoltage_flag[13] = 0;
//   undervoltage_flag[14] = 0;
//   undervoltage_flag[15] = 0;
//   Serial.println("voltage flags");
//   for(int i = 0; i<=5; i++){
//     Serial.println(undervoltage_flag[i]);
//   }
//   Serial.println("done");

// }

void flash_leds() {                        //Flashes each discharge resistor sequentially
  int time_on = 1000;                      //Time each led is on in milliseconds
  bool discharge[num_boards][18] = { 0 };  //'1': needs dischaged, '0': does not need discharged
  for (int i = num_boards; i >= 0; i--) {
    if (i % 4 < 2) {
      for (int j = 0; j < num_cells; j++) {
        discharge[i][j] = true;
        discharge_cells(discharge);
        delay(time_on);
        discharge[i][j] = false;
        discharge_cells(discharge);
      }
    } else {
      for (int j = num_cells; j >= 0; j--) {
        discharge[i][j] = true;
        discharge_cells(discharge);
        delay(time_on);
        discharge[i][j] = false;
        discharge_cells(discharge);
      }
    }
  }
}

void discharge_cells(bool discharge[num_boards][18]) {  //this function takes a 2D boolean array which is NOT dependent on num_cells.
  uint8_t data[6];
  uint8_t data_arr[num_boards][6];
  uint16_t VUV;
  uint16_t VOV;
  VUV = UV / (16 * 0.0001) - 1;  //Comparison Voltage = (VUV + 1) • 16 • 100μV  (pg. 68 in datasheet)
  VOV = OV / (16 * 0.0001);      //Comparison Voltage = VOV • 16 • 100μV        (pg. 68 in datasheet)
  ////configuration register group A////
  for (int i = 0; i < num_boards; i++) {
    data[0] = 0b11111100;  //GPIO1-5 = 1 (pull-down off), REFON=1, DTEN=0, ADCOPT=0
    data[1] = (uint8_t)VUV;
    data[2] = (uint8_t)(VOV & 0b11110000) | (VUV >> 8 & 0b00001111);
    data[3] = (uint8_t)VOV >> 4;
    data[4] = (uint8_t)discharge[i][7] << 7 | discharge[i][6] << 6 | discharge[i][5] << 5 | discharge[i][4] << 4 | discharge[i][3] << 3 | discharge[i][2] << 2 | discharge[i][1] << 1 | discharge[i][0] << 0;
    data[5] = (uint8_t)discharge[i][11] << 3 | discharge[i][10] << 2 | discharge[i][9] << 1 | discharge[i][8] << 0;
    std::copy(data, data + 6, data_arr[i]);
  }

  write_register_group(WRCFGA, data_arr);
  ////configuration register group B/////
  for (int i = 0; i < num_boards; i++) {
    data[0] = (uint8_t)discharge[i][15] << 7 | discharge[i][14] << 6 | discharge[i][13] << 5 | discharge[i][12] << 4 | 0b1111;
    data[1] = (uint8_t)discharge[i][17] | discharge[i][16];
    data[2] = (uint8_t)0b00000000;
    data[3] = (uint8_t)0b00000000;
    data[4] = (uint8_t)0b00000000;
    data[5] = (uint8_t)0b00000000;

    std::copy(data, data + 6, data_arr[i]);
  }
  write_register_group(WRCFGB, data_arr);
}

void myCallback() {
  Serial.println("Callback called");
  measure_voltage();
  measure_temp();
  reset_watchdog();
  watchdog_callback = true;  //set watchdog callback flag
}


//Analog Devices provided Functions//

void wakeup_sleep(uint8_t total_ic)  //Number of ICs in the system. This function needs some work. Enters Sleep state after 2 seconds of no command sent with valid PEC
{
  Serial.println("Wakeup Sleep");
  for (int i = 0; i < total_ic; i++) {
    digitalWrite(CS, LOW);
    //delayMicroseconds(300);
    delay(1);
    digitalWrite(CS, HIGH);
    //delayMicroseconds(10);
    delay(1);
  }
}

uint16_t pec15_calc(uint8_t len,   //Number of bytes that will be used to calculate a PEC
                    uint8_t *data  //Array of data that will be used to calculate  a PEC
) {
  uint16_t remainder, addr;
  remainder = 16;  //initialize the PEC

  for (uint8_t i = 0; i < len; i++)  // loops for each byte in data array
  {
    addr = ((remainder >> 7) ^ data[i]) & 0xff;  //calculate PEC table address
#ifdef MBED
    remainder = (remainder << 8) ^ crc15Table[addr];
#else
    remainder = (remainder << 8) ^ pgm_read_word_near(crc15Table + addr);
#endif
  }

  return (remainder * 2);  //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

void wakeup_idle(uint8_t total_ic) {        //idle after 4.3 ms of no isoSPI activity
                                            //Serial.println("wakeup_idle");
  for (int i = 0; i < total_ic + 1; i++) {  //+1 IC for the LTC6820
    digitalWrite(CS, LOW);
    SPI.transfer(0b11111111);  //Guarantees the isoSPI will be in ready mode
    digitalWrite(CS, HIGH);
  }
  delayMicroseconds(1);  //This delay is absolutely needed: t5 in datasheet - CSB Rising Edge to CSB Falling Edge >= 0.65us
}

uint16_t pec10_update_bit(uint16_t rem, uint8_t in_bit) {
    // CRC10 width=10, poly without x^10 term: x^7 + x^3 + x^2 + x + 1 => 0x08F
    const uint16_t poly = 0x008F;
    const uint16_t mask = 0x03FF;

    uint8_t fb = ((rem >> 9) & 1u) ^ (in_bit & 1u);  // feedback bit
    rem = (uint16_t)((rem << 1) & mask);

    if (fb) 
        rem ^= poly;

    return rem;
}

uint16_t pec10_calc_data_ccnt(const uint8_t *data6, uint8_t ccnt6) {
    uint16_t rem = 0x0010; // initial value = 0000010000 :contentReference[oaicite:2]{index=2}

    // 48 data bits, MSB-first
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t d = data6[i];
        for (int8_t b = 7; b >= 0; b--) {
            rem = pec10_update_bit(rem, (d >> b) & 1u);
        }
    }

    // 6 CCNT bits, MSB-first: CCNT[5]..CCNT[0] live in PEC0[7:2] :contentReference[oaicite:3]{index=3}
    for (int8_t b = 5; b >= 0; b--)
        rem = pec10_update_bit(rem, (ccnt6 >> b) & 1u);

    return (rem & 0x03FF);
}