# 1 "C:\\Users\\skyla\\AppData\\Local\\Temp\\tmp6pwtyvcv"
#include <Arduino.h>
# 1 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
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


#define CRX3 23
#define CTX3 22
#define STBY 21


const int chipSelect = BUILTIN_SDCARD;


#define CS 10
#define CS2 38
#define CS1 0


#define CHG_OUT 35
#define CHG_IN 34


unsigned int start_time = millis();
unsigned int sense_watchdog_timer;
bool new_voltage = false;
bool new_temp = false;


bool volt_sense_fault = 0;
bool isoSPI_fault = 0;
bool curr_sense_fault = 0;
bool overvolt_fault = 0;
bool undervolt_fault = 0;
bool overtemp_fault = 0;
bool undertemp_fault = 0;
bool fusible_link_fault = 0;

bool memory_fault = 0;
String serialBuffer = "";


uint8_t pos_volt_sense_fault = 0;
uint8_t pos_overvolt = 0;
uint8_t pos_undervolt = 0;
uint8_t pos_overtemp = 0;
uint8_t pos_undertemp = 0;
uint8_t pos_fusible_link_fault = 0;

int wire_cut = 0;
bool watchdog_callback = 0;
bool watchdog_reset = 0;
bool charger_fault = 0;

bool CHG_EN = 0;
# 87 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

WDT_T4<WDT1> wdt;


float current = 0;

float current_offset = 0;


float soc = 0.0;


int data_file_num = 0;


float inv_voltage = 0;


float cell_voltage[num_boards][num_cells];
float open_circuit_voltage[num_boards][num_cells];
float pack_voltage = 0;
float cell_temp[num_boards][10];
float die_temps[num_boards];
float min_cell_voltage = 0;
float max_cell_voltage = 0;
float min_cell_temp = 0;
float max_cell_temp = 0;
float min_die_temp = 0;
float max_die_temp = 0;


float GPIO_open_wire[num_boards][9];
bool overvoltage_flag[18];
bool undervoltage_flag[18];


unsigned int time_buffer[SD_interval];
float voltage_buffer[SD_interval / volt_interval][num_boards][num_cells];
float temp_buffer[SD_interval / temp_interval][num_boards][9];
float current_buffer[SD_interval / current_interval];
float currentbuffer_stat = 0;

long current_count = 0;
long current_sum = 0;
int RMS_Current = 0;
void setup();
void loop();
void initialize_ADC();
void read_ADC();
void print_min_max();
void dumpDataToSerial();
void check_memory();
void get_SOC();
void map_text2var(String name, String value);
float update_SOC();
void upadate_current_limit();
void SD_data_write();
void send_command(uint16_t command);
void read_register_group(uint16_t command, uint8_t response[num_boards][6]);
void write_register_group(uint16_t command, uint8_t data[num_boards][6]);
void poll_ADC(uint16_t command, bool curr_measure);
void measure_voltage();
float map_temp(float V);
void measure_temp(bool open_wire_check);
void cell_open_wire_check();
bool reset_watchdog();
void measure_current();
void charger_enable(bool enable);
void TX_CAN();
uint8_t power_limit(float max_cell_temp);
uint8_t float_2_uint8_t(float float_val, float min, float max);
CAN_message_t RX_CAN();
void configure_sense();
void update_faults();
void print_faults();
void sense_status();
void flash_leds();
void discharge_cells(bool discharge[num_boards][18]);
void myCallback();
void wakeup_sleep(uint8_t total_ic);
uint16_t pec15_calc(uint8_t len,
                    uint8_t *data
);
void wakeup_idle(uint8_t total_ic);
uint16_t pec10_update_bit(uint16_t rem, uint8_t in_bit);
uint16_t pec10_calc_data_ccnt(const uint8_t *data6, uint8_t ccnt6);
#line 135 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
void setup() {

  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);



  pinMode(CHG_OUT, OUTPUT);
  pinMode(CHG_IN, INPUT);
  digitalWrite(CHG_OUT, HIGH);

  delay(5000);


  sense_watchdog_timer = start_time - 5000;

  Serial.begin(9600);
  Serial.println("startup");
  Serial.print("Start Time: ");
  Serial.println(start_time);


  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));


  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
# 189 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
  pinMode(CRX3, INPUT);
  pinMode(CTX3, OUTPUT);
  pinMode(STBY, OUTPUT);
  can.begin();
  can.setBaudRate(250000);
  can.enableFIFO();


  if (watchdog_timeout != 0) {
    WDT_timings_t config;
    int watchdog_trigger = watchdog_timeout - 1;
    if (watchdog_trigger < 1) {
      watchdog_trigger = 1;
    }
    config.trigger = 11;
    config.timeout = watchdog_timeout;
    config.pin = 20;
    config.callback = myCallback;
    wdt.begin(config);
  }


  initialize_ADC();


  measure_current();
  current_offset = current;

  check_memory();



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
      TX_CAN();
      print_min_max();
      reset_watchdog();


      msg = RX_CAN();


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


      if (msg.id == INV_TX_ID && false) {
        mode = "standy";
        can.setBaudRate(500000);

        break;
      } else if (msg.id == CHG_TX_ID) {
        mode = "charge";
        can.setBaudRate(250000);

        break;
      } else if (current >= 0.5) {
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
    String filename = "data" + String(data_file_num) + ".csv";
    File file = SD.open(filename.c_str(), FILE_WRITE);
    file.close();

    delay(6000);

    while (1) {
      measure_voltage();
      measure_temp();
      reset_watchdog();
      charger_enable(true);
      msg = RX_CAN();
      charger_voltage = ((uint16_t)msg.buf[0] << 8 | (uint16_t)msg.buf[1]) / 10;
      charger_current = ((uint16_t)msg.buf[2] << 8 | (uint16_t)msg.buf[3]) / 10;
      Serial.println(charger_voltage);
      Serial.println(pack_voltage);

      break;

    }

    delay(1000);

    unsigned int charge_start_time = millis();
    while (1) {
      Serial.print("Time (minutes): "); Serial.println((float)(millis() - charge_start_time)/60000);
      Serial.print("charge fault status: ");
      Serial.println(msg.buf[4]);
      measure_voltage();
      measure_temp();
      measure_current();
      update_SOC();
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
        } else {
          digitalWrite(20, LOW);
          charger_enable(true);
          Serial.println("Charger Error");
          charger_fault = 1;
        }
      }
      delay(200);
    }


    while (1) {
      Serial.println("Charger Fault");
      delay(100);
    }
  }

  if (mode == "standby") {
    Serial.println("Standby Mode Entered");

    if (!memory_fault) {
      String filename = "data" + String(data_file_num) + ".csv";
      File file = SD.open(filename.c_str(), FILE_WRITE);
      file.close();
      SD_data_write();
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
      if (inv_voltage >= pack_voltage * 0.8) {
        mode = "drive";
        break;
      }
      delay(10);
    }
  }

  else if (mode == "drive") {
    Serial.println("Drive Mode Entered");
    int n = 0;

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
# 437 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
      while (millis() - start_time <= time_buffer[n] + time_step) {
      }
      if (n < SD_interval - 1) {
        n++;
      } else {
        n = 0;
      }

    }

  }

  else {
    digitalWrite(20, LOW);
    Serial.println("Debug Mode Entered");
    while (1)
      dumpDataToSerial();
  }
}

void initialize_ADC() {






  uint8_t CFR_reg_MSB;
  uint8_t CFR_reg_LSB;
  uint8_t CFR_readback_MSB;
  uint8_t CFR_readback_LSB;
  CFR_reg_MSB = 0b10000100;

  CFR_reg_LSB = 0b00000000;


  digitalWrite(CS1, LOW);
  SPI1.transfer(CFR_reg_MSB);
  SPI1.transfer(CFR_reg_LSB);
  for (int i = 0; i < 6; i++) {
    SPI1.transfer(0b00000000);
  }
  digitalWrite(CS1, HIGH);
  delay(2);


  digitalWrite(CS1, LOW);
  SPI1.transfer(0b00110000);
  for (int i = 0; i < 6; i++) {
    SPI1.transfer(0b00000000);
  }
  digitalWrite(CS1, HIGH);
  delay(2);


  digitalWrite(CS1, LOW);
  CFR_readback_MSB = SPI1.transfer(0b00000000);
  CFR_readback_LSB = SPI1.transfer(0b00000000);
  for (int i = 0; i < 6; i++) {
    SPI1.transfer(0b00000000);
  }
  digitalWrite(CS1, HIGH);


  if ((uint8_t)(CFR_reg_MSB << 4) != (uint8_t)(CFR_readback_MSB << 4) || (uint8_t)(CFR_reg_LSB << 4) != (uint8_t)(CFR_readback_LSB << 4)) {
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
    temp = SPI1.transfer(0b00000000);
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


void print_min_max() {
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
    if (file_num >= file_read_begin) {
      Serial.println(entry.name());
      while (entry.available()) {
        char character = entry.read();
        Serial.write(character);

        while (character == '\n') {
          String input = Serial.readStringUntil('\n');
          input.trim();
          if (input == "next line") {
            break;
          }
        }
      }
      Serial.println("done");

      while (1) {
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

void check_memory() {
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

void get_SOC() {

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

void map_text2var(String name, String value) {
  if (name == "SOC:") {
    soc = value.toFloat();
    Serial.println(soc);
  }
}

float update_SOC() {

    const int discharge_curve_length = sizeof(discharge_points) / sizeof(discharge_points[0]);
    const float max_capacity = discharge_points[0];
    const int num_current_curves = sizeof(discharge_currents) / sizeof(discharge_currents[0]);

    float min_OC_cell_voltage = open_circuit_voltage[0][0];
    float max_OC_cell_voltage = open_circuit_voltage[0][0];
    min_max<num_boards, num_cells>(open_circuit_voltage, &min_OC_cell_voltage, &max_OC_cell_voltage);
    float discharged = interpolate<discharge_curve_length>(discharge_curves[0], discharge_points, min_OC_cell_voltage);
    soc = 100 - ((max_capacity - discharged) / max_capacity * 100);

  Serial.print("SOC: ");
  Serial.println(soc);
  return soc;
}

void upadate_current_limit() {
  const int discharge_curve_length = sizeof(discharge_points) / sizeof(discharge_points[0]);
  const float max_capacity = discharge_points[0];
  const int num_current_curves = sizeof(discharge_currents) / sizeof(discharge_currents[0]);
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
      dataFile.print("\nTime:");


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

void read_register_group(uint16_t command, uint8_t response[num_boards][6]) {

  uint8_t ccmd;
  uint16_t rx_pec10;
  uint16_t calc_pec10;
  uint8_t response_pec0;
  uint8_t response_pec1;

  send_command(command);

  for (int i = 0; i < num_boards; i++) {
      for (int j = 0; j < 6; j++) {
          response[i][j] = SPI.transfer(0b11111111);

      }

      response_pec0 = SPI.transfer(0xFF);
      response_pec1 = SPI.transfer(0xFF);



      ccmd = (response_pec0 >> 2) & 0x3F;
      rx_pec10 = ((uint16_t)(response_pec0 & 0x03) << 8) | response_pec1;

      calc_pec10 = pec10_calc_data_ccnt(response[i], ccmd);
# 839 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
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
    return_data = SPI.transfer(0b11111111);
    num_polls++;
    if(millis() - adc_poll_start_time > 1000){
        Serial.println("ADBMS6830B ADC TImeout Error");
        break;
    }
}




  digitalWrite(CS, HIGH);
}

void measure_voltage() {
  uint8_t response[num_boards][6];
  uint16_t cell_comm[6] = { RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF };





  poll_ADC(ADCV);

  pack_voltage = 0;
  for (int i = 0; i * 3 < num_cells; i++) {


    uint16_t curr_comm = cell_comm[i];
    read_register_group(curr_comm, response);
    for (int j = 0; j < num_boards; j++) {


      for (int k = 0; k < 3 && i * 3 + k < num_cells; k++) {
        int16_t adc_code = (int16_t)(((uint16_t)response[j][k * 2 + 1] << 8) | response[j][k * 2]);
        cell_voltage[j][i * 3 + k] = (float)adc_code * 0.00015f + 1.5f;
        pack_voltage += cell_voltage[j][i * 3 + k];
      }
    }
  }


  if (current < 0.2 and current > -0.2) {
    for (int i = 0; i < num_boards; ++i) {
      for (int j = 0; j < num_cells; ++j) {
        open_circuit_voltage[i][j] = cell_voltage[i][j];
      }
    }
  }


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

    if (V_ref == V)
    return -55;

    float NTC_res = (V / V_ref * R_bias) / (1 - V / V_ref);
    int i = search<size>(NTC_LUT, NTC_res);
    float temperature = float(i) / float(size) * (150 + 55) - 55;
    return (temperature);
}

void measure_temp(bool open_wire_check) {

    uint8_t response[num_boards][6];
    uint16_t aux_comm[4] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD};
    int thermistor_idx = 0;
    int command_idx = 0;

    poll_ADC(ADAX);

    while (thermistor_idx < 10) {
        uint16_t curr_comm = aux_comm[command_idx];


        read_register_group(curr_comm, response);
        for (int reading = 0; reading < 3 && thermistor_idx < 10; reading++) {
            if (command_idx == 3 && reading > 0)
                continue;

            for (int b = 0; b < num_boards; b++) {
                int16_t adc_code = (int16_t)(((uint16_t)response[b][reading * 2 + 1] << 8) | response[b][reading * 2]);






                cell_temp[b][thermistor_idx] = (float)adc_code * 0.00015f + 1.5f;
            }
            thermistor_idx++;
        }
        command_idx++;
    }


    for (int i = 0; i < num_boards; i++)
        for (int j = 0; j < 10; j++)
            cell_temp[i][j] = map_temp(cell_temp[i][j]);
# 1040 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
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


void cell_open_wire_check() {
    uint8_t response[num_boards][6];
    float s_voltage_open[num_boards][num_cells];
    float s_voltage_closed[num_boards][num_cells];
    bool open_wire_flags[num_boards][num_cells] = {false};
    bool open_wire = 0;

    uint16_t cell_comm[6] = {
        RDSVA, RDSVB, RDSVC,
        RDSVD, RDSVE, RDSVF
    };
# 1092 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
    for (uint8_t i = 0; i < 3; i++){
        if(i == 0){
            poll_ADC(ADSV);
        }
        else if(i == 1){
            poll_ADC(ADSV | OW_EVEN);
        }
        else if(i == 2){
            poll_ADC(ADSV | OW_ODD);
        }

        for (int j = 0; j * 3 < num_cells; j++) {


            uint16_t curr_comm = cell_comm[j];

            read_register_group(curr_comm, response);
            for (int k = 0; k < num_boards; k++) {


                for (int l = 0; l < 3 && j * 3 + l < num_cells; l++) {


                    int16_t adc_code = (int16_t)(((uint16_t)response[k][l * 2 + 1] << 8) | response[k][l * 2]);


                    if(i == 0){
                        s_voltage_open[k][j * 3 + l] = (float)adc_code * 0.00015f + 1.5f;
                    }

                    else if(i == 1){
                        if((j * 3 + l) % 2){
                            s_voltage_closed[k][j * 3 + l] = (float)adc_code * 0.00015f + 1.5f;
                        }
                    }

                    else if(i == 2){
                        if( !((j * 3 + l) % 2)) {
                            s_voltage_closed[k][j * 3 + l] = (float)adc_code * 0.00015f + 1.5f;
                        }
                    }
                }
            }
        }
    }


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
        delay(1000);
        wire_cut = 1;
        Serial.println("Open voltage sense lead detected");
    }


}

bool reset_watchdog() {
  new_voltage = false;
  new_temp = false;


  for (int i = 0; i < num_boards; i++) {
    for (int j = 0; j < num_cells; j++) {
      if (cell_voltage[i][j] < OV && cell_voltage[i][j] > UV) {
        cell_voltage[i][j] = 0;
        continue;
      } else {
        digitalWrite(20, LOW);
        delay(1000);
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


  for (int i = 0; i < num_boards; i++) {
    for (int j = 0; j < 9; j++) {

      if (cell_temp[i][j] < max_temp) {
        cell_temp[i][j] = min_temp;
        continue;
      } else {
        digitalWrite(20, LOW);
        delay(1000);
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


  if (max_cell_voltage - min_cell_voltage > max_diff){
      digitalWrite(20, LOW);
      delay(1000);
      fusible_link_fault = 1;
      Serial.println("Open fusible link detected - max voltage differential exceeded");
      return false;
  }


  if ((mode == "charge" || true) && digitalRead(CHG_IN)){
    charger_enable(true);
    digitalWrite(20, LOW);
    delay(1000);
    Serial.println("\nCharger E-Stop Active");
    return false;
  }


  digitalWrite(20, HIGH);
  wdt.feed();

  return true;
}

void measure_current() {
  uint16_t ADC;
  float volt;
  digitalWrite(CS1, LOW);

  for (int i = 0; i < 2; i++) {
    SPI1.transfer(0b01100100);
  }

  ADC = SPI1.transfer(0b00000000);
  ADC = ADC << 8;
  ADC = ADC | SPI1.transfer(0b00000000);
  volt = (float)(ADC) / 65535 * 5;
  current = (volt - 2.5) / .0267 - current_offset;

  if (current > 50) {
    ADC = SPI1.transfer(0b00000000);
    ADC = ADC << 8;
    ADC = ADC | SPI1.transfer(0b00000000);
    volt = (float)(ADC) / 65535 * 5;
    current = (volt - 2.5) / .004 - current_offset;
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

  CHGR_EN.id = 0x1806E5F4;

  CHGR_EN.flags.extended = 1;
  CHGR_EN.len = 8;


  uint16_t voltage_int = (uint16_t)(CHG_voltage * 10);
  uint16_t current_int = (uint16_t)(chg_current * 10);


  CHGR_EN.buf[0] = (uint8_t)(voltage_int >> 8);
  CHGR_EN.buf[1] = (uint8_t)(voltage_int);
  CHGR_EN.buf[2] = (uint8_t)(current_int >> 8);
  CHGR_EN.buf[3] = (uint8_t)(current_int);
  CHGR_EN.buf[4] = (uint8_t)(enable);
  CHGR_EN.buf[5] = 0;
  CHGR_EN.buf[6] = 0;
  CHGR_EN.buf[7] = 0;

  bool message_sent = can.write(CHGR_EN);
  Serial.print("Charger CAN Message: ");
  for (int i = 0; i < CHGR_EN.len; i++) {
    Serial.print(CHGR_EN.buf[i], HEX);
    Serial.print(" ");
  }
  digitalWrite(CTX3, LOW);

  if (!message_sent) {
    Serial.println("\nFailed to send charger CAN message");
  }
  else{
    Serial.println("\nCharger CAN message sent");
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


  BMS_data.id = 0x00000007;
  BMS_data.flags.extended = 0;
  BMS_data.len = 8;

  BMS_data.buf[0] = float_2_uint8_t(soc, 0, 100);
  BMS_data.buf[1] = float_2_uint8_t(currentbuffer_stat, 0, 250);
  BMS_data.buf[2] = float_2_uint8_t(max_cell_voltage, 2.00, 4.50);
  BMS_data.buf[5] = float_2_uint8_t(max_cell_temp, 0, 75);
  BMS_data.buf[4] = float_2_uint8_t(min_cell_voltage, 2.00, 4.50);
  BMS_data.buf[3] = float_2_uint8_t(min_cell_temp, 0, 75);
  BMS_data.buf[6] = inst_power_limit;
  BMS_data.buf[7] = float_2_uint8_t(pack_voltage, 280, 600);

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




}

uint8_t float_2_uint8_t(float float_val, float min, float max) {
  if (max == min) { return (0); }
  if (float_val >= max) {
    return max;
  }
  if (float_val <= min) {
    return min;
  }
  uint8_t scaled = (uint8_t)(((float_val - min) / (max - min)) * 255.0);
  return (scaled);
}

CAN_message_t RX_CAN() {
  static int curr_time = 0;


  CAN_message_t msg = {};
  digitalWrite(STBY, LOW);
  bool recieved = false;
  can.read(msg);

  if (msg.id != 0 && debug) {
    Serial.print("ID: ");
    Serial.print(msg.id, HEX);
    Serial.println(" Data: ");

    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.print('\n');
  }
  return msg;
}

void configure_sense() {
  uint8_t data[6];
  uint8_t data_arr[num_boards][6];
  uint16_t VUV;
  uint16_t VOV;
  VUV = UV / (16 * 0.0001) - 1;
  VOV = OV / (16 * 0.0001);

  Serial.println(VUV, BIN);
  Serial.println(VOV, BIN);

  data[0] = 0b11111100;
  data[1] = (uint8_t)VUV;
  data[2] = (uint8_t)(VOV & 0b11110000) | (VUV >> 8 & 0b00001111);
  data[3] = (uint8_t)VOV >> 4;
  data[4] = 0b00000000;
  data[5] = 0b00000000;
# 1435 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
  for (int i = 0; i < num_boards; i++) {
    std::copy(data, data + 6, data_arr[i]);
  }
  write_register_group(WRCFGA, data_arr);
}

void update_faults(){

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

      serialBuffer = "";
    }
    else {
      serialBuffer += c;
    }
  }


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


  if(stored_faults != (current_faults | stored_faults) ){
    EEPROM.write(0, (current_faults | stored_faults) );
  }


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
# 1593 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
void sense_status() {
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
# 1633 "C:/Users/skyla/Documents/BCMS/EV4-bms/LTC6813.ino"
void flash_leds() {
  int time_on = 1000;
  bool discharge[num_boards][18] = { 0 };
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

void discharge_cells(bool discharge[num_boards][18]) {
  uint8_t data[6];
  uint8_t data_arr[num_boards][6];
  uint16_t VUV;
  uint16_t VOV;
  VUV = UV / (16 * 0.0001) - 1;
  VOV = OV / (16 * 0.0001);

  for (int i = 0; i < num_boards; i++) {
    data[0] = 0b11111100;
    data[1] = (uint8_t)VUV;
    data[2] = (uint8_t)(VOV & 0b11110000) | (VUV >> 8 & 0b00001111);
    data[3] = (uint8_t)VOV >> 4;
    data[4] = (uint8_t)discharge[i][7] << 7 | discharge[i][6] << 6 | discharge[i][5] << 5 | discharge[i][4] << 4 | discharge[i][3] << 3 | discharge[i][2] << 2 | discharge[i][1] << 1 | discharge[i][0] << 0;
    data[5] = (uint8_t)discharge[i][11] << 3 | discharge[i][10] << 2 | discharge[i][9] << 1 | discharge[i][8] << 0;
    std::copy(data, data + 6, data_arr[i]);
  }

  write_register_group(WRCFGA, data_arr);

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
  watchdog_callback = true;
}




void wakeup_sleep(uint8_t total_ic)
{
  Serial.println("Wakeup Sleep");
  for (int i = 0; i < total_ic; i++) {
    digitalWrite(CS, LOW);

    delay(1);
    digitalWrite(CS, HIGH);

    delay(1);
  }
}

uint16_t pec15_calc(uint8_t len,
                    uint8_t *data
) {
  uint16_t remainder, addr;
  remainder = 16;

  for (uint8_t i = 0; i < len; i++)
  {
    addr = ((remainder >> 7) ^ data[i]) & 0xff;
#ifdef MBED
    remainder = (remainder << 8) ^ crc15Table[addr];
#else
    remainder = (remainder << 8) ^ pgm_read_word_near(crc15Table + addr);
#endif
  }

  return (remainder * 2);
}

void wakeup_idle(uint8_t total_ic) {

  for (int i = 0; i < total_ic + 1; i++) {
    digitalWrite(CS, LOW);
    SPI.transfer(0b11111111);
    digitalWrite(CS, HIGH);
  }
  delayMicroseconds(1);
}

uint16_t pec10_update_bit(uint16_t rem, uint8_t in_bit) {

    const uint16_t poly = 0x008F;
    const uint16_t mask = 0x03FF;

    uint8_t fb = ((rem >> 9) & 1u) ^ (in_bit & 1u);
    rem = (uint16_t)((rem << 1) & mask);

    if (fb)
        rem ^= poly;

    return rem;
}

uint16_t pec10_calc_data_ccnt(const uint8_t *data6, uint8_t ccnt6) {
    uint16_t rem = 0x0010;


    for (uint8_t i = 0; i < 6; i++) {
        uint8_t d = data6[i];
        for (int8_t b = 7; b >= 0; b--) {
            rem = pec10_update_bit(rem, (d >> b) & 1u);
        }
    }


    for (int8_t b = 5; b >= 0; b--)
        rem = pec10_update_bit(rem, (ccnt6 >> b) & 1u);

    return (rem & 0x03FF);
}