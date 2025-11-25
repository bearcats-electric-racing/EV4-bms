////////////////////////////////////////////
/////// PEC TESTING SANDBOX ////////////////
////////////////////////////////////////////

#include <SPI.h>
#include <cmath>
#include <string>
#include "COMMANDS.h"
#include "CONFIGURE.h"
#include "LUTs.h"

#define CS 10   //chip select pin isoSPI

//LTC6813 minimum supply voltage is 16V
float cell_voltage[num_boards][num_cells];  //most recent cell voltages
float open_circuit_voltage[num_boards][num_cells];
float pack_voltage = 0;          //sum of cell voltages
float cell_temp[num_boards][9];  //most recent cell temperatures. Contans raw voltage data for the duration of open wire checks
float die_temps[num_boards];     //most recent sense board LTC6813 die temps

void measure_voltage();

void setup(){
  Serial.begin(9600);
  Serial.println("startup");

  //SPI (isoSPI)
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

}

void loop(){
  while(1){
    measure_voltage();
    delay(500);
  }

}

void measure_voltage() {  //18 millisecond execution time
  uint8_t response[num_boards][6];
  uint16_t cell_comm[6] = { RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF };  //read cell voltage registers A through E commands

  ////cell voltage measurement algorithm outlined in INTERNAL PROTECTION AND FILTERING section of LTC6813 datasheet////
  //poll_ADC(ADCV | 0b1);   //measure cells 1,7,13 to allow MUX voltage to settle
  //delay(cell_RC * 6);

  poll_ADC(ADCV, 0);  //initiate and wait for voltage measurement

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
                                                                                                                     //Serial.print('k');
                                                                                                                     //Serial.println(k);
        cell_voltage[j][i * 3 + k] = (float)(((uint8_t)response[j][k * 2 + 1] << 8) | response[j][k * 2]) * 0.00015f + 1.5f;  //LSB represents 150 uV, +1.5v offset
        pack_voltage = pack_voltage + cell_voltage[j][i * 3 + k];
      }
    }
  }

  /*
  if (current < 0.2 and current > -0.2) {
    for (int i = 0; i < num_boards; ++i) {
      for (int j = 0; j < num_cells; ++j) {
        open_circuit_voltage[i][j] = cell_voltage[i][j];
      }
    }
  }
  

  new_voltage = true;

  */

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

void read_register_group(uint16_t command, uint8_t response[num_boards][6]) {  //register group is always 6 bytes

  uint16_t pec;
  uint8_t pec0;
  uint8_t pec1;
  uint8_t response_pec0;
  uint8_t response_pec1;

  send_command(command);

  for (int i = 0; i < num_boards; i++) {
    for (int j = 0; j < 6; j++) {
      response[i][j] = SPI.transfer(0b11111111);  // Send dummy byte to receive data
      //Serial.println(response[i][j], BIN);
    }
    response_pec0 = SPI.transfer(0xFF);
    response_pec1 = SPI.transfer(0xFF);
    pec = pec15_calc(6, response[i]);
    pec1 = pec >> 0;
    pec0 = pec >> 8;

    if (response_pec0 != pec0 || response_pec1 != pec1) {  //this recursion needs fixed
      Serial.println("pec error");
      wakeup_sleep(num_boards + 1);
      //read_register_group(command, response);
    }
  }

  pec = pec15_calc(6, response[0]);  //this needs fixed to include multiple boards

  Serial.println("response pec");
  Serial.println(response_pec0, HEX);
  Serial.println(response_pec1, HEX);
  Serial.println("calculated pec");
  Serial.println(pec, HEX);

  digitalWrite(CS, HIGH);
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

/*
  if (millis() - sense_watchdog_timer >= 1800) {
    wakeup_sleep(num_boards + 1);
    sense_watchdog_timer = millis();
  } else {
    wakeup_idle(num_boards);
    sense_watchdog_timer = millis();
  }
*/

  wakeup_sleep(num_boards + 1); //Temporary wakeup, no watchdog, no idle wakeup logic

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

void poll_ADC(uint16_t command, bool curr_measure) {
  uint8_t return_data = 0;
  send_command(command);

/*
  if (!curr_sense_fault && curr_measure) {
    measure_current();
  }
*/

  int num_polls = 0;
  while (return_data == 0) {                 //This needs a timeout condition
    return_data = SPI.transfer(0b11111111);  // Send dummy byte to receive data
    num_polls++;
  }
  Serial.println("ADC Conversion Done!");
  // Serial.println(num_polls);

  digitalWrite(CS, HIGH);
}


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

void wakeup_idle(uint8_t total_ic) {        //idle after 4.3 ms of no isoSPI activity
                                            //Serial.println("wakeup_idle");
  for (int i = 0; i < total_ic + 1; i++) {  //+1 IC for the LTC6820
    digitalWrite(CS, LOW);
    SPI.transfer(0b11111111);  //Guarantees the isoSPI will be in ready mode
    digitalWrite(CS, HIGH);
  }
  delayMicroseconds(1);  //This delay is absolutely needed: t5 in datasheet - CSB Rising Edge to CSB Falling Edge >= 0.65us
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
