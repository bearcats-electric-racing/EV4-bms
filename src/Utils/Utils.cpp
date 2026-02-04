#include "Utils.h"

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

void read_register_group(uint16_t command, uint8_t response[num_boards][6]) {  //register group is always 6 bytes

  uint8_t response_pec0;
  uint8_t response_pec1;

  uint8_t ccmd; //command counter
  uint16_t rx_pec10; //Recieved and parsed 10 bit data PEC
  uint16_t calc_pec10; //Calculated 10 bit data PEC

  send_command(command);

  for (int i = 0; i < num_boards; i++) {
    for (int j = 0; j < 6; j++) {
      response[i][j] = SPI.transfer(0b11111111);  // Send dummy byte to receive data
      //Serial.println(response[i][j], BIN);
    }
    response_pec0 = SPI.transfer(0xFF); //reponse PEC = command counter + PEC, needs to be parsed
    response_pec1 = SPI.transfer(0xFF);

    // Extract command counter and received 10-bit PEC from the ADBMS6830B readback format:
    // PEC0 = [CCNT5..0 | PEC9..8], PEC1 = [PEC7..0]
    ccmd = (response_pec0 >> 2) & 0x3F;
    rx_pec10 = ((uint16_t)(response_pec0 & 0x03) << 8) | response_pec1;

    calc_pec10 = pec10_calc_data_ccnt(response[i], ccmd);

    if (rx_pec10 != (calc_pec10 & 0x3FF)) {
      Serial.println("PEC Error - Data PEC Mismatch");
      wakeup_sleep(num_boards + 1);
    }

  }

  Serial.print("response pec ");
  Serial.println(rx_pec10, HEX);
  Serial.print("calculated pec ");
  Serial.println(calc_pec10, HEX);

  digitalWrite(CS, HIGH);
}