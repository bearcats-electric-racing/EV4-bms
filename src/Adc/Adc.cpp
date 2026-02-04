#include "Adc.h"

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