#include "Measurement.h"

void measure_voltage(Ev4_t *ctx, bool open_wire_check) {  //18 millisecond execution time
  uint8_t response[num_boards][6];
  uint16_t cell_comm[6] = { RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF };  //read cell voltage registers A through E commands

  ////cell voltage measurement algorithm outlined in INTERNAL PROTECTION AND FILTERING section of LTC6813 datasheet////
  //poll_ADC(ADCV | 0b1);   //measure cells 1,7,13 to allow MUX voltage to settle
  //delay(cell_RC * 6);

  poll_ADC(ADCV, 0);  //initiate and wait for voltage measurement

  ctx->pack_voltage = 0;
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
        ctx->cell_voltage[j][i * 3 + k] = (float)(((uint8_t)response[j][k * 2 + 1] << 8) | response[j][k * 2]) * 0.00015f + 1.5f;  //LSB represents 150 uV, +1.5v offset
        ctx->pack_voltage = ctx->pack_voltage + ctx->cell_voltage[j][i * 3 + k];
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

  if (ctx->cfg.debug) {
    Serial.println("Voltages:");
    int g = 0;
    for (int i = 0; i < num_boards; i++) {
      Serial.print("board: ");
      Serial.println(i + 1);
      for (int j = 0; j < num_cells; j++) {
        Serial.print(ctx->cell_voltage[i][j]);
        Serial.print(" ");
        g++;
      }
      Serial.println("");
    }
  }
}