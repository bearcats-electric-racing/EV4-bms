#ifndef PEC_H
#define PEC_H

#include "../../LUTS.h"
#include <avr/pgmspace.h>

// Computes PEC10 over 6 data bytes + 6 CCNT bits (total 54 bits), MSB-first
uint16_t pec10_calc_data_ccnt(const uint8_t *data6, uint8_t ccnt6);

// Data PEC Calculation from ChatGPT 5.2 - Analog did not provide reference code for the ADBMS6830B
uint16_t pec10_update_bit(uint16_t rem, uint8_t in_bit);

uint16_t pec15_calc(uint8_t len,   //Number of bytes that will be used to calculate a PEC
                    uint8_t *data  //Array of data that will be used to calculate  a PEC
);

#endif