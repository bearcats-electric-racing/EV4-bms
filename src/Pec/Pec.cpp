#include "Pec.h"

// Computes PEC10 over 6 data bytes + 6 CCNT bits (total 54 bits), MSB-first
uint16_t pec10_calc_data_ccnt(const uint8_t *data6, uint8_t ccnt6)
{
  uint16_t rem = 0x0010; // initial value = 0000010000 :contentReference[oaicite:2]{index=2}
  
  // 48 data bits, MSB-first
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t d = data6[i];
    for (int8_t b = 7; b >= 0; b--) {
      rem = pec10_update_bit(rem, (d >> b) & 1u);
    }
  }
  
  // 6 CCNT bits, MSB-first: CCNT[5]..CCNT[0] live in PEC0[7:2] :contentReference[oaicite:3]{index=3}
  for (int8_t b = 5; b >= 0; b--) {
    rem = pec10_update_bit(rem, (ccnt6 >> b) & 1u);
  }
  
  return (rem & 0x03FF);
}

// Data PEC Calculation from ChatGPT 5.2 - Analog did not provide reference code for the ADBMS6830B
uint16_t pec10_update_bit(uint16_t rem, uint8_t in_bit)
{
  // CRC10 width=10, poly without x^10 term: x^7 + x^3 + x^2 + x + 1 => 0x08F
  const uint16_t poly = 0x008F;
  const uint16_t mask = 0x03FF;
  
  uint8_t fb = ((rem >> 9) & 1u) ^ (in_bit & 1u);  // feedback bit
  rem = (uint16_t)((rem << 1) & mask);
  if (fb) rem ^= poly;
  return rem;
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