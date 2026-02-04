#ifndef ADC_H
#define ADC_H

#include <cstdint>
#include "../Utils/Utils.h"
#include <SPI.h>
#include <src/Spi/Spi.h>

void poll_ADC(uint16_t command, bool curr_measure = false);     //curr_measure selects whether a current measurement is taken while polling the ADC (for synchronous Current and voltage measurements to determine cell internal resistance)

#endif