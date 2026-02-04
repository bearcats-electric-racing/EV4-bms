#ifndef WAKEUP_H
#define WAKEUP_H

#include <core_pins.h>
#include <SPI.h>
#include <src/Spi/Spi.h>

void wakeup_sleep(uint8_t total_ic);
void wakeup_idle(uint8_t total_ic);

#endif