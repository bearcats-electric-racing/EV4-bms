#ifndef WAKEUP_H
#define WAKEUP_H

#include <cstdint>
#include "../Spi/Spi.h"
#include "../System/System.h"

void wakeup_sleep(uint8_t total_ic); 
void wakeup_idle(uint8_t total_ic); // idle after 4.3 ms of no isoSPI activity

#endif