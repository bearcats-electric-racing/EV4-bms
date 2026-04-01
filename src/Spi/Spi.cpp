#include "Spi.h"

void spi_init(uint8_t pin, uint8_t dataMode) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, dataMode));
}