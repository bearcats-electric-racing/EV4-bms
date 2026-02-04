////////////////////////////////////////////
/////// PEC TESTING SANDBOX ////////////////
////////////////////////////////////////////

#include <SPI.h>
#include <cmath>
#include <string>

#include "COMMANDS.h"
#include "CONFIGURE.h"
#include "LUTS.h"

#include "src/Pec/Pec.h"
#include "src/Utils/Utils.h"
#include "src/Measurement/Measurement.h"
#include "src/Spi/Spi.h"
#include "src/Ev4/Ev4.h"

static Ev4_t ctx{};

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
    measure_voltage(&ctx);
    delay(500);
  }
}
