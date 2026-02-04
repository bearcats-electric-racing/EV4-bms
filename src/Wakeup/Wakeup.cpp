#include "Wakeup.h"

void wakeup_sleep(uint8_t total_ic)  //Number of ICs in the system. This function needs some work. Enters Sleep state after 2 seconds of no command sent with valid PEC
{
  Serial.println("Wakeup Sleep");
  for (int i = 0; i < total_ic; i++) {
    digitalWrite(CS, LOW);
    //delayMicroseconds(300);
    delay(1);
    digitalWrite(CS, HIGH);
    //delayMicroseconds(10);
    delay(1);
  }
}

void wakeup_idle(uint8_t total_ic) {        //idle after 4.3 ms of no isoSPI activity
                                            //Serial.println("wakeup_idle");
  for (int i = 0; i < total_ic + 1; i++) {  //+1 IC for the LTC6820
    digitalWrite(CS, LOW);
    SPI.transfer(0b11111111);  //Guarantees the isoSPI will be in ready mode
    digitalWrite(CS, HIGH);
  }
  delayMicroseconds(1);  //This delay is absolutely needed: t5 in datasheet - CSB Rising Edge to CSB Falling Edge >= 0.65us
}