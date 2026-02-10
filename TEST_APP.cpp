#include "src/Measurement/Measurement.h"

static ev4_t ctx{};

void setup() {
    // Open shutdown circuit
    pinMode(SC, OUTPUT);
    digitalWrite(SC, LOW);

    delay(5000); // startup delay should be use to make it easier to recover the teensy when runtime errors occurs

    Serial.begin(9600);
    println_with_args("Startup/n/tStart Time: %u", ctx.start_time);

    setup_timers(&ctx);
}

void loop() {
    measure_frequency(&ctx);
    println_with_args("Freq:\n\tPin 6: %f\tPin 9: %f", ctx.pcb_temp[0], ctx.pcb_temp[1]);
    delay(200);
}
