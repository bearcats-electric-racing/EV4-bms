#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <CONFIGURE.h>
#include <COMMANDS.h>
#include <src/Adc/Adc.h>
#include <src/Ev4/Ev4.h>

void measure_voltage(Ev4_t *ctx, bool open_wire_check = false);

#endif
