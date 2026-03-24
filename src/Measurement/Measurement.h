#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "../Utils/Utils.h"
#include "../Adc/Adc.h"
#include "../System/System.h"
#include "../Ev4/Ev4.h"

void measure_voltage(ev4_t *ctx); // 18 millisecond execution time
void measure_temp(ev4_t *ctx);
void measure_current(ev4_t *ctx);
void measure_die_temp(ev4_t *ctx);
void cell_open_wire_check(ev4_t *ctx);
void temp_open_wire_check(ev4_t *ctx);

#endif
