#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "../Utils/Utils.h"
#include "../Adc/Adc.h"
#include "../System/System.h"
#include "../Ev4/Ev4.h"

void measure_voltage(ev4_t *ctx); // 18 millisecond execution time
void measure_cell_temp(ev4_t *ctx, bool open_wire_check = false);
void measure_frequency(ev4_t *ctx);
void measure_pcb_temp(ev4_t *ctx);
void measure_current(ev4_t *ctx);
void measure_die_temp(ev4_t *ctx);

#endif
