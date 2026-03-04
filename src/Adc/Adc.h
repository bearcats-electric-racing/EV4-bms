#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "../Ev4/Ev4.h"
#include "../Utils/Utils.h"
#include "../System/System.h"

void adc_init(ev4_t *ctx);
void adc_read();
void adc_poll(ev4_t *ctx, uint16_t command);

#endif