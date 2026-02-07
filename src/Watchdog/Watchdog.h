#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "../System/System.h"
#include "../Utils/Utils.h"
#include "../Ev4/Ev4.h"
#include "../Measurement/Measurement.h"

void watchdog_init(ev4_t *ctx);
void watchdog_isr(ev4_t *ctx);
void watchdog_callback();
bool watchdog_reset(ev4_t *ctx);

#endif