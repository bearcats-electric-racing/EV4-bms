#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "../System/System.h"
#include "../Utils/Utils.h"
#include "../Ev4/Ev4.h"
#include "../Measurement/Measurement.h"

void watchdog_init(ev4_t *ctx);
void watchdog_callback(ev4_t *ctx);
void watchdog_callback_wrapper();
bool watchdog_reset(ev4_t *ctx);

#endif