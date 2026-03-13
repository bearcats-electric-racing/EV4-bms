#ifndef STANDBY_H
#define STANDBY_H

#include "../Can/Can.h"
#include "../Watchdog/Watchdog.h"
#include "../Ev4/Ev4.h"
#include "../Measurement/Measurement.h"

void standby_state(ev4_t *ctx);

#endif