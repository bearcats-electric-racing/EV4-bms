#ifndef CHARGE_H
#define CHARGE_H

#include <FlexCAN_T4.h>
#include "../Can/Can.h"
#include "../Watchdog/Watchdog.h"
#include "../Sd/Sd.h"
#include "../Ev4/Ev4.h"
#include "../Measurement/Measurement.h"
#include "../Soc/Soc.h"

const uint16_t charger_current_from_soc(ev4_t *ctx);
void configure_charger(ev4_t *ctx, bool enable, uint16_t charger_current = 0);
void charge_precharge(ev4_t *ctx);
void charge_state(ev4_t *ctx, uint32_t charge_start_time); // Charge cycle loop

#endif