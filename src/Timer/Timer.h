#ifndef TIMER_H
#define TIMER_H

#include <imxrt.h>

// https://github.com/PaulStoffregen/FreqCountMany

#define GATE_INTERVAL (4000) // microseconds per gate interval
#define SCALE (1000000.0f / GATE_INTERVAL)

struct ev4_t;

typedef struct {
    IMXRT_TMR_t *timer;
    int channel;
    int pin;
    int pin_config;
    volatile uint32_t *input_select_reg;
    int input_select_val;
} timerinfo_t;

void setup_timers(ev4_t *ctx);
void gate_timer_isr(ev4_t *ctx);
void gate_timer_callback();

#endif