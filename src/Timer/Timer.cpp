#include "Timer.h"

#include "../Ev4/Ev4.h"

static ev4_t *timer_ctx = NULL;

void setup_timers(ev4_t *ctx) {
    timer_ctx = ctx;
    CCM_CCGR6 |= CCM_CCGR6_QTIMER4(CCM_CCGR_ON); // Turn on clock on quad timers

    for (uint8_t i = 0; i < NUM_TIMERS; i++) { // Configure counting timers
        const timerinfo_t &t = ctx->timer_list[i];

        t.timer->CH[t.channel].CTRL = 0;
        t.timer->CH[t.channel].CNTR = 0;
        t.timer->CH[t.channel].LOAD = 0;
        t.timer->CH[t.channel].COMP1 = UINT16_MAX;
        t.timer->CH[t.channel].CMPLD1 = UINT16_MAX;
        t.timer->CH[t.channel].SCTRL = 0;
        t.timer->CH[t.channel].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(t.channel) | TMR_CTRL_LENGTH;

        *portConfigRegister(t.pin) = t.pin_config;

        if (t.input_select_reg)
            *t.input_select_reg = t.input_select_val;

        ctx->cntr[i] = &t.timer->CH[t.channel].CNTR;
    }
}

void gate_timer_callback() {
    if (timer_ctx)
        gate_timer_isr(timer_ctx);
}

void gate_timer_isr(ev4_t *ctx) {
    for (uint8_t i = 0; i < NUM_TIMERS; i++) {
        uint16_t change = *(ctx->cntr[i]) - ctx->start_count[i];
        ctx->pcb_temp[i] = change * SCALE;  
    }

    ctx->new_freq = true;
    ctx->gate_timer.end();
}