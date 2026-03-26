#include "Watchdog.h"

static ev4_t *wdt_ctx = NULL;

void watchdog_init(ev4_t *ctx) {
    if (WATCHDOG_TIMEOUT != 0) { // callback function is having some issues
        wdt_ctx = ctx;
        
        WDT_timings_t config;
        int watchdog_trigger = WATCHDOG_TIMEOUT - 1;
        if (watchdog_trigger < 1) 
            watchdog_trigger = 1;

        config.trigger = 11; /* in seconds, 0->128 */ // time until watchdog callback function is triggered.
        config.timeout = WATCHDOG_TIMEOUT; /* in seconds, 0->128 */ // time until watchdog reset
        config.pin = SC; // pin to be driven low upon reset. WDT1 holds low, WDT2 pulses low
        config.callback = watchdog_callback_wrapper;
        ctx->wdt.begin(config);
    }
}

void watchdog_callback_wrapper() {
    if (wdt_ctx)
        watchdog_callback(wdt_ctx);
}

void watchdog_callback(ev4_t *ctx) {
    Serial.println("Callback called");
    measure_voltage(ctx);
    measure_temp(ctx);
    watchdog_reset(ctx);
    ctx->watchdog_callback = true; // set watchdog callback flag
}

bool watchdog_reset(ev4_t *ctx) { // this needs to clear the voltage and temperature measurements after reading them
    ctx->new_voltage = false;
    ctx->new_temp = false;

    if (ctx->max_cell_voltage - ctx->min_cell_voltage > MAX_DIFF){
        digitalWrite(SC, LOW);
        delay(1000); // delay to overcome debounce of shutdown circuit
        Serial.println("Open fusible link detected - max voltage differential exceeded");
        return false;
    }

    if (ctx->max_gpio_voltage > GPIO_OV){
        digitalWrite(SC, LOW);
        delay(1000); // delay to overcome debounce of shutdown circuit
        Serial.println("Open thermistor detected - max GPIO voltage exceeded");
        return false;
    }

    for (int i = 0; i < NUM_BOARDS; i++) {
        for (int j = 0; j < NUM_CELLS; j++) {
            if (ctx->cell_voltage[i][j] < OV && ctx->cell_voltage[i][j] > UV) {
                ctx->cell_voltage[i][j] = 0;
                continue;
            } else {
                digitalWrite(SC, LOW);
                delay(1000); // delay to overcome debounce of shutdown circuit
                println_with_args("Invalid voltage: %f", ctx->cell_voltage[i][j]);
                return false;
            }
        }
    }

    for (int i = 0; i < NUM_BOARDS; i++) {
        for (int j = 0; j < 10; j++) {
            if (ctx->cell_temp[i][j] > MIN_TEMP && ctx->cell_temp[i][j] < MAX_TEMP) {
                ctx->cell_temp[i][j] = MIN_TEMP;
                continue;
            } else {
                digitalWrite(SC, LOW);
                delay(1000); // delay to overcome debounce of shutdown circuit
                println_with_args("Invalid temp -> Board: %d | Num: %d", i + 1, j + 1);
                return false;
            }
        }
    }

    digitalWrite(20, HIGH);
    ctx->wdt.feed();
    if(ctx->cfg.debug)
        Serial.println("Watchdog fed");
    return true;
}