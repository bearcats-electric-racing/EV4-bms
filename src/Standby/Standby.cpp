#include "Standby.h"

void standby_state(ev4_t *ctx) {
    while (1) {
        Serial.println(ctx->mode);

        measure_current(ctx);
        measure_voltage(ctx);
        measure_cell_temp(ctx);
        measure_pcb_temp(ctx);
        watchdog_reset(ctx);

        // CAN_message_t msg = can_rx(ctx);

        // BYPASSED PRECHARGE CHECK - FIX IN FUTURE BY PULLING DATA FROM ECU
        // if (msg.id == INV_TX_ID) 
        //     inv_voltage = float(msg.buf[0] * 256 + msg.buf[1]);

        // if (inv_voltage >= pack_voltage * 0.8) { // checks inverter voltage to see if precharge is occuring
        //     mode = Drive; // enter drive mode if precharging
        //     break;
        // }

        ctx->mode = Mode::Drive; // Temporary line to bypass precharge check
        delay(10);
        break;
    }
}