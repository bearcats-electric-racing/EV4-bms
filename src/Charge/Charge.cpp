#include "Charge.h"

const uint16_t charger_current_from_soc(ev4_t *ctx) {
    const float soc = soc_get(ctx);
    if (soc < 80)
        return CHG_CURRENT1;
    else if (soc < 90)
        return CHG_CURRENT2;
    else
        return CHG_CURRENT3; 
}

void configure_charger(ev4_t *ctx, bool enable, uint16_t charger_current) {
    uint16_t chg_current = charger_current_from_soc(ctx);
    if (chg_current == charger_current)
        return;

    digitalWrite(STBY, LOW);
    digitalWrite(CTX3, HIGH);
    delay(1);

    CAN_message_t CHGR_EN;
    CHGR_EN.id = 0x1806E6F4; // Set the CAN message ID datasheet
    CHGR_EN.flags.extended = 1;
    CHGR_EN.len = 8; // Set the data length
    // 7FF max CAN ID

    uint16_t voltage_int = (uint16_t)(CHG_VOLTAGE * 10);
    uint16_t current_int = (uint16_t)(chg_current * 10);

    CHGR_EN.buf[0] = (uint8_t)(voltage_int >> 8); // High byte
    CHGR_EN.buf[1] = (uint8_t)(voltage_int);      // Low byte
    CHGR_EN.buf[2] = (uint8_t)(current_int >> 8); // High byte
    CHGR_EN.buf[3] = (uint8_t)(current_int);      // Low byte
    CHGR_EN.buf[4] = (uint8_t)(!enable);          // 0 turns charger on
    CHGR_EN.buf[5] = 0;
    CHGR_EN.buf[6] = 0;
    CHGR_EN.buf[7] = 0;

    bool message_sent = ctx->can.write(CHGR_EN);
    for (int i = 0; i < CHGR_EN.len; i++) {
        Serial.print(CHGR_EN.buf[i], BIN);
        Serial.print(" ");
    }

    digitalWrite(CTX3, LOW);

    if (!message_sent)
    {
    }
}

void charge_precharge(ev4_t *ctx) {
    while (1) {
        measure_voltage(ctx);
        measure_cell_temp(ctx);
        measure_pcb_temp(ctx);
        watchdog_reset(ctx);
        configure_charger(ctx, true); // send charge-disable message and clear comm fault on charger
        CAN_message_t msg = can_rx(ctx);
        float charger_voltage = ((uint16_t)msg.buf[0] << 8 | (uint16_t)msg.buf[1]) / 10;
        Serial.println(charger_voltage);
        Serial.println(ctx->pack_voltage);

        if ((msg.id == CHG_TX_ID && msg.buf[4] == 0) || true) { // && charger_voltage >= pack_voltage * 0.80) {  // if can id
                                                                // matches charger AND there are no charger faults AND precharge is complete
            break;
        }
    }
}

void charge_state(ev4_t *ctx, uint32_t charge_start_time) {
    while (1) { // charge cycle
        println_with_args("Time: %f minutes", (float)(millis() - charge_start_time) / 60000);
        println_with_args("Charge fault status: %f", ctx->charger_fault);

        measure_voltage(ctx);
        measure_cell_temp(ctx);
        measure_pcb_temp(ctx);
        measure_current(ctx);

        println_with_args("Current: %f", ctx->current);
        println_with_args("Pack Voltage: %f", ctx->pack_voltage);
        print_min_max(ctx);

        if (!ctx->memory_fault) 
            sd_data_write(ctx);

        CAN_message_t msg = can_rx(ctx);
        float charger_current = ((uint16_t)msg.buf[2] << 8 | (uint16_t)msg.buf[3]) / 10;

        if (watchdog_reset(ctx)) {
            float charger_voltage = ((uint16_t)msg.buf[0] << 8 | (uint16_t)msg.buf[1]) / 10;
            println_with_args("Charger voltage: %d", charger_voltage);
            println_with_args("Charger current: %d", charger_current);

            if ((msg.id == CHG_TX_ID && msg.buf[4] == 0) || true) {
                configure_charger(ctx, false);
            } else { // charger error
                digitalWrite(CS, LOW);
                configure_charger(ctx, false);
                Serial.println("Charger Error");
                ctx->charger_fault = 1;
            }
        } else {
            configure_charger(ctx, true, (uint16_t)charger_current);
        }

        delay(1000);
    }
}