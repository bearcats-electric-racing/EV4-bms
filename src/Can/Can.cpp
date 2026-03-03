#include "Can.h"

void can_init(ev4_t* ctx) {
    pinMode(CRX3, INPUT);
    pinMode(CTX3, OUTPUT);
    pinMode(STBY, OUTPUT);
    ctx->can.begin();
    ctx->can.setBaudRate(250000);
    ctx->can.setMaxMB(3); // number of CAN message mailboxes
    digitalWrite(STBY, LOW);
    // https://github.com/tonton81/FlexCAN_T4/blob/master/examples/mailbox_filtering_example_with_interrupts/mailbox_filtering_example_with_interrupts.ino
    // Mailboxes must be configured for all messages - both TX and RX
    ctx->can.setMB((FLEXCAN_MAILBOX)0, RX, STD); // Standard mailbox for Inverter ID
    ctx->can.setMB((FLEXCAN_MAILBOX)1, RX, EXT); // Extended id for charger
    ctx->can.setMB((FLEXCAN_MAILBOX)2, TX, EXT); // BMS TX -> charger id
    ctx->can.setMBFilter(MB0, INV_TX_ID);        // Mailbox for Inverter CAN messages
    ctx->can.setMBFilter(MB1, CHG_TX_ID);        // Mailbox for Charger CAN Messages
    ctx->can.setMBFilter(MB2, 0x1806E5F4);       // Mailbox for Charger CAN Messages
}

CAN_message_t can_rx(ev4_t *ctx) {
    // left bit in charger flag is highest bit (bit 4)
    CAN_message_t msg = {};
    digitalWrite(STBY, LOW);
    // bool received = false;
    ctx->can.read(msg);
    // can.readMB(msg);
    if (msg.id != 0 && ctx->cfg.debug) {
        Serial.print("ID: ");
        Serial.print(msg.id, HEX);
        Serial.println(" Data: ");
        // msg.len = 8;
        for (int i = 0; i < msg.len; i++) {
            Serial.print(msg.buf[i], BIN);
            Serial.print(" ");
        }
        Serial.print('\n');
    }
    return msg; // always check the ID of the returned message. No messages in buffer returns 0 ID with 8 byte of zero data
}

void can_tx(ev4_t *ctx) {
    measure_voltage(ctx);
    measure_temp(ctx);
    float min_cell_voltage = ctx->cell_voltage[0][0];
    float max_cell_voltage = ctx->cell_voltage[0][0];
    float min_cell_temp = ctx->cell_temp[0][0];
    float max_cell_temp = ctx->cell_temp[0][0];
    min_max<NUM_BOARDS, NUM_CELLS>(ctx->cell_voltage, min_cell_voltage, max_cell_voltage);
    min_max<NUM_BOARDS, 10>(ctx->cell_temp, min_cell_temp, max_cell_temp);
    uint8_t inst_power_limit = power_limit(max_cell_temp);
    println_with_args("Power Limit: %u", inst_power_limit);

    digitalWrite(STBY, LOW);
    digitalWrite(CTX3, HIGH);
    delay(1);

    CAN_message_t BMS_data;
    // BMS_data.id = BMS_ID;
    BMS_data.id = 0x00000007;
    BMS_data.flags.extended = 0;
    BMS_data.len = 8; // Set the data length

    BMS_data.buf[0] = float_2_uint8_t(ctx->soc, 0, 100);                // SOC
    BMS_data.buf[1] = float_2_uint8_t(ctx->currentbuffer_stat, 0, 250); // current
    BMS_data.buf[2] = float_2_uint8_t(max_cell_voltage, 0, 5);     // max cell voltage
    BMS_data.buf[3] = float_2_uint8_t(max_cell_temp, 0, 75);      // max cell temp
    BMS_data.buf[4] = float_2_uint8_t(min_cell_voltage, 0, 5);     // min cell voltage
    BMS_data.buf[5] = float_2_uint8_t(min_cell_temp, 0, 75);      // min cell temp
    BMS_data.buf[6] = inst_power_limit;                            // BMS Suggested Power Limit
    BMS_data.buf[7] = 0;

    if (ctx->can.write(BMS_data))
        Serial.println("CAN message sent (Summary)");
    else
        Serial.println("CAN message TX Failed (Summary)");

    digitalWrite(CTX3, LOW);
}

void can_tx_all(ev4_t *ctx) {
    measure_voltage(ctx);

    digitalWrite(STBY, LOW);
    digitalWrite(CTX3, HIGH);
    delay(1);

    CAN_message_t BMS_data_all;

    BMS_data_all.id = 0x0000000A;
    BMS_data_all.flags.extended = 0;
    BMS_data_all.len = 8; // Set the data length
    
    //Find # of CAN Messages for Voltage Transmission
    uint8_t num_txs = NUM_BOARDS * NUM_CELLS / 8;
    if(NUM_BOARDS * NUM_CELLS % 8 != 0) {
        num_txs++;
    }
    uint8_t cell_index = 0;

    //Transmit Voltages
    for(int i = 0; i < num_txs; i++) {
        BMS_data_all.buf[0] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        BMS_data_all.buf[1] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        BMS_data_all.buf[2] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        BMS_data_all.buf[3] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        BMS_data_all.buf[4] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        BMS_data_all.buf[5] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        BMS_data_all.buf[6] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        BMS_data_all.buf[7] = float_2_uint8_t(get_voltage(ctx, cell_index++), 2.00, 4.55);
        ctx->can.write(BMS_data_all);
    }

    measure_temp(ctx);

    //Find # of CAN Messages for Temperature Transmission
    num_txs = NUM_BOARDS * 10 / 8;
    if(NUM_BOARDS * 10 % 8 != 0) {
        num_txs++;
    }
    uint8_t temp_index = 0;

    //Transmit Temperatures
    for(int i = 0; i < num_txs; i++) {
        BMS_data_all.buf[0] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        BMS_data_all.buf[1] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        BMS_data_all.buf[2] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        BMS_data_all.buf[3] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        BMS_data_all.buf[4] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        BMS_data_all.buf[5] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        BMS_data_all.buf[6] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        BMS_data_all.buf[7] = float_2_uint8_t(get_temperature(ctx, temp_index++), 0.00, 75.00);
        ctx->can.write(BMS_data_all);
    }

    digitalWrite(CTX3, LOW);
}

