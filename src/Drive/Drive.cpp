#include "Drive.h"

void drive_state(ev4_t *ctx, int t, CAN_message_t msg) {
    while (1) {
        ctx->time_buffer[t] = millis() - ctx->start_time;
        if (t % CURRENT_INTERVAL == 0) {
            measure_current(ctx);
            ctx->current_buffer[int(t / CURRENT_INTERVAL)] = ctx->current;
        }

        if (t % VOLT_INTERVAL == 0) {
            measure_voltage(ctx);
            Serial.println("New Volt");
            for (int i = 0; i < NUM_BOARDS; i++) {
                for (int j = 0; j < NUM_CELLS; j++) {
                    ctx->voltage_buffer[int(t / VOLT_INTERVAL)][i][j] = ctx->cell_voltage[i][j];
                }
            }
        }

        if (t % CELL_TEMP_INTERVAL == 0) {
            Serial.println("New Cell Temp");
            measure_cell_temp(ctx);
            for (int i = 0; i < NUM_BOARDS; i++) {
                for (int j = 0; j < NUM_THERMISTORS; j++) {
                    ctx->cell_temp_buffer[int(t / CELL_TEMP_INTERVAL)][i][j] = ctx->cell_temp[i][j];
                }
            }
        }

        if (t % PCB_TEMP_INTERVAL == 0) {
            Serial.println("New PCB Temp");
            measure_pcb_temp(ctx);
            for (int i = 0; i < NUM_TIMERS; i++) {
                ctx->pcb_temp_buffer[int(t / PCB_TEMP_INTERVAL)][i] = ctx->pcb_temp[i];
            }
        }

        if (ctx->new_voltage && ctx->new_temp) {
            print_min_max(ctx);
            watchdog_reset(ctx);
        }

        if (t % SD_INTERVAL == 0 && !ctx->memory_fault) 
            sd_data_write(ctx);

        if (t % CAN_INTERVAL == 0) {
            Serial.println("Send CAN");
            soc_update(ctx);
            can_tx(ctx);
        }

        // msg = RX_CAN();
        // if(msg.id == INV_TX_ID){
        //   inv_voltage = float(msg.buf[0]*256 + msg.buf[1]);
        // }
        // if(inv_voltage < pack_voltage * 0.5){    //checks inverter voltage
        // to see if tractive system voltage is dropping
        //   mode = "standby";                         //enter standby mode
        //   if ready to drive is exited
        //   //data_file_num = 0;
        //   //check_memory();         //assign a new data file number in
        //   case RTD is entered agian break;
        // }

        while (millis() - ctx->start_time <= ctx->time_buffer[t] + TIME_STEP) {} // this needs checked

        if (t < SD_INTERVAL - 1)
            t++;
        else
            t = 0;

        println_with_args("t: %d", t);
    }
}