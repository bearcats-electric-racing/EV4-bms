#include "Sd.h"

void sd_data_write(ev4_t *ctx) {
    String filename = "data" + String(ctx->data_file_num) + ".csv";
    File dataFile = SD.open(filename.c_str(), FILE_WRITE);
    if (dataFile) {
        dataFile.print("Mode: ");
        dataFile.println(ctx->mode);
        for (int n = 0; n < SD_INTERVAL; n++) {
            dataFile.print("Voltage:\n");
            if (n % VOLT_INTERVAL == 0 || ctx->mode != Drive) {
                for (int i = 0; i < NUM_BOARDS; i++) {
                    for (int j = 0; j < NUM_CELLS; j++) {
                        if (ctx->mode == Drive)
                            dataFile.print(ctx->voltage_buffer[int(n / VOLT_INTERVAL)][i][j], 4);
                        else
                            dataFile.print(ctx->cell_voltage[i][j], 4);
                        dataFile.print(", ");
                    }
                    dataFile.print("\n");
                }
            }

            if (n % CELL_TEMP_INTERVAL == 0 || ctx->mode != Drive) {
                dataFile.print("\nTemperature:\n");
                for (int i = 0; i < NUM_BOARDS; i++) {
                    for (int j = 0; j < NUM_THERMISTORS; j++) {
                        if (ctx->mode == Drive)
                            dataFile.print(ctx->cell_temp_buffer[int(n / CELL_TEMP_INTERVAL)][i][j], 2);
                        else
                            dataFile.print(ctx->cell_temp[i][j], 2);
                        dataFile.print(", ");
                    }
                    dataFile.print("\n");
                }
            }

            if (n % CURRENT_INTERVAL == 0 || ctx->mode != Drive) {
                dataFile.print("Current: ");
                if (ctx->mode == Drive)
                    ctx->currentbuffer_stat = ctx->current_buffer[int(n / CURRENT_INTERVAL)];
                dataFile.print(ctx->currentbuffer_stat);
            } else {
                dataFile.print(ctx->current);
                dataFile.print("\n");
            }
            dataFile.print("Time:\n");

            // time stamp
            if (ctx->mode == Drive)
                dataFile.println(ctx->time_buffer[n]);
            else
                dataFile.println(millis() - ctx->start_time);
            break;
        }
        dataFile.close();
    }
}