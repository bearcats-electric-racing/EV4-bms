#include "Balance.h"

void discharge_cells(ev4_t *ctx, bool discharge[NUM_BOARDS][18]) {
    uint8_t data[6];
    uint8_t data_arr[NUM_BOARDS][6];
    uint16_t VUV = (UV - 1.5f) / (16 * 0.00015f); // Cell undervoltage threshold = VUV * 16 * 150μV + 1.5V
    uint16_t VOV = (OV - 1.5f) / (16 * 0.00015f); // Cell overvoltage threshold = VOV * 16 * 150μV + 1.5V

    // Configuration register group B
    for (int i = 0; i < NUM_BOARDS; i++) {
        data[0] = (uint8_t)VUV;
        data[1] = ((uint8_t)VOV << 4) | (VUV >> 8 & 0b00001111);
        data[2] = (uint8_t)(VOV >> 4);
        data[3] = (uint8_t)CLEAR_REG;
        data[4] = (uint8_t)discharge[i][7] << 7 | discharge[i][6] << 6 |
                  discharge[i][5] << 5 | discharge[i][4] << 4 |
                  discharge[i][3] << 3 | discharge[i][2] << 2 |
                  discharge[i][1] << 1 | discharge[i][0] << 0; 
        data[5] = (uint8_t)discharge[i][15] << 7 | discharge[i][14] << 6 |
                  discharge[i][13] << 5 | discharge[i][12] << 4 |
                  discharge[i][11] << 3 | discharge[i][10] << 2 |
                  discharge[i][9] << 1 | discharge[i][8] << 0;
        std::copy(data, data + 6, data_arr[i]);
    }
    write_register_group(ctx, WRCFGB, data_arr);
}

void balance_cells(ev4_t *ctx, bool set) {
    bool discharge[NUM_BOARDS][18] = {0}; // '1': needs dischaged, '0': does not need discharged

    if (!set) {
        discharge_cells(ctx, discharge);
        return;
    }

    measure_die_temp(ctx);
    println_with_args("Min cell voltage: %f", ctx->min_cell_voltage);
    println_with_args("Max cell voltage: %f", ctx->max_cell_voltage);

    for (int i = 0; i < NUM_BOARDS; i++) {
        for (int j = 0; j < NUM_CELLS; j++) {
            discharge[i][j] = ctx->cell_voltage[i][j] > (ctx->min_cell_voltage + ctx->cfg.balance_precision) &&
                                ctx->cell_voltage[i][j] > ctx->cfg.balance_threshold &&
                                ctx->die_temps[i] < 58.0f;
            if (discharge[i][j]) {
                println_with_args("Board: %d | Cell: %d | Volt: %f", i + 1, j + 1, ctx->cell_voltage[i][j]);
                println_with_args("Die temp: %f", ctx->die_temps[i]);
            }
        }
    }

    discharge_cells(ctx, discharge);
}