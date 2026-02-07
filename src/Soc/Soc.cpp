#include "Soc.h"

void soc_save(ev4_t *ctx) {
    if (SD.exists("state.txt")) {
        File file = SD.open("state.txt", FILE_READ);
        if (file) {
            String line;
            while (file.available()) {
                line = file.readStringUntil('\n');
                Serial.println(line);
                int delim_index = line.indexOf(':');
                String name = line.substring(0, delim_index);
                String value = line.substring(delim_index + 1);
                map_text_to_var(ctx, name, value);
            }
            file.close();
        }
    } else {
        File file = SD.open("state.txt", FILE_WRITE);
        Serial.println("Initializing state of charge to 100%");
        file.close();
        Serial.println("state.txt initialized.");
    }
}

void soc_update(ev4_t *ctx) {
    const int discharge_curve_length = sizeof(DISCHARGE_POINTS) / sizeof(DISCHARGE_POINTS[0]); // length of each discharge curve
    const float max_capacity = DISCHARGE_POINTS[0]; // maximum capacity of a single cell
    // const int num_current_curves = sizeof(DISCHARGE_CURRENTS) / sizeof(DISCHARGE_CURRENTS[0]);  // number of discharge curves @ different currents
    float min_OC_cell_voltage = ctx->open_circuit_voltage[0][0]; // funct. min_max requires that the min and
                                                            // max values are initalized within the
                                                            // range of the min max values
    float max_OC_cell_voltage = ctx->open_circuit_voltage[0][0];
    min_max<NUM_BOARDS, NUM_CELLS>(ctx->open_circuit_voltage, min_OC_cell_voltage, max_OC_cell_voltage);
    float discharged = interpolate<discharge_curve_length>( // capacity which has already been discharged (mAh)
        DISCHARGE_CURVES[0], 
        DISCHARGE_POINTS, 
        min_OC_cell_voltage);
    ctx->soc = 100 - ((max_capacity - discharged) / max_capacity * 100);

    println_with_args("SOC: %f", ctx->soc);
}

float soc_get(ev4_t *ctx) {
    soc_update(ctx);
    return ctx->soc;
}