#include "Measurement.h"

void measure_voltage(ev4_t *ctx) {
    uint8_t response[NUM_BOARDS][6];
    uint16_t cell_comm[6] = {
        RDCVA, RDCVB, RDCVC,
        RDCVD, RDCVE, RDCVF
    }; // read cell voltage registers A through F commands

    adc_poll(ctx, ADCV); // initiate and wait for voltage measurement

    ctx->pack_voltage = 0;
    for (int i = 0; i * 3 < NUM_CELLS; i++) { // i: cell group (3 cells per register group)
        // Serial.print('i');
        // Serial.println(i);
        uint16_t curr_comm = cell_comm[i]; // each command reads a sequential set
                                           // of three cells from each board
        read_register_group(ctx, curr_comm, response);
        for (int j = 0; j < NUM_BOARDS; j++) { // j: board number
            // Serial.print('j');
            // Serial.println(j);
            for (int k = 0; k < 3 && i * 3 + k < NUM_CELLS; k++) { // cell within cell group
                // Serial.print('k');
                // Serial.println(k);
                int16_t adc_code = (int16_t)(((uint16_t)response[j][k * 2 + 1] << 8) | response[j][k * 2]);
                ctx->cell_voltage[j][i * 3 + k] = (float)adc_code * 0.00015f + 1.5f; // LSB represents 150 uV, +1.5v offset
                ctx->pack_voltage += ctx->cell_voltage[j][i * 3 + k];
            }
        }
    }

    // Update open circuit voltage if possible
    if (ctx->current < 0.2 and ctx->current > -0.2) {
        for (int i = 0; i < NUM_BOARDS; ++i)
            for (int j = 0; j < NUM_CELLS; ++j)
                ctx->open_circuit_voltage[i][j] = ctx->cell_voltage[i][j];
    }

    // Update min, max voltage
    ctx->min_cell_voltage = ctx->cell_voltage[0][0];
    ctx->max_cell_voltage = ctx->cell_voltage[0][0];
    min_max<NUM_BOARDS, NUM_CELLS>(ctx->cell_voltage, ctx->min_cell_voltage, ctx->max_cell_voltage);

    ctx->new_voltage = true;

    if (ctx->cfg.debug) {
        Serial.println("Voltages:");
        for (int i = 0; i < NUM_BOARDS; i++) {
            print_with_args("\tBoard: %d\n\t", i + 1);
            for (int j = 0; j < NUM_CELLS; j++) {
                print_with_args("%f ", ctx->cell_voltage[i][j]);
            }
            Serial.println("");
        }
    }
}

void measure_temp(ev4_t *ctx, bool open_wire_check) {
    uint8_t response[NUM_BOARDS][6];
    uint16_t aux_comm[4] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD}; // read aux registers A through D commands
    int thermistor_idx = 0; // thermistor index 0-9
    int command_idx = 0; // command index within aux_comm array

    if (open_wire_check)
        adc_poll(ctx, ADAX | OW); // initiate and wait for GPIO measurement
    else
        adc_poll(ctx, ADAX);

    while (thermistor_idx < 10) {
        uint16_t curr_comm = aux_comm[command_idx];     // each command reads a sequential set of
                                                        // three GPIO from each board (RDAUXB is an
                                                        // exception with just 2 GPIO)
        read_register_group(ctx, curr_comm, response);
        for (int reading = 0; reading < 3 && thermistor_idx < 10; reading++) { // GPIO reading within group (~3 per group)
            if (command_idx == 3 && reading > 0) // Group register D has 1 reading only
                continue;

            for (int b = 0; b < NUM_BOARDS; b++) {
                uint16_t adc_code = ((uint16_t)response[b][reading * 2 + 1] << 8) | response[b][reading * 2];
                ctx->cell_temp[b][thermistor_idx] = (float)adc_code * 0.00015f - 8.33f; // LSB represents 150 uV + 1.5V
            }
            thermistor_idx++;
        }
        command_idx++;
    }

    // for (int i = 0; i < NUM_BOARDS; i++)
    //     for (int j = 0; j < 10; j++)
    //         cell_temp[i][j] = map_voltage_to_temp(cell_temp[i][j]);

    ctx->new_temp = true;

    // Update min, max temp
    ctx->min_cell_temp = ctx->cell_temp[0][0];
    ctx->max_cell_temp = ctx->cell_temp[0][0];
    min_max<NUM_BOARDS, 10>(ctx->cell_temp, ctx->min_cell_temp, ctx->max_cell_temp);

    if (ctx->cfg.debug) {
        Serial.println("Temperatures:");
        for (int i = 0; i < NUM_BOARDS; i++) {
            print_with_args("\tBoard: %d\n\t", i + 1);
            for (int j = 0; j < 10; j++) {
                print_with_args("%f ", ctx->cell_temp[i][j]);
            }
            Serial.println("");
        }
    }
}

void measure_current(ev4_t *ctx) {
    uint16_t ADC;
    float volt;
    digitalWrite(CS1, LOW);

    for (int i = 0; i < 2; i++)
        SPI1.transfer(CLEAR_REG); // clock ADC

    ADC = SPI1.transfer(CLEAR_REG);
    ADC = ADC << 8;
    ADC = ADC | SPI1.transfer(CLEAR_REG);
    volt = (float)(ADC) / 65535 * 5;
    ctx->current = (volt - 2.5) / .0267 - ctx->current_offset; // this needs checked

    if (ctx->current > 50) { // so does this
        ADC = SPI1.transfer(CLEAR_REG);
        ADC = ADC << 8;
        ADC = ADC | SPI1.transfer(CLEAR_REG);
        volt = (float)(ADC) / 65535 * 5;
        ctx->current = (volt - 2.5) / .004 - ctx->current_offset; // this needs checked
    }

    if(ctx->cfg.debug){
        if(ctx->curr_sense_fault)
            Serial.println("Current Sense Fault");
        else{
            Serial.print("Current: "); 
            Serial.println(ctx->current);
        }
    }
    
    ctx->current_count++;
    digitalWrite(CS1, HIGH);
}

void measure_die_temp(ev4_t *ctx) {
    uint8_t response[NUM_BOARDS][6];
    adc_poll(ctx, ADAX | ITEMP);
    read_register_group(ctx, RDSTATA, response);

    // Serial.println("die_temps");
    for (int i = 0; i < NUM_BOARDS; i++) {
        uint16_t adc_code = (uint16_t)response[i][3] << 8 | response[i][2];
        ctx->die_temps[i] = ((float)adc_code * 0.00015f + 1.5f) / .0075f - 273; // (ITMP * 150uV + 1.5V) / 7.5mV/C - 273C
        // Serial.println(die_temps[i]);
    }
    // Serial.println();
}
