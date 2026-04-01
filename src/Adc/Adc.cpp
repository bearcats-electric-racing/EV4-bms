#include "Adc.h"

void adc_init(ev4_t *ctx) {
    // ADC sampling time constant (without external filter) = 50 ohms * 40 pF
    // CFR.B6 = 0 : uses external voltage reference
    // CFR.B9 = 0, FSR_ADC_A = 0 to VREF_A and FSR_ADC_B = 0 to VREF_B
    // CFR.B7 = 0 : single ended measurements
    // CFR.B11 = 0 and CFR.B10 = 1  Single-SDO Mode
    // CFR.B15:B11 = 1000 (write) or 0011 (read)
    uint8_t CFR_reg_MSB;
    uint8_t CFR_reg_LSB;
    uint8_t CFR_readback_MSB;
    uint8_t CFR_readback_LSB;
    CFR_reg_MSB = 0b10000100; // CFR [B15:B8]
    // CFR_reg_LSB = 0b01000000;  //CFR [B7:B0]
    CFR_reg_LSB = CLEAR_REG;

    // Send write CFR register command
    digitalWrite(CS1, LOW);
    SPI1.transfer(CFR_reg_MSB);
    SPI1.transfer(CFR_reg_LSB);
    for (int i = 0; i < 6; i++) // clock ADC
        SPI1.transfer(CLEAR_REG);
    digitalWrite(CS1, HIGH);
    delay(2);

    // Send read CFR register command while clocking the write config
    digitalWrite(CS1, LOW);
    SPI1.transfer(0b00110000);
    for (int i = 0; i < 6; i++)
        SPI1.transfer(CLEAR_REG); // clock ADC
    digitalWrite(CS1, HIGH);
    delay(2);

    // readback the CFR configuration
    digitalWrite(CS1, LOW);
    CFR_readback_MSB = SPI1.transfer(CLEAR_REG);
    CFR_readback_LSB = SPI1.transfer(CLEAR_REG);
    for (int i = 0; i < 6; i++)
        SPI1.transfer(CLEAR_REG); // clock ADC
    digitalWrite(CS1, HIGH);

    // the 4 MSBs of the CFR register (read/write command bits) are cleared in
    // Frame F+2 which is not consistant with the datasheet
    if ((uint8_t)(CFR_reg_MSB << 4) != (uint8_t)(CFR_readback_MSB << 4) ||
        (uint8_t)(CFR_reg_LSB << 4) != (uint8_t)(CFR_readback_LSB << 4))
    { // bit-shifts to mask the 4 MSBs
        Serial.println("adc_initialization ERROR");
        Serial.println((CFR_reg_MSB << 4), BIN);
        Serial.println((CFR_reg_LSB << 4), BIN);
        ctx->curr_sense_fault = 1;
    }
}

void adc_read() {
    uint16_t ADC_A;
    uint16_t ADC_B;
    float A_volt;
    float B_volt;
    uint16_t temp;

    digitalWrite(CS1, LOW);

    for (int i = 0; i < 2; i++) {
        temp = SPI1.transfer(CLEAR_REG); // clock ADC
        Serial.print("temp: ");
        Serial.println(temp, BIN);
    }

    ADC_A = SPI1.transfer(CLEAR_REG);
    ADC_A = ADC_A << 8;
    ADC_A = ADC_A | SPI1.transfer(CLEAR_REG);
    println_with_args("ADC_A: %u", ADC_A);

    ADC_B = SPI1.transfer(CLEAR_REG);
    ADC_B = ADC_B << 8;
    ADC_B = ADC_B | SPI1.transfer(CLEAR_REG);
    println_with_args("ADC_B: %u", ADC_B);

    temp = SPI1.transfer(CLEAR_REG);
    Serial.print("temp: ");
    Serial.println(temp, BIN);
    digitalWrite(CS1, HIGH);
    delay(2);

    A_volt = (float)(ADC_A) / UINT16_MAX * 5;
    B_volt = (float)(ADC_B) / UINT16_MAX * 5;
    println_with_args("A Voltage: %f", A_volt);
    println_with_args("B Voltage: %f", B_volt);
}

void adc_poll(ev4_t *ctx, uint16_t command) {
    uint8_t return_data = 0;
    send_command(ctx, command);

    // if (!curr_sense_fault && curr_measure)
    //     measure_current();

    int num_polls = 0;
    uint32_t adc_poll_start_time = millis();
    while (return_data == 0) {
        return_data = SPI.transfer(FULL_REG); // Send dummy byte to receive data
        num_polls++;
        if(millis() - adc_poll_start_time > 1000){          // 1000ms is a made up number and should be reviewed
            Serial.println("ADC TImeout Error");
            break;
        }
    }
    // Serial.println("ADC Conversion Done!");
    // Serial.println(num_polls);

    digitalWrite(CS, HIGH);
}