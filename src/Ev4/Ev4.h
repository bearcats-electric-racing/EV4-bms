#ifndef EV4_H
#define EV4_H

#include <Watchdog_t4.h>
#include <FlexCAN_T4.h>
#include "../System/System.h"

typedef struct ev4_t {
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
    WDT_T4<WDT1> wdt; // watchdog 1 holds output pin low until power-on-reset. This is desired for a shutdown circuit

    uint8_t mode;

    float cell_voltage[NUM_BOARDS][NUM_CELLS]; // most recent cell voltages
    float min_cell_voltage;
    float max_cell_voltage;
    float open_circuit_voltage[NUM_BOARDS][NUM_CELLS];
    float pack_voltage; // sum of cell voltages
    bool new_voltage;

    float cell_temp[NUM_BOARDS][10]; // most recent cell temperatures. Contains raw voltage data for the duration of open wire checks
    float min_cell_temp;
    float max_cell_temp;
    float die_temps[NUM_BOARDS]; // most recent sense board LTC6813 die temps
    bool new_temp;

    float current;
    float current_offset;

    float soc; // state of charge
    float inv_voltage; // inverter voltage read from CAN
    int data_file_num; // data.csv enumeration

    // measurement buffers
    unsigned int time_buffer[SD_INTERVAL];
    float voltage_buffer[SD_INTERVAL / VOLT_INTERVAL][NUM_BOARDS][NUM_CELLS];
    float temp_buffer[SD_INTERVAL / TEMP_INTERVAL][NUM_BOARDS][10];
    float current_buffer[SD_INTERVAL / CURRENT_INTERVAL];
    float currentbuffer_stat;

    unsigned int start_time = millis();
    unsigned int sense_watchdog_timer;  // senseboard watchdog timer. Sense boards will go to sleep after 2s
                                        // if no valid command with correct PEC is sent from master.

    // flags
    int wire_cut;
    bool memory_fault;
    bool comms_fault;
    bool curr_sense_fault;
    bool watchdog_callback;
    bool charger_fault;

    // RMS calc values
    long current_count;
    long current_sum;
    int RMS_Current;

    // Ev4 configuration (modifiable attributes)
    config_t cfg;
} ev4_t;

#endif