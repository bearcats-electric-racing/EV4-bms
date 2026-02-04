#ifndef EV4_H
#define EV4_H

#include <CONFIGURE.h>

typedef struct Ev4_t {
    //LTC6813 minimum supply voltage is 16V
    float cell_voltage[num_boards][num_cells];  //most recent cell voltages
    float open_circuit_voltage[num_boards][num_cells];
    float pack_voltage = 0;          //sum of cell voltages
    float cell_temp[num_boards][9];  //most recent cell temperatures. Contans raw voltage data for the duration of open wire checks
    float die_temps[num_boards];     //most recent sense board LTC6813 die temps
    Config_t cfg;
} Ev4_t;

#endif