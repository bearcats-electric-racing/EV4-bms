#include "Blinky.h"

void flash_leds() {
    int time_on = 1000; // Time each led is on in milliseconds
    bool discharge[NUM_BOARDS][18] = {0}; // '1': needs discharged, '0': does not need discharged
    for (int i = NUM_BOARDS - 1; i >= 0; i--) {
        if (i % 4 < 2) {
            for (int j = 0; j < NUM_CELLS; j++) {
                discharge[i][j] = true;
                discharge_cells(discharge);
                delay(time_on);
                discharge[i][j] = false;
                discharge_cells(discharge);
            }
        } else {
            for (int j = NUM_CELLS; j >= 0; j--) {
                discharge[i][j] = true;
                discharge_cells(discharge);
                delay(time_on);
                discharge[i][j] = false;
                discharge_cells(discharge);
            }
        }
    }
}