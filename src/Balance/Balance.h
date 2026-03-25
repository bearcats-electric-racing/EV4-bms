#ifndef BALANCE_H
#define BALANCE_H

#include "../Ev4/Ev4.h"
#include "../Utils/Utils.h"
#include "../System/System.h"
#include "../Measurement/Measurement.h"

void discharge_cells(bool discharge[NUM_BOARDS][18]); // this function takes a 2D boolean array which is NOT dependent on NUM_CELLS.
void balance_cells(ev4_t *ctx, bool set);

#endif