#ifndef DATA_H
#define DATA_H

#include "../Utils/Utils.h"
#include "../Sd/Sd.h"
#include "../Ev4/Ev4.h"
#include "../System/System.h"

void dump_data_to_serial();
void check_memory(ev4_t *ctx); // this should check all files

#endif