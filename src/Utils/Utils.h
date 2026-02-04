#ifndef UTILS_H
#define UTILS_H

#include <CONFIGURE.h>
#include <src/Wakeup/Wakeup.h>
#include <src/Pec/Pec.h>
#include "TUtils.h"

void send_command(uint16_t command);
void read_register_group(uint16_t command, uint8_t response[num_boards][6]);

#endif