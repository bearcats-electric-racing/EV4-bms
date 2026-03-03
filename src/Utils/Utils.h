#ifndef UTILS_H  
#define UTILS_H

#include <stdint.h>
#include <cstddef>
#include <stdarg.h>
#include "../System/System.h"
#include "../Spi/Spi.h"
#include "../Pec/Pec.h"
#include "../Ev4/Ev4.h"
#include "../Wakeup/Wakeup.h"
#include "TUtils.h"

String format_string(const char* format, va_list args);
void print_with_args(const char* format, ...);
void println_with_args(const char* format, ...);

uint8_t power_limit(const float max_cell_temp);
uint8_t float_2_uint8_t(const float float_val, const float min, const float max); // float to uint8_t, clips values under/over min or max
float map_voltage_to_temp(float V); // voltage -> actual temp
void map_text2var(ev4_t *ctx, String name, String value); // map text name and value to a variable
float get_voltage(ev4_t *ctx, int index);   //Gets voltage based on given index
float get_temperature(ev4_t *ctx, int index);   //Gets voltage based on given index

void send_command(ev4_t *ctx, uint16_t command);
void read_register_group(ev4_t *ctx, uint16_t command, uint8_t response[NUM_BOARDS][6]); // register group is always 6 bytes
void write_register_group(ev4_t *ctx, uint16_t command, uint8_t data[NUM_BOARDS][6]);

void print_min_max(ev4_t *ctx); // This function prints the min and max parameters
bool determine_mode(ev4_t *ctx, CAN_message_t msg, bool CAN_baud_alt); // Determine starting mode (return true to break outer loop)
void configure_sense(ev4_t *ctx);

#endif