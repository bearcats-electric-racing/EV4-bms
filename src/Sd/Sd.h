#ifndef SD_H
#define SD_H

#include <FS.h>
#include <SD.h>
#include "../Ev4/Ev4.h"
#include "../System/System.h"

// SD card pins
const int CHIP_SELECT = BUILTIN_SDCARD;

void sd_data_write(ev4_t *ctx);

#endif