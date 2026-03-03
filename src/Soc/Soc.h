#ifndef SOC_H
#define SOC_H

#include <SD.h>
#include "../Utils/Utils.h"
#include "../System/System.h"
#include "../Ev4/Ev4.h"

void soc_save(ev4_t *ctx); // SOC should be written in the state.txt file as: "SOC:100"
void soc_update(ev4_t *ctx);
float soc_get(ev4_t *ctx);

#endif