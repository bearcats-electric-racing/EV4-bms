#ifndef CAN_H
#define CAN_H

#include <FlexCAN_T4.h>
#include "../Ev4/Ev4.h"
#include "../Utils/Utils.h"
#include "../System/System.h"
#include "../Measurement/Measurement.h"

void can_init(ev4_t *ctx);
CAN_message_t can_rx(ev4_t *ctx); // grabs the first message in the FIFO.
void can_tx(ev4_t *ctx);
void can_tx_all(ev4_t *ctx);

#endif