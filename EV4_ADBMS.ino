#include <SD.h>
#include <SPI.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "src/System/System.h"
#include "src/Utils/Utils.h"

#include "src/Can/Can.h"
#include "src/Watchdog/Watchdog.h"
#include "src/Adc/Adc.h"
#include "src/Charge/Charge.h"
#include "src/Measurement/Measurement.h"
#include "src/Pec/Pec.h"
#include "src/Spi/Spi.h"
#include "src/Sd/Sd.h"
#include "src/Standby/Standby.h"
#include "src/Drive/Drive.h"
#include "src/Soc/Soc.h"
#include "src/Data/Data.h"
#include "src/Balance/Balance.h"


// Holds all globals in the context of EV4
static ev4_t ctx{};

void setup() {
    // Open shutdown circuit
    pinMode(SC, OUTPUT);
    digitalWrite(SC, LOW);

    delay(5000); // startup delay should be use to make it easier to recover the teensy when runtime errors occurs

    // Start timers
    ctx.sense_watchdog_timer = ctx.start_time - 2000;   // initial sense_watchdog timer with expired watchdog time (T - 2000 milliseconds)

    Serial.begin(9600);
    println_with_args("Startup/n/tStart Time: %u", ctx.start_time);

    // Initialize communications
    spi_init(CS, SPI_MODE0); // SPI (isoSPI)
    spi_init(CS1, SPI_MODE1); // SPI1 (ADC)
    can_init(&ctx);

    // Initialize watchdog and ADC
    watchdog_init(&ctx);
    adc_init(&ctx);

    // Current offset compensation
    measure_current(&ctx);
    ctx.current_offset = ctx.current;

    // Bring up references on sense boards
    // configure_sense(&ctx);

    check_memory(&ctx); // must be called to use SD card

    // voltage poll and temperature poll take 16 and 24 milliseconds. The rest of
    // the measure functions only take 1 or two milliseconds

    if (ctx.mode == Mode::Init) {
        measure_voltage(&ctx);
        measure_current(&ctx);
        soc_update(&ctx);

        CAN_message_t msg;
        bool CAN_baud_alt = true;

        while (1) {
            Serial.println("Setup");
            measure_current(&ctx);
            measure_voltage(&ctx);
            measure_cell_temp(&ctx);

            can_tx(&ctx); // wrong baud rate every other message
            print_min_max(&ctx);
            watchdog_reset(&ctx);
            msg = can_rx(&ctx);

            // Alternate CAN baud rate (250000 for charger, 500000 for vehicle)
            if (CAN_baud_alt) {
                ctx.can.setBaudRate(500000);
                CAN_baud_alt = false;
            } else {
                ctx.can.setBaudRate(250000);
                CAN_baud_alt = true;
            }

            if (determine_mode(&ctx, msg, CAN_baud_alt)) 
                break;

            delay(20);
        }
    }
}

void loop() {
    switch (ctx.mode) {
    case Mode::Charge: {
        Serial.println("Charge Mode Entered");

        String filename = "data" + String(ctx.data_file_num) + ".csv"; // create data file
        File file = SD.open(filename.c_str(), FILE_WRITE);
        file.close();

        delay(6000); // cause comm fault on charger. Power cycling the BMS without
                     // ensuring the charger fully powers down would otherwise can
                     // cause the BMS to enter the charge cycle agian.

        charge_precharge(&ctx);

        balance_cells(&ctx, ON);
        
        // 00100 low ac power on charger flag
        delay(1000);    // delay so that another Charger CAN message is sent to the
                        // BMS (so that an empty CAN buffer is not read which would
                        // indicate a charger error)

        uint32_t charge_start_time = millis();
        charge_state(&ctx, charge_start_time);

        // Charger fault
        while (1) {
            Serial.println("Charger Fault");
            delay(1000);
        }
    }

    case Mode::Standby: { // waiting to drive. Still provides rules-compliant monitering in case CAN is lost
        Serial.println("Standby Mode Entered");
        balance_cells(&ctx, OFF);

        if (!ctx.memory_fault) {
            String filename = "data" + String(ctx.data_file_num) + ".csv"; // create data file
            File file = SD.open(filename.c_str(), FILE_WRITE);
            file.close();
            sd_data_write(&ctx); // write initial conditions to data file once
        }

        standby_state(&ctx);
    }

    case Mode::Drive: {
        Serial.println("Drive Mode Entered");
        balance_cells(&ctx, OFF);

        int t = 0; // time step number
        CAN_message_t msg;
        drive_state(&ctx, t, msg);
    }

    case Mode::Balance: {
        println_with_args("Balance Mode Entered");
        balance_cells(&ctx, ON);
        while(1)
            dump_data_to_serial();
    }

    case Mode::Debug: 
    default: {
        digitalWrite(SC, LOW); // open shutdown circuit in debug mode
        Serial.println("Debug Mode Entered");
        while (1) 
            dump_data_to_serial();
    }
    }
}
