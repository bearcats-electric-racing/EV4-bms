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
#include "src/Sd/Sd.h"
#include "src/Standby/Standby.h"
#include "src/Drive/Drive.h"
#include "src/Soc/Soc.h"
#include "src/Data/Data.h"
#include "src/Balance/Balance.h"


// Holds all globals in the context of EV4
static ev4_t ctx{};

void setup() {

    delay(5000); // startup delay should be use to make it easier to recover the teensy when runtime errors occurs
    
    // Start timers
    ctx.sense_watchdog_timer = ctx.start_time - 2000;   // initial sense_watchdog timer with expired watchdog time (T - 2000 milliseconds)

    Serial.begin(9600);
    println_with_args("Startup/n/tStart Time: %u", ctx.start_time);

    //Initialize SPI (isoSPI)
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    Serial.println("SPI Initialized");

    /*
    //Initialize SPI1 (ADC)
    pinMode(CS1, OUTPUT);
    digitalWrite(CS1, HIGH);
    SPI1.begin();
    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    Serial.println("SPI1 Initialized");
    

    // can_init(&ctx);
    // Serial.println("CAN Initialized");

    // Initialize watchdog and ADC
    watchdog_init(&ctx);
    Serial.println("Watchdog Initialized");

    /*
    adc_init(&ctx);
    Serial.println("ADC (Current) initialized");
    


    // Current offset compensation
    measure_current(&ctx);
    ctx.current_offset = ctx.current;
    Serial.println("Set Current Offset");
    */

    // Bring up references on sense boards
    // configure_sense(&ctx);

    // check_memory(&ctx); // must be called to use SD card
    // Serial.println("SD Checked");

    //balance_cells(&ctx, OFF);       // No cell balancing by default
    //Serial.println("Balance Disabled");

    // voltage poll and temperature poll take 16 and 24 milliseconds. The rest of
    // the measure functions only take 1 or two milliseconds

    if (ctx.mode == Mode::Init) {
        measure_voltage(&ctx);
        // measure_current(&ctx);
        // soc_update(&ctx);

        // CAN_message_t msg;
        // bool CAN_baud_alt = true;

        while (1) {
            Serial.println("Setup");
            //measure_current(&ctx);
            measure_voltage(&ctx);
            //cell_open_wire_check(&ctx);
            //measure_temp(&ctx);
               
            // can_tx(&ctx); // wrong baud rate every other message

            // Serial.println("can_tx complete");
            //print_min_max(&ctx);
            //watchdog_reset(&ctx);
            // msg = can_rx(&ctx);
            // Serial.println("can_rx done");
            
            /*
            // Alternate CAN baud rate (250000 for charger, 500000 for vehicle)
            if (CAN_baud_alt) {
                Serial.println("CAN_baud_alt");
                ctx.can.setBaudRate(500000);
                Serial.println("Set new baud rate");
                CAN_baud_alt = false;
            } else {
                Serial.println("NOT CAN_baud_alt");
                ctx.can.setBaudRate(250000);
                Serial.println("Set new baud rate");
                CAN_baud_alt = true;
            }
            */

            //Serial.println("Calling determine_mode");
            //if (determine_mode(&ctx, msg, CAN_baud_alt)) 
            //    break;
            //Serial.println("Determine mode done");
            
            // TEMPORARY NEEDS TO BE REMOVED
            ctx.wdt.feed();

            //delay(20);
            // Serial.println("Loop complete");
            delay(500);
            //Serial.println("Loop delay complete");
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

        balance_cells(&ctx, OFF);       //Temporarily disabled
        
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
            print_min_max(&ctx);
            delay(20);
    }

    case Mode::Debug: 
    default: {
        digitalWrite(SC, LOW); // open shutdown circuit in debug mode
        Serial.println("Debug Mode Entered");
        balance_cells(&ctx, OFF);
        while (1) 
            dump_data_to_serial();
    }
    }
}
