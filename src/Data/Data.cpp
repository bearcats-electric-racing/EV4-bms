#include "Data.h"

void dump_data_to_serial() {
    while (1) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "begin") break;
    }

    File root = SD.open("/");
    File entry = root.openNextFile();
    int file_num = 0;
    while (entry) {
        if (file_num >= FILE_READ_BEGIN) { // begin with file at FILE_READ_BEGIN
            Serial.println(entry.name());
            while (entry.available()) {
                char character = entry.read();
                Serial.write(character);

                while (character == '\n') { // block until confirmation from Python that line has been read
                    String input = Serial.readStringUntil('\n');
                    input.trim();
                    if (input == "next line")
                        break;
                }
            }

            Serial.println("done");

            while (1) { // block until Python is ready to read next file
                String input = Serial.readStringUntil('\n');
                input.trim();
                if (input == "next file")
                    break;
            }
        }
        entry.close();
        entry = root.openNextFile();
        file_num++;
    }
    root.close();

    Serial.println("serial dump done");
}

void check_memory(ev4_t *ctx) {
    if (!SD.begin(CHIP_SELECT)) {
        Serial.println("SD card initialization failed!");
        ctx->memory_fault = 1;
        return;
    }

    if (!SD.exists("state.txt")) {
        File file = SD.open("state.txt", FILE_WRITE);
        file.close();
    }

    File root = SD.open("/");
    File entry = root.openNextFile();
    uint64_t memory_usage = 0;
    int num_files = 0;
    while (entry) {
        num_files++;
        println_with_args("%s\t%u bytes", entry.name(), entry.size());
        memory_usage += entry.size();
        entry.close();
        // SD.remove(entry.name());
        entry = root.openNextFile();
    }
    root.close();

    println_with_args("Memory Usage: %f%\n", 100 * memory_usage / (SD_CARD_SIZE * 1e9));

    if (memory_usage > 0.9 * (SD_CARD_SIZE * 1e9)) {
        Serial.println("SD card over 90% full");
        ctx->memory_fault = 1;
        return;
    }

    if (ctx->data_file_num == 0) {
        for (int i = 1; i < num_files + 100; i++) {
            String filename = "data" + String(i) + ".csv";
            if (!SD.exists(filename.c_str())) {
                ctx->data_file_num = i;
                break;
            }
        }
    }
}