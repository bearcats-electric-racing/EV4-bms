.PHONY: run upload tdd clean

BUILD_DIR := build

run:
	pio run

upload:
	pio run -t upload

tdd:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR)
	ctest --test-dir $(BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR)
	pio run -t clean
