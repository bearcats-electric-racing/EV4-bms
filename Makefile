.PHONY: run run-test upload upload-test tdd test-single test-group clean

BUILD_DIR := build
TEST_BIN  := $(BUILD_DIR)/tests/tests

run:
	pio run

run-test:
	pio run -e test

upload:
	pio run -t upload

upload-test:
	pio run -e test -t upload

test-single:
	@if [ -z "$(TEST)" ]; then \
		echo "Usage: make test-single TEST=FirstTest"; \
		exit 1; \
	fi
	$(TEST_BIN) -n $(TEST)

test-group:
	@if [ -z "$(GROUP)" ]; then \
		echo "Usage: make test-group GROUP=FirstTestGroup"; \
		exit 1; \
	fi
	$(TEST_BIN) -g $(GROUP)

tdd:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR)
	$(TEST_BIN)

# If ever there are multiple test executables
# tdd-ctest:
# 	cmake -S . -B $(BUILD_DIR)
# 	cmake --build $(BUILD_DIR)
# 	ctest --test-dir $(BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR)
	pio run -t clean
