.PHONY: run upload tdd clean

BUILD_DIR := build

run:
	pio run

upload:
	pio run -t upload

test-single:
	@if [ -z "$(TEST)" ]; then \
		echo "Usage: make test-single TEST=FirstTest"; \
		exit 1; \
	fi
	TEST_NAME=$(TEST) ctest --test-dir build -R all_tests

test-group:
	@if [ -z "$(GROUP)" ]; then \
		echo "Usage: make test-group GROUP=FirstTestGroup"; \
		exit 1; \
	fi
	TEST_GROUP=$(GROUP) ctest --test-dir build -R all_tests
	
tdd:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR)
	ctest --test-dir $(BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR)
	pio run -t clean
