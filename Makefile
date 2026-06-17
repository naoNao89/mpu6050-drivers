PORT ?=
BAUD ?= 115200
TARGET ?= riscv32imc-unknown-none-elf
LOG_DIR ?= logs
LOG_FILE ?=
DURATION ?=
NO_FLASH ?= 0
NO_MONITOR ?= 0
NO_LOG ?= 0
SECONDS ?= 600
SAMPLE_RATE_HZ ?= 10
LABEL ?= stationary
VALIDATION_MODE ?= report
NOISE_PSD_BAND_LOW_HZ ?=
NOISE_PSD_BAND_HIGH_HZ ?=
MIN_SAMPLES_PER_AXIS ?= 10
MIN_SAMPLES ?=
MIN_STATIONARY_SAMPLES ?=
BIN := target/$(TARGET)/release/mpu6050-esp32c3-bringup

.PHONY: build flash monitor run clean fmt check validate-stationary validate-orientation orientation-analyze allan psd imu-tool-smoke

build:
	env -u RUSTFLAGS cargo build -p mpu6050-esp32c3-bringup --release --target $(TARGET)

flash: build
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'env -u RUSTFLAGS espflash flash --port "$$ESP_PORT" "$(BIN)"'

monitor:
	PORT="$(PORT)" BAUD="$(BAUD)" TARGET="$(TARGET)" LOG_DIR="$(LOG_DIR)" LOG_FILE="$(LOG_FILE)" DURATION="$(DURATION)" NO_FLASH=1 NO_MONITOR="$(NO_MONITOR)" NO_LOG="$(NO_LOG)" ./run.sh

run:
	PORT="$(PORT)" BAUD="$(BAUD)" TARGET="$(TARGET)" LOG_DIR="$(LOG_DIR)" LOG_FILE="$(LOG_FILE)" DURATION="$(DURATION)" NO_FLASH="$(NO_FLASH)" NO_MONITOR="$(NO_MONITOR)" NO_LOG="$(NO_LOG)" ./run.sh

validate-stationary:
	PORT="$(PORT)" NOISE_PSD_BAND_LOW_HZ="$(NOISE_PSD_BAND_LOW_HZ)" NOISE_PSD_BAND_HIGH_HZ="$(NOISE_PSD_BAND_HIGH_HZ)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- stationary-suite --port "$$ESP_PORT" --seconds "$(SECONDS)" --baud "$(BAUD)" --sample-rate-hz "$(SAMPLE_RATE_HZ)" --label "$(LABEL)" --out-dir "$(LOG_DIR)" --validation-mode "$(VALIDATION_MODE)" $${NOISE_PSD_BAND_LOW_HZ:+--noise-psd-band-low-hz "$$NOISE_PSD_BAND_LOW_HZ"} $${NOISE_PSD_BAND_HIGH_HZ:+--noise-psd-band-high-hz "$$NOISE_PSD_BAND_HIGH_HZ"}'

validate-orientation:
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- orientation-capture --port "$$ESP_PORT" --seconds "$(SECONDS)" --baud "$(BAUD)" --stop-when-covered --min-samples-per-axis "$(MIN_SAMPLES_PER_AXIS)" --out "$(LOG_DIR)/auto-orientation-$$(date +%Y%m%d-%H%M%S).log"'

orientation-analyze:
	cargo run -p imu-tool -- orientation-analyze "$(LOG_FILE)" --min-samples-per-axis "$(MIN_SAMPLES_PER_AXIS)"

allan:
	cargo run -p imu-tool -- allan-analyze "$(LOG_FILE)" --sample-rate-hz "$(SAMPLE_RATE_HZ)" --out "$(LOG_DIR)/allan-$$(date +%Y%m%d-%H%M%S).csv"

psd:
	cargo run -p imu-tool -- psd-analyze "$(LOG_FILE)" --sample-rate-hz "$(SAMPLE_RATE_HZ)" --out "$(LOG_DIR)/psd-$$(date +%Y%m%d-%H%M%S).csv"

imu-tool-smoke:
	cargo run -p imu-tool -- analyze tools/imu-tool/tests/fixtures/stationary-60s.log --min-samples 20
	cargo run -p imu-tool -- orientation-analyze tools/imu-tool/tests/fixtures/auto-orientation.log --min-samples-per-axis 3
	cargo run -p imu-tool -- sixface-analyze tools/imu-tool/tests/fixtures/sixface.log --mapping config/sixface-mapping.example.json || test $$? -eq 1
	cargo test -p imu-tool sixface_fixture_parses_real_face_samples

clean:
	cargo clean

fmt:
	cargo fmt --all

check:
	cargo fmt --all -- --check
	cargo build -p imu-tool
	cargo build -p mpu6050-esp32c3-bringup --release --target $(TARGET)
	cargo test -p imu-tool
