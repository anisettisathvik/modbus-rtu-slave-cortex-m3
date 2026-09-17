CC      := gcc
CLANG   := clang
CFLAGS  := -std=c99 -Wall -Wextra -Wpedantic -Wconversion -Werror -O1 -g \
           -Isrc -Iport -Itest
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer
SRC     := src/modbus_crc.c src/modbus_slave.c

FUZZ_TIME  ?= 60
RAND_ITERS ?= 200000
RAND_SEED  ?= 0xC0FFEE

.PHONY: all verify test random bus loopback fuzz coverage analyze clean help

help:
	@echo "make verify    - run the whole verification suite"
	@echo "make test      - directed unit tests"
	@echo "make random    - constrained-random + coverage closure (sanitized)"
	@echo "make bus       - exhaustive multi-slave bus contention proof"
	@echo "make fuzz      - libFuzzer, FUZZ_TIME=$(FUZZ_TIME)s"
	@echo "make coverage  - gcov line and branch coverage"
	@echo "make analyze   - cppcheck static analysis"
	@echo "make loopback  - end-to-end over a virtual serial pair"

all: verify

verify: test random bus coverage analyze
	@echo ""
	@echo "=============================================="
	@echo "  full verification suite complete"
	@echo "=============================================="

build/test_slave: $(SRC) test/test_slave.c
	@mkdir -p build
	$(CC) $(CFLAGS) $(SAN) $(SRC) test/test_slave.c -o $@

test: build/test_slave
	./build/test_slave

build/random_test: $(SRC) test/random_test.c test/mb_invariants.h test/mb_coverage.h
	@mkdir -p build
	$(CC) -std=c99 -Wall -Wextra -O1 -g -Isrc -Iport -Itest $(SAN) \
	      $(SRC) test/random_test.c -o $@

random: build/random_test
	./build/random_test $(RAND_ITERS) $(RAND_SEED)

build/bus_sim: $(SRC) test/bus_sim.c
	@mkdir -p build
	$(CC) $(CFLAGS) $(SAN) $(SRC) test/bus_sim.c -o $@

bus: build/bus_sim
	./build/bus_sim

build/fuzz_slave: $(SRC) test/fuzz_slave.c test/mb_invariants.h
	@mkdir -p build corpus
	$(CLANG) -std=c99 -g -O1 -Isrc -Iport -Itest \
	         -fsanitize=fuzzer,address,undefined \
	         $(SRC) test/fuzz_slave.c -o $@

fuzz: build/fuzz_slave
	@mkdir -p corpus
	# -symbolize=0: on a NEW_FUNC discovery libFuzzer otherwise shells out to
	# llvm-symbolizer to name the function. If that binary is absent or slow
	# to spawn the fuzzer blocks on it, which looks like a hang and can drop
	# throughput by six orders of magnitude. Function names are not what this
	# run is measuring — crash count and execution rate are.
	./build/fuzz_slave corpus/ -max_total_time=$(FUZZ_TIME) \
	    -print_final_stats=1 -max_len=256 -symbolize=0

coverage:
	@mkdir -p build/cov
	$(CC) -std=c99 -O0 -g --coverage -Isrc -Iport -Itest \
	      $(SRC) test/random_test.c -o build/cov/covtest
	cd build/cov && ./covtest 50000 0xC0FFEE > /dev/null
	@gcov -b -s . build/cov/covtest-modbus_slave.gcda \
	      build/cov/covtest-modbus_crc.gcda \
	  | grep -E "^File|^Lines|^Branches|^Taken"

analyze:
	cppcheck --enable=all --inconclusive --check-level=exhaustive --std=c99 --error-exitcode=1 \
	         --suppress=missingIncludeSystem --suppress=unusedFunction \
	         --suppress=checkersReport \
	         -Isrc -Iport src/ 2>&1 | tail -20
	@echo "cppcheck: clean"

build/host_slave: $(SRC) test/host_slave.c
	@mkdir -p build
	$(CC) -std=c99 -Wall -Wextra -Wpedantic -O1 -g -Isrc -Iport \
	      $(SRC) test/host_slave.c -o $@

loopback: build/host_slave
	./run_loopback.sh

clean:
	rm -rf build *.gcov corpus

# ---- Cortex-M3 firmware ---------------------------------------------------
# The SAME modbus_slave.c and modbus_crc.c that the host suite verifies,
# cross-compiled for ARMv7-M. Only port/mb_port_cm3.c knows about hardware.
ARMCC   := arm-none-eabi-gcc
ARMSIZE := arm-none-eabi-size
QEMU    := qemu-system-arm

ARMFLAGS := -mcpu=cortex-m3 -mthumb -std=gnu99 -O2 -g -Wall -Wextra \
            -ffunction-sections -fdata-sections -Isrc -Iport
ARMLD    := -T linker/mps2_an385.ld -nostartfiles -Wl,--gc-sections \
            --specs=nano.specs --specs=nosys.specs -lc -lnosys

FW_SRC := firmware/main.c src/modbus_slave.c src/modbus_crc.c \
          port/mb_port_cm3.c port/startup_mps2.c port/semihost.c

build/firmware.elf: $(FW_SRC) linker/mps2_an385.ld
	@mkdir -p build
	$(ARMCC) $(ARMFLAGS) $(FW_SRC) $(ARMLD) -o $@
	$(ARMSIZE) $@

firmware: build/firmware.elf

run-firmware: build/firmware.elf
	$(QEMU) -machine mps2-an385 -cpu cortex-m3 -m 16M -nographic \
	        -serial mon:stdio -kernel $<

# ---- one-command demo -----------------------------------------------------
demo:
	@mkdir -p logs
	@{ \
	  echo "########################################################"; \
	  echo "#  Modbus RTU Slave - full verification run             #"; \
	  echo "########################################################"; \
	  echo ""; echo "### 1. directed unit tests (ASan + UBSan) ###"; $(MAKE) -s test; \
	  echo ""; echo "### 2. exhaustive multi-slave bus contention proof ###"; $(MAKE) -s bus; \
	  echo ""; echo "### 3. constrained-random + functional coverage closure ###"; $(MAKE) -s random; \
	  echo ""; echo "### 4. line and branch coverage ###"; $(MAKE) -s coverage; \
	  echo ""; echo "### 5. static analysis ###"; $(MAKE) -s analyze; \
	  echo ""; echo "### 6. coverage-guided fuzzing (60 s smoke run) ###"; \
	  echo "    NOTE: 60 s is a smoke test. The figure quoted in the README"; \
	  echo "    comes from 'make fuzz FUZZ_TIME=600' with a warm corpus."; \
	  $(MAKE) -s fuzz FUZZ_TIME=60 2>&1 | tail -8; \
	  echo ""; echo "### 7. end-to-end over a virtual serial pair ###"; $(MAKE) -s loopback; \
	  echo ""; echo "### 8. Cortex-M3 firmware build ###"; $(MAKE) -s firmware; \
	  echo ""; echo "### 9. real master driving the firmware over UART ###"; ./tools/run_firmware_test.sh; \
	} 2>&1 | tee logs/demo.log
	@echo ""; echo "transcript written to logs/demo.log"

.PHONY: demo
