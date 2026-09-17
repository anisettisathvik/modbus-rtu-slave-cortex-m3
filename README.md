# Modbus RTU Slave — Portable C Core with Cortex-M3 Firmware

A Modbus RTU slave implemented as a portable C99 protocol core with no hardware
dependencies, plus a bare-metal ARM Cortex-M3 port. The protocol layer turns a
received byte buffer into a response buffer and contains no reference to any
peripheral, so the identical source file is verified on a host PC and
cross-compiled for the target. Verification uses constrained-random stimulus
with a functional coverage model, assertion checking on every transaction,
coverage-guided fuzzing, and an exhaustive proof that two slaves can never
transmit simultaneously on a shared bus.

---

## Repository layout

```
mb-rtu/
├── src/
│   ├── modbus_slave.h        protocol API, function and exception codes
│   ├── modbus_slave.c        frame validation, function decode, exceptions
│   ├── modbus_crc.h          CRC-16/MODBUS
│   └── modbus_crc.c
├── port/
│   ├── mb_port.h             the complete hardware interface (6 functions)
│   ├── mb_port_cm3.c         Cortex-M3: UART RX ISR, ring buffer, t3.5 timer
│   ├── startup_mps2.c        vector table, reset handler, C runtime init
│   └── semihost.c            debug console over ARM semihosting
├── firmware/
│   └── main.c                target frame loop and on-target self-test
├── linker/
│   └── mps2_an385.ld         memory map for the MPS2-AN385
├── test/
│   ├── test_slave.c          directed unit tests
│   ├── random_test.c         constrained-random stimulus, coverage closure
│   ├── mb_invariants.h       invariants checked after every transaction
│   ├── mb_coverage.h         functional coverage model
│   ├── bus_sim.c             exhaustive multi-slave bus contention check
│   ├── fuzz_slave.c          libFuzzer harness
│   └── host_slave.c          POSIX serial runner for the loopback test
├── tools/
│   ├── mb_master.py          Modbus RTU test master (serial)
│   ├── fw_master.py          master driving the firmware over QEMU's UART
│   └── run_firmware_test.sh  firmware under QEMU + master
├── .github/workflows/ci.yml
├── run_loopback.sh
└── Makefile
```

---

## Requirements

```bash
sudo apt install build-essential clang cppcheck socat \
                 gcc-arm-none-eabi qemu-system-arm
pip install pyserial
```

## Build and run

```bash
make test        # directed unit tests, ASan + UBSan
make bus         # exhaustive multi-slave bus contention check
make random      # constrained-random stimulus, coverage closure
make coverage    # gcov line and branch coverage
make analyze     # cppcheck static analysis
make fuzz        # libFuzzer, FUZZ_TIME=60 by default
make loopback    # Python master <-> socat pty <-> C slave
make firmware    # Cortex-M3 image
make demo        # all of the above, transcript to logs/demo.log
```

The firmware under QEMU, driven by a real master:

```bash
./tools/run_firmware_test.sh
```

A longer fuzzing run, which produced the figure below:

```bash
make fuzz FUZZ_TIME=600
```

---

## Verification results

Measured on Ubuntu 26.04 under WSL2 — GCC 13.3, Clang 18, arm-none-eabi-gcc 14.2.

| Check | Result |
|---|---|
| Directed unit tests | 34 / 34 pass |
| Constrained-random | 200,000 frames, 0 invariant violations |
| Functional coverage | 40 / 40 bins |
| Line coverage (gcov) | 100.00% of 124 lines (`modbus_slave.c`) |
| Branch coverage | 100.00% executed, 100.00% taken at least once |
| Fuzzing (libFuzzer) | 117,914,441 executions, 0 crashes, 196,197 exec/s |
| Fuzz coverage | saturated at `cov: 80`, `ft: 127` |
| Sanitizers | ASan + UBSan clean |
| Static analysis | cppcheck `--enable=all --check-level=exhaustive`, clean |
| Compiler | `-Wall -Wextra -Wpedantic -Wconversion -Werror`, clean |
| Multi-slave bus | 1,024 frames, 8 slaves, 0 collisions |
| Seed regression | 5 independent seeds, all closed coverage |
| Host loopback | 15 / 15 |
| Firmware on QEMU | 9 / 9, 72 bytes via RX ISR, 9 frames delimited, 0 overruns |

CRC verified against the standard check value: `mb_crc16("123456789")` = `0x4B37`.

### Verification structure

| Layer | File | Finds |
|---|---|---|
| Directed tests | `test/test_slave.c` | spec conformance, known cases |
| Constrained-random | `test/random_test.c` | untested input combinations |
| Functional coverage | `test/mb_coverage.h` | when the input space is covered |
| Invariants | `test/mb_invariants.h` | silent corruption, on every frame |
| Fuzzing | `test/fuzz_slave.c` | parser faults on arbitrary bytes |

Ten invariants are checked after every transaction, including that a rejected
request leaves the register file byte-identical — asserted across 80,703
rejected frames.

### Multi-slave bus contention

RS-485 is a shared differential pair; two slaves transmitting simultaneously
destroys the frame, and on hardware the fault is intermittent and gives no
indication of which device transmitted. `test/bus_sim.c` places 8 slaves on a
virtual segment and sweeps every unit identifier 0–255 against every function
code, testing that the number of transmitting slaves is never greater than one.
Result: 1,024 frames, 32 answered, 992 correctly ignored, 0 collisions, and a
broadcast write verified to reach all 8 slaves silently.

---

## Cortex-M3 firmware

`port/mb_port_cm3.c` implements the three properties that are timing behaviour
rather than protocol behaviour:

- **UART RX interrupt into a lock-free ring buffer.** Single producer, single
  consumer, power-of-two size; no critical section required because each side
  owns one index and 32-bit aligned access is atomic on Cortex-M.
- **A hardware timer measuring the 3.5-character inter-frame gap.** Above 19200
  baud the spec fixes this at 1.750 ms; below, it is 3.5 character times of 11
  bit times. At 19200 baud that is 5,390 core cycles — 215 µs. RTU has no
  framing character, so this timer's expiry is the only frame delimiter.
- **RS-485 DE/RE direction control**, released only after the shift register
  empties rather than the holding register.

The debug console uses semihosting rather than a second UART, so log output
cannot corrupt protocol traffic.

---

## Protocol notes

- CRC is checked ahead of the address filter: a corrupted address byte cannot
  be trusted, and the spec's bus communication error counter is bus-wide.
- `start + qty` is widened to 32 bits; `start = 0xFFFF, qty = 2` wraps in
  16-bit arithmetic and would pass a naive bounds check.
- Byte count in function 0x10 is cross-checked twice: against `2 × qty`
  (protocol error, exception 03) and against the received length (malformed
  ADU, silent drop). These are different failures and take separate paths.
- Broadcast writes execute but never reply; broadcast reads are dropped. No
  exception is ever sent to address 0.
- A 4-byte truncated frame is reported as a CRC error, not a length error:
  4 bytes is a structurally legal ADU, so it clears the length gate.
- The CRC is the only field in an RTU frame transmitted low byte first.

---

## Limitations and known gaps

**No physical hardware.** The firmware is real ARMv7-M, cross-compiled with
`arm-none-eabi-gcc` and run on QEMU's MPS2-AN385 Cortex-M3 model. It has not run
on silicon.

Not verified here, and requiring a transceiver and an oscilloscope:

- DE/RE turnaround timing measured on the wire
- The UART's true transmission-complete flag — the CMSDK model exposes none, so
  the port waits one character time instead
- Electrical t3.5 on a loaded bus
- Parity enforcement: a pty ignores parity entirely, so an 8E1/8N1 mismatch
  would go undetected here and fail completely on real hardware
- Failsafe biasing, termination and reflections
- Logic-analyzer captures of DE transitions and frame gaps

The host loopback uses POSIX `VTIME`, which has 100 ms granularity, so it
delimits frames by inactivity rather than a true 3.5-character gap. Frame timing
is verified only on the target, where a hardware timer measures it.

**CI runs on every push.** The workflow in `.github/workflows/ci.yml` runs the
full suite on a clean Ubuntu runner: sanitized unit tests, 500,000 random
frames, the bus proof, 120 s of fuzzing, static analysis, the loopback, a
five-seed regression, and the Cortex-M3 firmware driven by a real master. The
coverage step fails the build if line or branch coverage drops below 100%. It
passes on a runner that has never seen the development machine, so the results
above are independently reproducible rather than self-reported.
