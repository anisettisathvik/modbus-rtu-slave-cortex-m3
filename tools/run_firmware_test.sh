#!/usr/bin/env bash
# UART0 carries Modbus, exposed as TCP 4321. The debug console goes out over
# semihosting, so the two never share a peripheral.
set -u
cd "$(dirname "$0")/.."
cleanup() { kill ${QEMU:-} 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT
mkdir -p logs
qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -m 16M -nographic \
    -semihosting-config enable=on,target=native \
    -serial tcp::4321,server,nowait \
    -kernel build/firmware.elf > logs/firmware_console.log 2>&1 &
QEMU=$!
sleep 3
python3 tools/fw_master.py 4321
RC=$?
sleep 1
echo "--- firmware console (semihosting) ---"
tr -d '\r' < logs/firmware_console.log | tail -22
exit $RC
