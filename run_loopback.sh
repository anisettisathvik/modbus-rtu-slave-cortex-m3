#!/usr/bin/env bash
# End-to-end loopback with no hardware:
#   Python master  <-->  socat virtual serial pair  <-->  C slave core
# The same modbus_slave.c runs here and on the STM32.
set -u
cd "$(dirname "$0")"

cleanup() { kill "${SLAVE_PID:-}" "${SOCAT_PID:-}" 2>/dev/null; }
trap cleanup EXIT

rm -f /tmp/mb_slave /tmp/mb_master
socat pty,raw,echo=0,link=/tmp/mb_slave pty,raw,echo=0,link=/tmp/mb_master \
      >/tmp/socat.log 2>&1 &
SOCAT_PID=$!
sleep 2

./build/host_slave /tmp/mb_slave 17 >/tmp/slave.log 2>&1 &
SLAVE_PID=$!
sleep 1

python3 tools/mb_master.py /tmp/mb_master --slave 0x11 "$@"
RC=$?

echo "--- slave side log ---"
cat /tmp/slave.log
exit $RC
