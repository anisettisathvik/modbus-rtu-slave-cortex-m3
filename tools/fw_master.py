#!/usr/bin/env python3
"""Modbus RTU master driving the Cortex-M3 firmware over QEMU's UART, exposed
as a TCP socket. Exercises the UART RX interrupt, the ring buffer and the
hardware frame-gap timer inside the emulated MCU."""
import socket, struct, sys, time

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def adu(body): return body + struct.pack("<H", crc16(body))
def read_hold(u, s, q): return adu(struct.pack(">BBHH", u, 3, s, q))
def write_one(u, r, v): return adu(struct.pack(">BBHH", u, 6, r, v))

host, port = "127.0.0.1", int(sys.argv[1]) if len(sys.argv) > 1 else 4321
s = socket.create_connection((host, port), timeout=5)
s.settimeout(3.0)
npass = nfail = 0

def case(name, frame, expect_reply=True, want_exc=None):
    global npass, nfail
    s.sendall(frame)
    time.sleep(0.4)
    try:
        r = s.recv(256)
    except socket.timeout:
        r = b""
    if not expect_reply:
        ok, detail = (r == b""), ("silent" if r == b"" else f"got {len(r)}B")
    elif not r:
        ok, detail = False, "no reply"
    elif crc16(r) != 0:
        ok, detail = False, f"bad crc: {r.hex(' ')}"
    elif want_exc is not None:
        ok = (r[1] & 0x80) and r[2] == want_exc
        detail = r.hex(' ')
    else:
        ok, detail = not (r[1] & 0x80), r.hex(' ')
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<38} {detail}")
    if ok: npass += 1
    else:  nfail += 1

print("\n=== driving the Cortex-M3 firmware over UART0 ===")
case("read 2 holding registers",  read_hold(0x11, 0, 2))
case("write single register",     write_one(0x11, 3, 0xBEEF))
case("read back written value",   read_hold(0x11, 3, 1))
case("quantity zero -> exc 03",   read_hold(0x11, 0, 0), want_exc=0x03)
case("bad address -> exc 02",     read_hold(0x11, 900, 1), want_exc=0x02)
bad = bytearray(read_hold(0x11, 0, 1)); bad[-1] ^= 0xFF
case("corrupt CRC dropped",       bytes(bad), expect_reply=False)
case("other slave ignored",       read_hold(0x22, 0, 1), expect_reply=False)
case("broadcast write silent",    write_one(0x00, 7, 0xC0DE), expect_reply=False)
case("broadcast took effect",     read_hold(0x11, 7, 1))

print(f"\n  {npass} passed, {nfail} failed\n")
s.close()
sys.exit(0 if nfail == 0 else 1)
