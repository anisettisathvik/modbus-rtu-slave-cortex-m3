#!/usr/bin/env python3
"""Modbus RTU test master.

Frames are constructed by hand rather than via a library so every byte on the
wire is accounted for.

    python3 tools/mb_master.py /dev/pts/5
    python3 tools/mb_master.py /dev/pts/5 --soak 5000
"""
import argparse
import struct
import sys
import time

import serial


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def adu(body: bytes) -> bytes:
    """Append CRC in wire order: low byte first."""
    return body + struct.pack("<H", crc16(body))


class Master:
    def __init__(self, port, baud=19200, timeout=0.5):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.sent = 0
        self.recv = 0

    def txn(self, frame: bytes, expect_reply=True) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        self.sent += 1
        if not expect_reply:
            time.sleep(0.15)
            return b""
        time.sleep(0.15)
        reply = self.ser.read(256)
        if reply:
            self.recv += 1
        return reply

    @staticmethod
    def check_reply(reply: bytes):
        """Returns (ok, description)."""
        if not reply:
            return False, "timeout, no reply"
        if len(reply) < 4:
            return False, f"runt reply {len(reply)} B"
        if crc16(reply) != 0:
            return False, "reply CRC bad"
        if reply[1] & 0x80:
            return True, f"exception 0x{reply[2]:02X}"
        return True, f"ok, {len(reply)} B"


def read_holding(slave, start, qty):
    return adu(struct.pack(">BBHH", slave, 0x03, start, qty))


def write_single(slave, reg, val):
    return adu(struct.pack(">BBHH", slave, 0x06, reg, val))


def write_multiple(slave, start, values):
    body = struct.pack(">BBHHB", slave, 0x10, start, len(values), len(values) * 2)
    body += b"".join(struct.pack(">H", v) for v in values)
    return adu(body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--slave", type=lambda x: int(x, 0), default=0x11)
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--soak", type=int, default=0)
    args = ap.parse_args()

    m = Master(args.port, args.baud)
    s = args.slave
    npass = nfail = 0

    def case(name, frame, want_reply=True, want_exception=None):
        nonlocal npass, nfail
        reply = m.txn(frame, want_reply)
        if not want_reply:
            ok = (reply == b"")
            desc = "silent as expected" if ok else f"unexpected {len(reply)} B"
        else:
            ok, desc = Master.check_reply(reply)
            if ok and want_exception is not None:
                ok = len(reply) >= 3 and (reply[1] & 0x80) and reply[2] == want_exception
                desc += f" (wanted exception 0x{want_exception:02X})"
        print(f"  {'PASS' if ok else 'FAIL'}  {name:<38} {desc}")
        print(f"        tx {frame.hex(' ')}")
        if reply:
            print(f"        rx {reply.hex(' ')}")
        if ok:
            npass += 1
        else:
            nfail += 1
        return reply

    print("\n=== valid transactions ===")
    case("read 4 holding registers", read_holding(s, 0, 4))
    case("write single register", write_single(s, 3, 0xBEEF))
    case("read back written value", read_holding(s, 3, 1))
    case("write multiple, 3 registers", write_multiple(s, 8, [0x1111, 0x2222, 0x3333]))
    case("read back the block", read_holding(s, 8, 3))

    print("\n=== protocol exceptions ===")
    case("quantity zero", read_holding(s, 0, 0), want_exception=0x03)
    case("quantity over 125", read_holding(s, 0, 200), want_exception=0x03)
    case("address beyond register file", read_holding(s, 900, 1), want_exception=0x02)
    case("write beyond register file", write_single(s, 900, 1), want_exception=0x02)
    case("unsupported function 0x2B",
         adu(struct.pack(">BBHH", s, 0x2B, 0, 1)), want_exception=0x01)

    print("\n=== frames that must be dropped ===")
    bad = bytearray(read_holding(s, 0, 1))
    bad[-1] ^= 0xFF
    case("corrupted CRC", bytes(bad), want_reply=False)
    case("addressed to another slave", read_holding(0x22, 0, 1), want_reply=False)
    case("truncated frame", read_holding(s, 0, 1)[:4], want_reply=False)
    case("broadcast write", write_single(0x00, 5, 0xC0DE), want_reply=False)
    case("broadcast took effect", read_holding(s, 5, 1))

    if args.soak:
        print(f"\n=== soak: {args.soak} transactions ===")
        t0 = time.time()
        errors = 0
        for i in range(args.soak):
            r = m.txn(read_holding(s, i % 16, 1))
            ok, _ = Master.check_reply(r)
            if not ok:
                errors += 1
        dt = time.time() - t0
        print(f"  {args.soak} txns, {errors} errors, {dt:.1f} s, "
              f"{args.soak/dt:.1f} txn/s")
        if errors:
            nfail += 1
        else:
            npass += 1

    print("\n---------------------------------------")
    print(f"  {npass} passed, {nfail} failed")
    print("---------------------------------------\n")
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
