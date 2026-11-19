#!/usr/bin/env python3
"""modbus 自检：CRC、帧编解码、位打包、从站逻辑、客户端-从站联调。

运行：python test_modbus.py
"""
from __future__ import annotations

import socket
import struct
import threading
import time
import unittest

import modbus as mb
from modbus_cli import Client, serve


class TestCrc(unittest.TestCase):
    def test_known_vector(self):
        """标准测试向量：01 03 00 00 00 0A -> CRC 0xCDC5"""
        frame = bytes.fromhex("01030000000A")
        self.assertEqual(mb.crc16(frame), 0xCDC5)

    def test_append_and_check(self):
        frame = mb.append_crc(bytes.fromhex("01030000000A"))
        self.assertEqual(frame.hex(), "01030000000ac5cd")   # 低字节在前
        self.assertTrue(mb.check_crc(frame))

    def test_corrupted_frame_fails(self):
        frame = bytearray(mb.append_crc(bytes.fromhex("01030000000A")))
        frame[2] ^= 0xFF
        self.assertFalse(mb.check_crc(bytes(frame)))

    def test_too_short(self):
        self.assertFalse(mb.check_crc(b"\x01\x03"))


class TestRtuFrame(unittest.TestCase):
    def test_roundtrip(self):
        pdu = mb.req_read_holding(0, 4)
        frame = mb.build_rtu(1, pdu)
        parsed = mb.parse_rtu(frame)
        self.assertEqual(parsed.slave, 1)
        self.assertEqual(parsed.pdu, pdu)

    def test_bad_slave_rejected(self):
        with self.assertRaises(mb.ModbusError):
            mb.build_rtu(248, b"\x03")


class TestTcpFrame(unittest.TestCase):
    def test_roundtrip(self):
        pdu = mb.req_read_holding(10, 2)
        frame = mb.build_tcp(0x1234, 7, pdu)
        f = mb.parse_tcp(frame)
        self.assertEqual(f.transaction, 0x1234)
        self.assertEqual(f.unit, 7)
        self.assertEqual(f.pdu, pdu)
        self.assertEqual(f.length, len(pdu) + 1)

    def test_length_field_used_for_reading(self):
        pdu = mb.req_read_holding(0, 4)
        frame = mb.build_tcp(1, 1, pdu)
        self.assertEqual(mb.tcp_frame_length(frame[:6]), len(frame))

    def test_truncated_frame_rejected(self):
        frame = mb.build_tcp(1, 1, mb.req_read_holding(0, 4))
        with self.assertRaises(mb.ModbusError):
            mb.parse_tcp(frame[:-1])


class TestBits(unittest.TestCase):
    def test_pack_low_bit_first(self):
        self.assertEqual(mb.pack_bits([True, False, True]), b"\x05")

    def test_roundtrip(self):
        bits = [i % 3 == 0 for i in range(20)]
        self.assertEqual(mb.unpack_bits(mb.pack_bits(bits), 20), bits)

    def test_unpack_beyond_data(self):
        self.assertEqual(mb.unpack_bits(b"\xff", 16), [True] * 8 + [False] * 8)


class TestSlaveState(unittest.TestCase):
    def setUp(self):
        self.state = mb.SlaveState.demo()

    def test_read_holding(self):
        resp = self.state.handle(mb.req_read_holding(0, 4))
        self.assertEqual(mb.parse_response(resp), [1000, 1001, 1002, 1003])

    def test_write_register_then_read(self):
        self.state.handle(mb.req_write_register(5, 4242))
        resp = self.state.handle(mb.req_read_holding(5, 1))
        self.assertEqual(mb.parse_response(resp), [4242])

    def test_write_multi_registers(self):
        self.state.handle(mb.req_write_multi_registers(10, [1, 2, 3]))
        resp = self.state.handle(mb.req_read_holding(10, 3))
        self.assertEqual(mb.parse_response(resp), [1, 2, 3])

    def test_write_coil(self):
        self.state.handle(mb.req_write_coil(3, True))
        resp = self.state.handle(mb.req_read_coils(0, 4))
        self.assertEqual(mb.parse_response(resp, expect_bits=4), [0, 1, 0, 1])

    def test_coils_default_returns_full_bytes(self):
        resp = self.state.handle(mb.req_read_coils(0, 3))
        bits = mb.parse_response(resp)          # 不裁剪，1 个字节 = 8 位
        self.assertEqual(len(bits), 8)
        self.assertEqual(bits[:3], [0, 1, 0])

    def test_illegal_address_returns_exception(self):
        resp = self.state.handle(mb.req_read_holding(900, 1))
        self.assertEqual(resp[0], 0x80 | mb.FC_READ_HOLDING)
        self.assertEqual(resp[1], mb.EX_ILLEGAL_ADDRESS)

    def test_illegal_function_returns_exception(self):
        resp = self.state.handle(bytes([0x2B]))   # 未实现的功能码
        self.assertEqual(resp[0], 0x80 | 0x2B)
        self.assertEqual(resp[1], mb.EX_ILLEGAL_FUNCTION)

    def test_illegal_coil_value(self):
        resp = self.state.handle(struct.pack(">BHH", mb.FC_WRITE_COIL, 0, 0x1234))
        self.assertEqual(resp[1], mb.EX_ILLEGAL_VALUE)


class TestClientServer(unittest.TestCase):
    """起一个真从站线程，用真 socket 联调。"""

    PORT = 15020

    @classmethod
    def setUpClass(cls):
        cls.state = mb.SlaveState.demo()
        cls.thread = threading.Thread(
            target=serve, args=("127.0.0.1", cls.PORT, cls.state), daemon=True
        )
        cls.thread.start()
        time.sleep(0.4)

    def test_end_to_end_read_write(self):
        c = Client("127.0.0.1", self.PORT, unit=1)
        try:
            self.assertEqual(mb.parse_response(c.request(mb.req_read_holding(0, 2))), [1000, 1001])
            c.request(mb.req_write_register(0, 777))
            self.assertEqual(mb.parse_response(c.request(mb.req_read_holding(0, 1))), [777])
        finally:
            c.close()

    def test_exception_propagates_to_client(self):
        c = Client("127.0.0.1", self.PORT, unit=1)
        try:
            with self.assertRaises(mb.ModbusException) as cm:
                mb.parse_response(c.request(mb.req_read_holding(9999, 1)))
            self.assertEqual(cm.exception.code, mb.EX_ILLEGAL_ADDRESS)
        finally:
            c.close()

    def test_multiple_connections(self):
        for _ in range(3):
            c = Client("127.0.0.1", self.PORT, unit=1)
            try:
                self.assertEqual(len(mb.parse_response(c.request(mb.req_read_holding(0, 3)))), 3)
            finally:
                c.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
