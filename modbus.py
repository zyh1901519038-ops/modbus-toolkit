#!/usr/bin/env python3
"""modbus.py —— Modbus RTU / TCP 协议实现（纯 Python 标准库）

支持：
    - RTU 帧：地址 + PDU + CRC16（多项式 0xA001，init 0xFFFF）
    - TCP 帧：MBAP 头（事务号 + 协议号 + 长度 + 单元号）+ PDU
    - 功能码：0x01 读线圈 / 0x03 读保持寄存器 / 0x05 写单线圈
              0x06 写单寄存器 / 0x10 写多寄存器
    - 异常响应：0x80 | fc + 异常码

"""
from __future__ import annotations

import struct
from dataclasses import dataclass

# ----------------------------- 异常码 -----------------------------
EX_ILLEGAL_FUNCTION = 0x01
EX_ILLEGAL_ADDRESS = 0x02
EX_ILLEGAL_VALUE = 0x03
EX_SLAVE_FAILURE = 0x04

EXCEPTION_TEXT = {
    EX_ILLEGAL_FUNCTION: "非法功能码",
    EX_ILLEGAL_ADDRESS: "非法数据地址",
    EX_ILLEGAL_VALUE: "非法数据值",
    EX_SLAVE_FAILURE: "从站设备故障",
}

# ----------------------------- 功能码 -----------------------------
FC_READ_COILS = 0x01
FC_READ_HOLDING = 0x03
FC_WRITE_COIL = 0x05
FC_WRITE_REGISTER = 0x06
FC_WRITE_MULTI_REGISTERS = 0x10


class ModbusError(Exception):
    """协议层错误（长度不对、CRC 错、功能码不支持等）。"""


class ModbusException(Exception):
    """从站返回的异常响应。"""

    def __init__(self, function: int, code: int):
        self.function = function
        self.code = code
        super().__init__(f"功能码 0x{function:02X} 返回异常 0x{code:02X}：{EXCEPTION_TEXT.get(code, '未知')}")


# ----------------------------- CRC16 -----------------------------
def crc16(data: bytes) -> int:
    """Modbus CRC16：多项式 0xA001（反向），初值 0xFFFF，不异或输出。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def append_crc(frame: bytes) -> bytes:
    """RTU 帧尾部追加 CRC（低字节在前）。"""
    return frame + struct.pack("<H", crc16(frame))


def check_crc(frame: bytes) -> bool:
    """校验带 CRC 的 RTU 帧。帧长不足 4 字节直接判失败。"""
    if len(frame) < 4:
        return False
    return crc16(frame[:-2]) == struct.unpack("<H", frame[-2:])[0]


# ----------------------------- RTU 封装 -----------------------------
def build_rtu(slave: int, pdu: bytes) -> bytes:
    if not 0 <= slave <= 247:
        raise ModbusError(f"从站地址超出范围：{slave}")
    return append_crc(bytes([slave]) + pdu)


@dataclass
class RtuFrame:
    slave: int
    pdu: bytes


def parse_rtu(frame: bytes) -> RtuFrame:
    if len(frame) < 4:
        raise ModbusError("RTU 帧过短")
    if not check_crc(frame):
        raise ModbusError("CRC 校验失败")
    return RtuFrame(slave=frame[0], pdu=frame[1:-2])


# ----------------------------- TCP 封装（MBAP） -----------------------------
@dataclass
class MbapFrame:
    transaction: int
    protocol: int
    length: int
    unit: int
    pdu: bytes


def build_tcp(transaction: int, unit: int, pdu: bytes) -> bytes:
    """MBAP 头的 length 字段 = 单元号(1) + PDU 长度。"""
    header = struct.pack(">HHHB", transaction & 0xFFFF, 0, len(pdu) + 1, unit & 0xFF)
    return header + pdu


def parse_tcp(buffer: bytes) -> MbapFrame:
    """解析一条完整的 MBAP 报文。长度字段先校验，再切片。"""
    if len(buffer) < 7:
        raise ModbusError("MBAP 头不完整")
    transaction, protocol, length, unit = struct.unpack(">HHHB", buffer[:7])
    if length < 1:
        raise ModbusError(f"MBAP 长度字段非法：{length}")
    total = 6 + length
    if len(buffer) < total:
        raise ModbusError(f"报文不完整：需要 {total} 字节，实际 {len(buffer)}")
    return MbapFrame(transaction, protocol, length, unit, buffer[7:total])


def tcp_frame_length(header6: bytes) -> int:
    """从 MBAP 前 6 字节推算整帧长度，用于流式读取。"""
    if len(header6) < 6:
        raise ModbusError("MBAP 头不完整")
    _, _, length = struct.unpack(">HHH", header6[:6])
    return 6 + length


# ----------------------------- 位打包 -----------------------------
def pack_bits(bits: list[bool]) -> bytes:
    """低位在前（Modbus 的位序约定）。"""
    out = bytearray((len(bits) + 7) // 8)
    for i, bit in enumerate(bits):
        if bit:
            out[i // 8] |= 1 << (i % 8)
    return bytes(out)


def unpack_bits(data: bytes, count: int) -> list[bool]:
    bits: list[bool] = []
    for i in range(count):
        byte_index, bit_index = divmod(i, 8)
        if byte_index >= len(data):
            bits.append(False)
        else:
            bits.append(bool(data[byte_index] >> bit_index & 1))
    return bits


# ----------------------------- PDU 构造 -----------------------------
def make_read_coils(slave: int) -> bytes:
    return bytes([FC_READ_COILS])


def req_read_coils(start: int, quantity: int) -> bytes:
    _check_qty(quantity, 1, 2000)
    return struct.pack(">BHH", FC_READ_COILS, start, quantity)


def req_read_holding(start: int, quantity: int) -> bytes:
    _check_qty(quantity, 1, 125)
    return struct.pack(">BHH", FC_READ_HOLDING, start, quantity)


def req_write_coil(address: int, on: bool) -> bytes:
    return struct.pack(">BHH", FC_WRITE_COIL, address, 0xFF00 if on else 0x0000)


def req_write_register(address: int, value: int) -> bytes:
    _check_value(value, 0, 0xFFFF)
    return struct.pack(">BHH", FC_WRITE_REGISTER, address, value)


def req_write_multi_registers(start: int, values: list[int]) -> bytes:
    _check_qty(len(values), 1, 123)
    for v in values:
        _check_value(v, 0, 0xFFFF)
    body = struct.pack(">BHHB", FC_WRITE_MULTI_REGISTERS, start, len(values), len(values) * 2)
    return body + b"".join(struct.pack(">H", v) for v in values)


def _check_qty(qty: int, lo: int, hi: int) -> None:
    if not lo <= qty <= hi:
        raise ModbusError(f"数量越界：{qty}（应在 {lo}~{hi}）")


def _check_value(v: int, lo: int, hi: int) -> None:
    if not lo <= v <= hi:
        raise ModbusError(f"数值越界：{v}（应在 {lo}~{hi}）")


# ----------------------------- 响应解析 -----------------------------
def parse_response(pdu: bytes, expect_bits: int | None = None) -> list[int] | str:
    """把响应 PDU 解析成寄存器值列表 / 位列表 / 回显描述。异常响应抛 ModbusException。

    expect_bits：读线圈时响应只带「字节数」，不带「数量」。主站知道自己请求了几个，
    所以要传进来做裁剪；不传则返回完整的 字节数×8 个位。
    """
    if not pdu:
        raise ModbusError("PDU 为空")
    fc = pdu[0]

    if fc & 0x80:
        if len(pdu) < 2:
            raise ModbusError("异常响应缺异常码")
        raise ModbusException(fc & 0x7F, pdu[1])

    if fc == FC_READ_COILS:
        byte_count = pdu[1]
        bits = unpack_bits(pdu[2:2 + byte_count], byte_count * 8)
        if expect_bits is not None:
            bits = bits[:expect_bits]
        return [int(b) for b in bits]
    if fc == FC_READ_HOLDING:
        count = pdu[1] // 2
        return list(struct.unpack(f">{count}H", pdu[2:2 + count * 2]))
    if fc in (FC_WRITE_COIL, FC_WRITE_REGISTER):
        return f"已写入 地址={struct.unpack('>H', pdu[1:3])[0]} 值={struct.unpack('>H', pdu[3:5])[0]}"
    if fc == FC_WRITE_MULTI_REGISTERS:
        addr, qty = struct.unpack(">HH", pdu[1:5])
        return f"已写入 {qty} 个寄存器，起始地址={addr}"
    raise ModbusError(f"不支持的功能码 0x{fc:02X}")


# ----------------------------- 从站逻辑 -----------------------------
@dataclass
class SlaveState:
    """一个最小可用的从站数据模型。"""

    coils: dict[int, bool]
    registers: dict[int, int]

    @classmethod
    def demo(cls) -> "SlaveState":
        return cls(
            coils={i: bool(i % 2) for i in range(16)},
            registers={i: 1000 + i for i in range(32)},
        )

    def handle(self, pdu: bytes) -> bytes:
        """处理请求 PDU，返回响应 PDU。异常以 0x80|fc 返回。"""
        if not pdu:
            raise ModbusError("PDU 为空")
        fc = pdu[0]

        try:
            if fc == FC_READ_COILS:
                _, start, qty = struct.unpack(">BHH", pdu[:5])
                _check_qty(qty, 1, 2000)
                missing = [a for a in range(start, start + qty) if a not in self.coils]
                if missing:
                    raise ModbusException(fc, EX_ILLEGAL_ADDRESS)
                bits = [self.coils[a] for a in range(start, start + qty)]
                payload = pack_bits(bits)
                return bytes([fc, len(payload)]) + payload

            if fc == FC_READ_HOLDING:
                _, start, qty = struct.unpack(">BHH", pdu[:5])
                _check_qty(qty, 1, 125)
                missing = [a for a in range(start, start + qty) if a not in self.registers]
                if missing:
                    raise ModbusException(fc, EX_ILLEGAL_ADDRESS)
                values = [self.registers[a] for a in range(start, start + qty)]
                return bytes([fc, qty * 2]) + b"".join(struct.pack(">H", v) for v in values)

            if fc == FC_WRITE_COIL:
                _, addr, value = struct.unpack(">BHH", pdu[:5])
                if value not in (0x0000, 0xFF00):
                    raise ModbusException(fc, EX_ILLEGAL_VALUE)
                if addr not in self.coils:
                    raise ModbusException(fc, EX_ILLEGAL_ADDRESS)
                self.coils[addr] = value == 0xFF00
                return pdu[:5]

            if fc == FC_WRITE_REGISTER:
                _, addr, value = struct.unpack(">BHH", pdu[:5])
                if addr not in self.registers:
                    raise ModbusException(fc, EX_ILLEGAL_ADDRESS)
                self.registers[addr] = value
                return pdu[:5]

            if fc == FC_WRITE_MULTI_REGISTERS:
                _, start, qty, byte_count = struct.unpack(">BHHB", pdu[:6])
                _check_qty(qty, 1, 123)
                if byte_count != qty * 2 or len(pdu) < 6 + byte_count:
                    raise ModbusException(fc, EX_ILLEGAL_VALUE)
                missing = [a for a in range(start, start + qty) if a not in self.registers]
                if missing:
                    raise ModbusException(fc, EX_ILLEGAL_ADDRESS)
                for i in range(qty):
                    self.registers[start + i] = struct.unpack(">H", pdu[6 + i * 2:8 + i * 2])[0]
                return struct.pack(">BHH", fc, start, qty)

            raise ModbusException(fc, EX_ILLEGAL_FUNCTION)

        except ModbusException as e:
            return bytes([0x80 | e.function, e.code])
