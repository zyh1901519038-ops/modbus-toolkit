#!/usr/bin/env python3
"""modbus_cli.py —— Modbus TCP 从站模拟器 + 主站客户端（命令行）

用法：
    # 启动从站（监听 1502 端口，内部有一份演示寄存器表）
    python modbus_cli.py serve --port 1502

    # 另一个终端：读 4 个保持寄存器
    python modbus_cli.py read-holding --host 127.0.0.1 --port 1502 --start 0 --count 4

    # 写单个寄存器
    python modbus_cli.py write-register --start 0 --value 1234

    # 读线圈
    python modbus_cli.py read-coils --start 0 --count 8
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading

import modbus as mb

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


# ---------------------------------------------------------------------------
# 从站（TCP）
# ---------------------------------------------------------------------------
def serve(host: str, port: int, state: mb.SlaveState) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(8)
    print(f"Modbus TCP 从站已启动：{host}:{port}")
    print("演示寄存器表：")
    print("  保持寄存器 0..31 = 1000..1031")
    print("  线圈 0..15 = 0,1,0,1…")
    print("按 Ctrl+C 退出\n")

    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=_client_loop, args=(conn, addr, state), daemon=True).start()
    except KeyboardInterrupt:
        print("\n从站关闭")
    finally:
        srv.close()


def _recv_exact(conn: socket.socket, n: int) -> bytes:
    """读满 n 字节，对端关闭返回空。MBAP 的 length 字段依赖这个。"""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf


def _client_loop(conn: socket.socket, addr, state: mb.SlaveState) -> None:
    print(f"[连接] {addr[0]}:{addr[1]}")
    try:
        while True:
            header = _recv_exact(conn, 6)
            if not header:
                break
            total = mb.tcp_frame_length(header)
            rest = _recv_exact(conn, total - 6)
            if not rest and total > 6:
                break
            frame = header + rest

            mbap = mb.parse_tcp(frame)
            resp_pdu = state.handle(mbap.pdu)
            conn.sendall(mb.build_tcp(mbap.transaction, mbap.unit, resp_pdu))
    except (mb.ModbusError, OSError) as e:
        print(f"[错误] {e}")
    finally:
        conn.close()
        print(f"[断开] {addr[0]}:{addr[1]}")


# ---------------------------------------------------------------------------
# 主站（客户端）
# ---------------------------------------------------------------------------
class Client:
    def __init__(self, host: str, port: int, unit: int = 1, timeout: float = 3.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.unit = unit
        self.transaction = 0

    def close(self) -> None:
        self.sock.close()

    def _recv_exact(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise mb.ModbusError("连接被对端关闭")
            buf += chunk
        return buf

    def request(self, pdu: bytes) -> bytes:
        self.transaction = (self.transaction + 1) & 0xFFFF
        self.sock.sendall(mb.build_tcp(self.transaction, self.unit, pdu))

        header = self._recv_exact(6)
        total = mb.tcp_frame_length(header)
        frame = header + self._recv_exact(total - 6)
        mbap = mb.parse_tcp(frame)
        if mbap.transaction != self.transaction:
            raise mb.ModbusError(f"事务号不匹配：{mbap.transaction} != {self.transaction}")
        return mbap.pdu


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def cmd_serve(args) -> int:
    serve(args.host, args.port, mb.SlaveState.demo())
    return 0


def cmd_read_holding(args) -> int:
    c = Client(args.host, args.port, args.unit)
    try:
        values = mb.parse_response(c.request(mb.req_read_holding(args.start, args.count)))
        print(f"保持寄存器 {args.start}~{args.start + args.count - 1}：")
        for i, v in enumerate(values):
            print(f"  [{args.start + i:>4}] = {v}  (0x{v:04X})")
    finally:
        c.close()
    return 0


def cmd_read_coils(args) -> int:
    c = Client(args.host, args.port, args.unit)
    try:
        bits = mb.parse_response(c.request(mb.req_read_coils(args.start, args.count)), expect_bits=args.count)
        print(f"线圈 {args.start}~{args.start + args.count - 1}：")
        print("  " + "".join(str(b) for b in bits))
    finally:
        c.close()
    return 0


def cmd_write_register(args) -> int:
    c = Client(args.host, args.port, args.unit)
    try:
        print(mb.parse_response(c.request(mb.req_write_register(args.start, args.value))))
    finally:
        c.close()
    return 0


def cmd_write_registers(args) -> int:
    values = [int(v, 0) for v in args.values.split(",")]
    c = Client(args.host, args.port, args.unit)
    try:
        print(mb.parse_response(c.request(mb.req_write_multi_registers(args.start, values))))
    finally:
        c.close()
    return 0


def cmd_write_coil(args) -> int:
    c = Client(args.host, args.port, args.unit)
    try:
        print(mb.parse_response(c.request(mb.req_write_coil(args.start, args.on == "1"))))
    finally:
        c.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="modbus_cli", description="Modbus TCP 从站模拟器与客户端")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_conn(p):
        p.add_argument("--host", default="127.0.0.1")
        p.add_argument("--port", type=int, default=1502)
        p.add_argument("--unit", type=int, default=1)

    p = sub.add_parser("serve", help="启动从站模拟器")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=1502)
    p.set_defaults(func=cmd_serve)

    p = sub.add_parser("read-holding", help="读保持寄存器 (FC03)")
    add_conn(p)
    p.add_argument("--start", type=int, default=0)
    p.add_argument("--count", type=int, default=4)
    p.set_defaults(func=cmd_read_holding)

    p = sub.add_parser("read-coils", help="读线圈 (FC01)")
    add_conn(p)
    p.add_argument("--start", type=int, default=0)
    p.add_argument("--count", type=int, default=8)
    p.set_defaults(func=cmd_read_coils)

    p = sub.add_parser("write-register", help="写单个寄存器 (FC06)")
    add_conn(p)
    p.add_argument("--start", type=int, required=True)
    p.add_argument("--value", type=int, required=True)
    p.set_defaults(func=cmd_write_register)

    p = sub.add_parser("write-registers", help="写多个寄存器 (FC16)")
    add_conn(p)
    p.add_argument("--start", type=int, required=True)
    p.add_argument("--values", required=True, help='逗号分隔，如 "1,2,3"')
    p.set_defaults(func=cmd_write_registers)

    p = sub.add_parser("write-coil", help="写单个线圈 (FC05)")
    add_conn(p)
    p.add_argument("--start", type=int, required=True)
    p.add_argument("--on", choices=["0", "1"], required=True)
    p.set_defaults(func=cmd_write_coil)

    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except mb.ModbusException as e:
        print(f"[从站返回异常] {e}")
        return 2
    except (mb.ModbusError, OSError) as e:
        print(f"[错误] {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
