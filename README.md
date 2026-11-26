# modbus-toolkit

[![tests](https://github.com/zyh1901519038-ops/modbus-toolkit/actions/workflows/tests.yml/badge.svg)](https://github.com/zyh1901519038-ops/modbus-toolkit/actions/workflows/tests.yml)

Modbus 的 RTU 和 TCP 两种封帧，加一个从站模拟器和一个命令行客户端。纯 Python 标准库。

一开始是为了搞清楚「RTU 和 TCP 到底差在哪」才写的。写下来发现 PDU 那层是一套东西，区别只在外层怎么封帧。

## 支持的部分

| 层 | 内容 |
|---|---|
| RTU | 从站地址 + PDU + CRC16（0xA001，初值 0xFFFF，低字节在前） |
| TCP | MBAP 头（事务号、协议号、长度、单元号）+ PDU |
| 功能码 | 0x01 读线圈、0x03 读保持寄存器、0x05 写单线圈、0x06 写单寄存器、0x10 写多寄存器 |
| 异常 | 0x80 \| fc + 异常码（非法功能码、非法地址、非法数值） |

另外带一个 TCP 从站模拟器（内置演示用的寄存器和线圈），和一个能读写寄存器的命令行主站。

## 用法

先起从站：

```bash
python modbus_cli.py serve --port 1502
```

演示数据是寄存器 0..31 存 1000..1031，线圈 0..15 交替真假。

再开一个终端读寄存器：

```bash
python modbus_cli.py read-holding --port 1502 --start 0 --count 4
```

```
保持寄存器 0~3：
  [   0] = 1000  (0x03E8)
  [   1] = 1001  (0x03E9)
  [   2] = 1002  (0x03EA)
  [   3] = 1003  (0x03EB)
```

写单寄存器再读回来：

```
$ python modbus_cli.py write-register --port 1502 --start 0 --value 1234
已写入 地址=0 值=1234

$ python modbus_cli.py read-holding --port 1502 --start 0 --count 2
保持寄存器 0~1：
  [   0] = 1234  (0x04D2)
  [   1] = 1001  (0x03E9)
```

批量写：

```
$ python modbus_cli.py write-registers --port 1502 --start 10 --values "7,8,9"
已写入 3 个寄存器，起始地址=10
```

读线圈：

```
$ python modbus_cli.py read-coils --port 1502 --start 0 --count 8
线圈 0~7：
  01010101
```

地址越界会收到异常响应，客户端会把异常码翻译出来：

```
$ python modbus_cli.py read-holding --port 1502 --start 9999 --count 1
[从站返回异常] 功能码 0x03 返回异常 0x02：非法数据地址
```

## 测试

```bash
python test_modbus.py
```

23 个用例。CRC 用的是标准向量 `01 03 00 00 00 0A → 0xCDC5`，逐位实现算出来能对上。除了编解码往返，还有三个是起真从站线程、用真 socket 连上去读写的，异常响应也验了。

## 几个实现上的选择

**PDU 只实现一次**。RTU 和 TCP 的 PDU 完全一样，所以封帧拆成两个函数，从站只处理 PDU。这样加传输方式不用动业务逻辑。

**TCP 读帧必须先看 MBAP 的 length 字段**。TCP 是流式的，一次 recv 可能拿到半个帧，也可能两个帧粘一起。做法是先读 6 字节头，从 length 算出整帧长度，再补齐剩下的，拼完整才解析。

**CRC16 有几个容易写错的点**：多项式是 0xA001（0x8005 的位反向），初值 0xFFFF，结果不异或，发到线上时低字节在前。

**线圈响应里没有「数量」这个字段**。响应格式是 `[功能码][字节数][数据]`，主站得按自己请求的数量去裁剪。这个我一开始写错了。

## 踩过的坑

**把「字节数」当成「位数」用了**。读线圈的响应只有一个字节数字段，我直接拿它当 bit 数传给解包函数，结果请求 8 个线圈只解析出 1 个。后来改成 `字节数 × 8` 拿到全部位，再按请求数量裁。这个是单元测试抓出来的——断言 `[0,1,0,1]` 实际只返回了 `[0]`。

## 目录

```
modbus.py       协议栈：CRC、RTU/TCP 封帧、PDU 构造解析、从站状态机
modbus_cli.py   命令行
test_modbus.py  测试，含真实 socket 联调
```

## 许可

MIT
