#!/usr/bin/env python3
"""串口练习：实现 0x55 帧的编码器与解析器。

用法：
    python 代码/tools/serial_drill.py

只实现 Level 2 的 encode_frame()，编码用例就会通过；
再实现 Level 3 的 FrameParser，解析用例也会通过。
参考答案见 ../串口练习_答案.md。
"""

import math

HEADER = 0x55
TRAILER = 0x0D


# ---------------------------------------------------------------- Level 2
def encode_frame(freq, vib, contact, trailer=True):
    """把一次测量编码成 0x55 帧。

    参数：
        freq:    float | None，持续振动主频（Hz）
        vib:     int，0 = 停，1 = 振
        contact: int，0 = 未接触，1 = 接触
        trailer: True = 6 字节（带 0x0D），False = 5 字节

    返回：
        bytes

    规则：
        - freq 为 None / NaN / inf / <= 0 时，整数和小数都发 0
        - 频率按 0.1Hz 拆分，+0.5 四舍五入
        - 整数部分饱和到 255
    """
    # TODO(Level 2): 在这里实现
    raise NotImplementedError("TODO: 实现 encode_frame()")


# ---------------------------------------------------------------- Level 3
class FrameParser:
    """流式解析器：feed() 可以喂任意长度的数据块。"""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data):
        """喂入字节，返回本次能解析出的记录列表。

        记录格式：
            {"freq": 165.2, "vib": 1, "contact": 1}

        规则：
            - 帧前允许有垃圾字节
            - 帧尾不是 0x0D 时丢弃，并继续向后找
            - 不完整帧留在缓冲区
            - fi == 0 且 fd == 0 时，freq 返回 None
        """
        # TODO(Level 3): 在这里实现
        raise NotImplementedError("TODO: 实现 FrameParser.feed()")


# ---------------------------------------------------------------- 测试
def _hex(data):
    return " ".join(f"{b:02X}" for b in data)


def _same_record(got, freq, vib, contact):
    if not isinstance(got, dict):
        return False
    gf = got.get("freq")
    if freq is None:
        return gf is None and got.get("vib") == vib and got.get("contact") == contact
    return (
        isinstance(gf, (int, float))
        and abs(float(gf) - freq) < 1e-6
        and got.get("vib") == vib
        and got.get("contact") == contact
    )


def run_tests():
    results = []

    def check(name, fn):
        try:
            fn()
            results.append((name, True, ""))
        except AssertionError as exc:
            results.append((name, False, str(exc) or "assert failed"))
        except NotImplementedError:
            results.append((name, False, "还没实现"))
        except Exception as exc:  # noqa: BLE001
            results.append((name, False, f"{type(exc).__name__}: {exc}"))

    # ---------------- 编码用例 ----------------
    def enc(freq, vib, contact, trailer, want):
        def case():
            got = encode_frame(freq, vib, contact, trailer=trailer)
            assert isinstance(got, (bytes, bytearray)), f"应返回 bytes，实际 {type(got)}"
            assert bytes(got) == want, f"期望 {_hex(want)}，实际 {_hex(bytes(got))}"

        return case

    check("enc 165.2 / 振动 / 接触",
          enc(165.2, 1, 1, True, b"\x55\xA5\x02\x01\x01\x0D"))
    check("enc 80.0 / 振动 / 未接触",
          enc(80.0, 1, 0, True, b"\x55\x50\x00\x01\x00\x0D"))
    check("enc 179.9 / 振动 / 接触",
          enc(179.9, 1, 1, True, b"\x55\xB3\x09\x01\x01\x0D"))
    check("enc 无效频率 -> 0,0",
          enc(None, 0, 0, True, b"\x55\x00\x00\x00\x00\x0D"))
    check("enc NaN -> 0,0",
          enc(float("nan"), 0, 0, True, b"\x55\x00\x00\x00\x00\x0D"))
    check("enc 99.99 -> 100,0",
          enc(99.99, 1, 0, True, b"\x55\x64\x00\x01\x00\x0D"))
    check("enc 165.24 -> 165,2",
          enc(165.24, 1, 1, True, b"\x55\xA5\x02\x01\x01\x0D"))
    check("enc 165.26 -> 165,3",
          enc(165.26, 1, 1, True, b"\x55\xA5\x03\x01\x01\x0D"))
    check("enc 300.0 -> 255,0",
          enc(300.0, 1, 0, True, b"\x55\xFF\x00\x01\x00\x0D"))
    check("enc 5 字节（无帧尾）",
          enc(165.2, 1, 1, False, b"\x55\xA5\x02\x01\x01"))

    # ---------------- 解析用例 ----------------
    def parse(chunks, want):
        def case():
            p = FrameParser()
            got = []
            for chunk in chunks:
                out = p.feed(chunk)
                assert isinstance(out, list), f"feed() 应返回 list，实际 {type(out)}"
                got.extend(out)
            assert len(got) == len(want), f"期望 {len(want)} 条，实际 {len(got)} 条: {got}"
            for i, (freq, vib, contact) in enumerate(want):
                assert _same_record(got[i], freq, vib, contact), \
                    f"第 {i + 1} 条不匹配: {got[i]}"

        return case

    good = b"\x55\xA5\x02\x01\x01\x0D"
    check("parse 单帧", parse([good], [(165.2, 1, 1)]))
    check("parse 两帧粘连", parse([good + good], [(165.2, 1, 1), (165.2, 1, 1)]))
    check("parse 逐字节喂", parse([bytes([b]) for b in good], [(165.2, 1, 1)]))
    check("parse 帧前有垃圾", parse([b"\xAA\xBB" + good], [(165.2, 1, 1)]))
    check("parse 坏帧尾后重新同步",
          parse([b"\x55\xA5\x02\x01\x01\x00" + good], [(165.2, 1, 1)]))
    check("parse 85Hz（数据里有 0x55）",
          parse([b"\x55\x55\x00\x01\x00\x0D"], [(85.0, 1, 0)]))
    check("parse 重复帧头",
          parse([b"\x55\x55\xA5\x02\x01\x01\x0D"], [(165.2, 1, 1)]))
    check("parse 半帧 + 补齐",
          parse([b"\x55\xA5\x02", b"\x01\x01\x0D"], [(165.2, 1, 1)]))

    def incomplete_keeps_buffer():
        p = FrameParser()
        assert p.feed(b"\x55\xA5\x02") == []
        assert p.feed(b"") == []
        assert p.feed(b"\x01\x01") == []       # 还差 0x0D
        out = p.feed(b"\x0D")
        assert len(out) == 1 and _same_record(out[0], 165.2, 1, 1), out

    check("parse 不完整帧保留缓冲", incomplete_keeps_buffer)

    # ---------------- 输出 ----------------
    passed = sum(1 for _, ok, _ in results if ok)
    failed = len(results) - passed
    for name, ok, msg in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  <- {msg}" if msg else ""))
    print()
    print(f"serial_drill: {passed} pass / {failed} fail")
    return passed, failed


if __name__ == "__main__":
    _, failed = run_tests()
    raise SystemExit(1 if failed else 0)
