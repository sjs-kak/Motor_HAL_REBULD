"""Execute production C parser, RC state machine and motion ramp on the host.

Windows: clang + lld (RC_TEST_CLANG can override the detected compiler).
Linux/macOS: a host C compiler (CC). No third-party Python packages required.
No serial port, probe or target board is touched.
"""
import ctypes as C
import os
import re
from pathlib import Path
import shutil
import struct
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


def channel_index(name):
    config = (ROOT / "Config/Inc/cfg_rc.h").read_text(encoding="utf-8")
    match = re.search(r"^\s*#define\s+" + name + r"\s+(\d+)U?\b", config, re.MULTILINE)
    if not match:
        raise ValueError("Missing numeric channel config: " + name)
    return int(match.group(1)) - 1


THROTTLE = channel_index("CFG_RC_THROTTLE_CH")
STEERING = channel_index("CFG_RC_STEERING_CH")
ENABLE = channel_index("CFG_RC_ENABLE_CH")
OUT = ROOT / "build" / "host-tests"
MASK = 0xFFFFFFFF
WAIT_SIGNAL, DISARMED, WAIT_NEUTRAL, STOPPED, DRIVING, FAILSAFE = range(6)


class Frame(C.Structure):
    _fields_ = [("channel", C.c_uint16 * 14), ("received_ms", C.c_uint32), ("sequence", C.c_uint32)]


class Stats(C.Structure):
    _fields_ = [("valid_frames", C.c_uint32), ("bad_checksums", C.c_uint32), ("discarded_bytes", C.c_uint32)]


class Status(C.Structure):
    _fields_ = [("state", C.c_int), ("throttle_us", C.c_uint16), ("steering_us", C.c_uint16),
                ("enable_us", C.c_uint16), ("throttle", C.c_int32), ("steering", C.c_int32),
                ("target_left", C.c_int32), ("target_right", C.c_int32),
                ("frame_age_ms", C.c_uint32), ("need_enable_low", C.c_uint8)]


def build_library():
    OUT.mkdir(parents=True, exist_ok=True)
    sources = [ROOT / p for p in ("Service/Src/ibus.c", "App/Src/rc_control.c",
                                  "App/Src/motion.c", "Bsp/Src/bsp_motor_pulse.c", "Bsp/Src/bsp_uart.c",
                                  "Service/Src/protocol.c", "tests/rc_test_support.c")]
    includes = ["-I" + str(ROOT / p) for p in ("tests/stubs", "Config/Inc", "Osal/Inc", "Bsp/Inc", "Service/Inc", "App/Inc")]
    flags = ["-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", "-fno-builtin", "-fno-stack-protector"]
    if os.name == "nt":
        compiler = os.environ.get("RC_TEST_CLANG") or shutil.which("clang")
        if not compiler:
            bundled = Path(r"D:\Matlab\toolbox\sldrt\clang\win64\clang.exe")
            if bundled.is_file():
                compiler = str(bundled)
        if not compiler:
            raise RuntimeError("Set RC_TEST_CLANG to a host clang.exe with lld.exe alongside it")
        objects = []
        for source in sources:
            obj = OUT / (source.stem + ".obj")
            subprocess.run([compiler, *flags, "-ffreestanding", *includes, "-c", str(source), "-o", str(obj)], check=True)
            objects.append(str(obj))
        library = OUT / "rc_control_tests.dll"
        exports = ["Test_SetTick", "Test_GetFrequency", "Test_GetCen", "Test_GetCcer", "Ibus_Init", "Ibus_FeedByte",
                   "Ibus_GetLatest", "Ibus_GetStats", "Motion_Init", "Motion_SetAccel", "Motion_GetSpeed",
                   "Motion_SetTarget", "Motion_SlewTick", "Motion_StopImmediate", "RcControl_Init",
                   "RcControl_Tick", "RcControl_GetStatus", "Test_UartInit", "Test_UartInject", "Test_UartError",
                   "Bsp_Uart_RcGetErrorCount", "Protocol_TakeFrame", "Bsp_MotorPulse_SetFreq",
                   "Test_ResetPulseModel", "Test_AdvancePulseUs", "Test_GetPulseEdges", "Motion_Move", "Test_GetDir"]
        linker = Path(compiler).with_name("lld.exe")
        subprocess.run([str(linker), "-flavor", "link", "/DLL", "/NOENTRY", "/NODEFAULTLIB",
                        "/OUT:" + str(library), *objects, *["/EXPORT:" + name for name in exports]], check=True)
    else:
        compiler = os.environ.get("CC", "cc")
        library = OUT / "librc_control_tests.so"
        subprocess.run([compiler, *flags, "-shared", "-fPIC", *includes, *map(str, sources), "-o", str(library)], check=True)
    lib = C.CDLL(str(library))
    for name in ("Test_SetTick", "Motion_SetAccel", "Motion_SlewTick", "RcControl_Init", "RcControl_Tick"):
        getattr(lib, name).argtypes = [C.c_uint32]
    lib.Ibus_FeedByte.argtypes = [C.c_uint8]
    lib.Ibus_GetLatest.argtypes = [C.POINTER(Frame)]
    lib.Ibus_GetLatest.restype = C.c_uint8
    lib.Ibus_GetStats.argtypes = [C.POINTER(Stats)]
    lib.RcControl_GetStatus.argtypes = [C.POINTER(Status)]
    lib.Motion_SetTarget.argtypes = [C.c_int32, C.c_int32]
    lib.Motion_Move.argtypes = [C.c_int32, C.c_int32]
    lib.Motion_GetSpeed.argtypes = [C.POINTER(C.c_int32), C.POINTER(C.c_int32)]
    lib.Test_GetFrequency.argtypes = [C.c_uint32]
    lib.Test_GetFrequency.restype = C.c_uint32
    lib.Bsp_MotorPulse_SetFreq.argtypes = [C.c_int, C.c_uint32]
    lib.Test_AdvancePulseUs.argtypes = [C.c_uint32, C.c_uint32]
    lib.Test_GetPulseEdges.argtypes = [C.c_uint32]
    lib.Test_GetPulseEdges.restype = C.c_uint32
    lib.Test_GetDir.argtypes = [C.c_uint32]
    lib.Test_GetDir.restype = C.c_uint32
    lib.Test_UartInject.argtypes = [C.c_uint32, C.c_uint8]
    lib.Test_UartError.argtypes = [C.c_uint32]
    lib.Protocol_TakeFrame.argtypes = [C.c_char_p, C.c_uint16, C.POINTER(C.c_uint16)]
    lib.Protocol_TakeFrame.restype = C.c_uint16
    return lib


LIB = None


def packet(channels):
    payload = b"\x20\x40" + struct.pack("<14H", *channels)
    return payload + struct.pack("<H", 0xFFFF - sum(payload))


class RemoteTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.reset()

    def reset(self):
        LIB.Test_SetTick(self.now & MASK)
        LIB.Ibus_Init()
        LIB.Motion_Init()
        LIB.Motion_SetAccel(2000)
        LIB.RcControl_Init(self.now & MASK)
        LIB.Test_UartInit()
        self.channels = [1500] * 14
        self.channels[ENABLE] = 1000

    def feed(self, data, at=None):
        if at is not None:
            self.now = at
        LIB.Test_SetTick(self.now & MASK)
        for byte in data:
            LIB.Ibus_FeedByte(byte)

    def frame(self):
        result = Frame()
        available = LIB.Ibus_GetLatest(C.byref(result))
        return available, result

    def stats(self):
        result = Stats()
        LIB.Ibus_GetStats(C.byref(result))
        return result

    def status(self):
        result = Status()
        LIB.RcControl_GetStatus(C.byref(result))
        return result

    def speeds(self):
        left, right = C.c_int32(), C.c_int32()
        LIB.Motion_GetSpeed(C.byref(left), C.byref(right))
        return left.value, right.value

    def tick(self, dt=10, send=True):
        self.now += dt
        LIB.Test_SetTick(self.now & MASK)
        if send:
            self.feed(packet(self.channels))
        LIB.RcControl_Tick(self.now & MASK)

    def arm(self):
        self.channels[THROTTLE] = self.channels[STEERING] = 1500
        self.channels[ENABLE] = 1000
        self.tick()
        self.channels[ENABLE] = 2000
        for _ in range(51):
            self.tick()
        self.assertEqual(self.status().state, STOPPED)
        self.assertEqual(self.speeds(), (0, 0))

    def drive(self):
        self.arm()
        self.channels[THROTTLE] = 2000
        for _ in range(130):
            self.tick()
        self.assertEqual(self.speeds(), (2500, 2500))

    def test_published_protocol_vector(self):
        # Published i-BUS format example; independent of our packet generator.
        self.feed(bytes.fromhex("20 40 DB 05 DC 05 54 05 DC 05 E8 03 D0 07 D2 05 E8 03 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DA F3"))
        available, frame = self.frame()
        self.assertTrue(available)
        self.assertEqual(list(frame.channel[:6]), [1499, 1500, 1364, 1500, 1000, 2000])

    def test_checksum_error_does_not_refresh_watchdog(self):
        self.feed(packet(self.channels), 20)
        bad = bytearray(packet(self.channels))
        bad[10] ^= 1
        self.feed(bad, 100)
        available, frame = self.frame()
        self.assertTrue(available)
        self.assertEqual((frame.sequence, frame.received_ms), (1, 20))
        self.assertEqual(self.stats().bad_checksums, 1)

    def test_noise_and_back_to_back_frames(self):
        self.feed(b"\x01\x40\xff\x20\x21\x20\x20")
        for value in range(1000, 1100):
            self.channels[THROTTLE] = value
            self.feed(packet(self.channels))
        self.assertEqual(self.frame()[1].channel[THROTTLE], 1099)
        self.assertEqual(self.stats().valid_frames, 100)

    def test_dropped_inserted_bytes_and_embedded_header_recover(self):
        self.channels[7] = 0x4020
        good = packet(self.channels)
        for damaged in (good[:9] + good[10:], good[:8] + b"\x20" + good[8:]):
            self.feed(damaged + good + good)
            self.assertEqual(self.frame()[1].channel[7], 0x4020)
        self.assertGreater(self.stats().bad_checksums, 0)

    def test_partial_frame_gap_resets_and_recovers(self):
        good = packet(self.channels)
        self.feed(good[:15], 10)
        self.feed(good[15:], 20)
        self.assertFalse(self.frame()[0])
        self.feed(good, 30)
        self.assertEqual(self.stats().valid_frames, 1)

    def test_startup_requires_enable_low_then_neutral(self):
        self.channels[ENABLE] = self.channels[THROTTLE] = 2000
        for _ in range(70):
            self.tick()
        self.assertEqual(self.status().state, DISARMED)
        self.assertEqual(self.speeds(), (0, 0))
        self.channels[ENABLE] = 1000
        self.tick()
        self.channels[ENABLE] = 2000
        for _ in range(70):
            self.tick()
        self.assertEqual(self.status().state, WAIT_NEUTRAL)
        self.assertEqual(self.speeds(), (0, 0))

    def test_both_sticks_must_remain_neutral(self):
        self.tick()
        self.channels[ENABLE] = 2000
        for _ in range(30):
            self.tick()
        self.channels[STEERING] = 1800
        self.tick()
        self.channels[STEERING] = 1500
        for _ in range(50):
            self.tick()
        self.assertEqual(self.status().state, WAIT_NEUTRAL)
        self.tick()
        self.assertEqual(self.status().state, STOPPED)

    def test_single_frame_cannot_arm_and_continuity_required(self):
        self.tick()
        self.channels[ENABLE] = 2000
        self.tick()
        for _ in range(20):
            self.tick(send=False)
        self.assertEqual(self.status().state, FAILSAFE)
        self.assertEqual(self.speeds(), (0, 0))
        self.arm()

    def test_forward_slew_and_stop(self):
        self.arm()
        self.channels[THROTTLE] = 2000
        previous = 0
        for _ in range(130):
            self.tick()
            left, right = self.speeds()
            self.assertEqual(left, right)
            self.assertLessEqual(left - previous, 20)
            self.assertLessEqual(left, 2500)
            previous = left
        self.assertEqual(self.speeds(), (2500, 2500))
        self.channels[THROTTLE] = 1500
        for _ in range(130):
            self.tick()
        self.assertEqual(self.status().state, STOPPED)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertEqual(LIB.Test_GetCen(0), 0)
        self.assertEqual(LIB.Test_GetCen(1), 0)

    def test_turning_is_bounded_and_pivot_at_zero_throttle_is_limited(self):
        self.arm()
        self.channels[STEERING] = 2000
        self.tick()
        self.assertEqual((self.status().target_left, self.status().target_right), (600, -600))
        self.channels[STEERING] = 1000
        self.tick()
        self.assertEqual((self.status().target_left, self.status().target_right), (-600, 600))
        self.channels[STEERING] = 2000
        self.channels[THROTTLE] = 2000
        self.tick()
        self.assertEqual((self.status().target_left, self.status().target_right), (2500, 0))
        self.channels[STEERING] = 1000
        self.tick()
        self.assertEqual((self.status().target_left, self.status().target_right), (0, 2500))
        self.channels[THROTTLE] = 1000
        self.channels[STEERING] = 2000
        self.tick()
        self.assertEqual((self.status().target_left, self.status().target_right), (0, -2500))

    def test_pivot_center_stops_and_disable_gates_both_sides(self):
        self.arm()
        self.channels[STEERING] = 2000
        for _ in range(40):
            self.tick()
        self.assertEqual(self.speeds(), (600, -600))
        self.channels[STEERING] = 1500
        for _ in range(40):
            self.tick()
        self.assertEqual(self.status().state, STOPPED)
        self.assertEqual(self.speeds(), (0, 0))
        self.channels[STEERING] = 1000
        for _ in range(40):
            self.tick()
        self.assertEqual(self.speeds(), (-600, 600))
        self.channels[ENABLE] = 1000
        self.tick()
        self.assertEqual(self.status().state, DISARMED)
        for side in (0, 1):
            self.assertEqual(LIB.Test_GetCen(side), 0)
            self.assertEqual(LIB.Test_GetCcer(side) & 17, 0)

    def test_pivot_reverse_crosses_zero_on_each_side(self):
        self.arm()
        self.channels[STEERING] = 2000
        for _ in range(40):
            self.tick()
        previous = self.speeds()
        saw_zero = [False, False]
        self.channels[STEERING] = 1000
        for _ in range(70):
            self.tick()
            current = self.speeds()
            for side in (0, 1):
                self.assertLessEqual(abs(current[side] - previous[side]), 20)
                if current[side] == 0:
                    saw_zero[side] = True
                if current[side] * (1 if side == 0 else -1) < 0:
                    self.assertTrue(saw_zero[side])
            previous = current
        self.assertEqual(previous, (-600, 600))

    def test_front_rear_directions_match_on_each_side_in_both_gears(self):
        LIB.Motion_Move(600, 600)
        self.assertEqual([LIB.Test_GetDir(i) for i in range(4)], [1, 1, 0, 0])
        LIB.Motion_Move(-600, -600)
        self.assertEqual([LIB.Test_GetDir(i) for i in range(4)], [0, 0, 1, 1])

    def test_this_transmitter_captured_ch3_reverse_sample(self):
        self.arm()
        # Captured while the user held the right stick down; CH2 stayed centered.
        self.channels[:6] = [1526, 1509, 1135, 1528, 2000, 2000]
        self.tick()
        self.assertEqual(self.status().throttle_us, 1135)
        self.assertEqual((self.status().target_left, self.status().target_right), (-1750, -1750))
        self.channels[1] = 2000  # CH2 is no longer the throttle on this model.
        self.tick()
        self.assertEqual((self.status().target_left, self.status().target_right), (-1750, -1750))

    def test_reversal_visits_zero_before_negative(self):
        self.drive()
        self.channels[THROTTLE] = 1000
        saw_zero = False
        previous = 2500
        for _ in range(260):
            self.tick()
            left, right = self.speeds()
            self.assertEqual(left, right)
            self.assertLessEqual(abs(left - previous), 20)
            if left == 0:
                saw_zero = True
            if left < 0:
                self.assertTrue(saw_zero)
            previous = left
        self.assertEqual(self.speeds(), (-2500, -2500))

    def test_enable_low_stops_immediately_and_clears_target(self):
        self.drive()
        self.channels[ENABLE] = 1000
        self.tick()
        self.assertEqual(self.speeds(), (0, 0))
        self.assertEqual((self.status().target_left, self.status().target_right), (0, 0))
        self.assertEqual(self.status().state, DISARMED)
        self.channels[ENABLE] = 2000
        for _ in range(60):
            self.tick()
        self.assertEqual(self.status().state, WAIT_NEUTRAL)
        self.assertEqual(self.speeds(), (0, 0))

    def test_timeout_and_recovery_require_enable_cycle(self):
        self.drive()
        self.tick(199, send=False)
        self.assertEqual(self.status().state, DRIVING)
        self.tick(1, send=False)
        self.assertEqual(self.status().state, FAILSAFE)
        self.assertEqual(self.speeds(), (0, 0))
        self.channels[THROTTLE] = 1500
        for _ in range(60):
            self.tick()
        self.assertEqual(self.status().state, DISARMED)
        self.assertTrue(self.status().need_enable_low)
        self.arm()

    def test_configured_receiver_failsafe_stops_with_live_frames(self):
        self.drive()
        before = self.stats().valid_frames
        self.channels[ENABLE] = 1000  # Receiver keeps transmitting, but CH5 failsafe is low.
        self.tick()
        self.assertGreater(self.stats().valid_frames, before)
        self.assertEqual(self.status().state, DISARMED)
        self.assertEqual(self.speeds(), (0, 0))

    def test_invalid_control_channel_stops_even_with_good_checksum(self):
        self.drive()
        self.channels[THROTTLE] = 0
        self.tick()
        self.assertEqual(self.status().state, FAILSAFE)
        self.assertEqual(self.speeds(), (0, 0))

    def test_unused_channels_can_be_zero(self):
        self.channels[6:] = [0] * 8
        self.arm()

    def test_timestamp_wrap_and_future_snapshot_tick(self):
        self.now = 0xFFFFFF00
        self.reset()
        self.arm()
        self.channels[THROTTLE] = 2000
        self.tick()
        LIB.RcControl_Tick((self.now - 1) & MASK)
        self.assertNotEqual(self.status().state, FAILSAFE)
        self.tick(200, send=False)
        self.assertEqual(self.status().state, FAILSAFE)

    def test_middle_enable_is_not_startup_low_acknowledgement(self):
        self.channels[ENABLE] = 1500
        self.tick()
        self.channels[ENABLE] = 2000
        for _ in range(60):
            self.tick()
        self.assertTrue(self.status().need_enable_low)
        self.assertEqual(self.speeds(), (0, 0))

    def test_delayed_motion_tick_has_bounded_step(self):
        LIB.Motion_SetTarget(2500, 2500)
        LIB.Test_SetTick(1000)
        LIB.Motion_SlewTick(1000)
        self.assertEqual(self.speeds(), (40, 40))
        LIB.Motion_StopImmediate()
        LIB.Test_SetTick(1010)
        LIB.Motion_SlewTick(1010)
        self.assertEqual(self.speeds(), (0, 0))

    def test_hardware_counters_and_channels_stop_after_pwm_enabled(self):
        self.drive()
        for side in (0, 1):
            self.assertEqual(LIB.Test_GetCen(side), 1)
            self.assertEqual(LIB.Test_GetCcer(side) & 17, 17)
        LIB.Motion_StopImmediate()
        for side in (0, 1):
            self.assertEqual(LIB.Test_GetCen(side), 0)
            self.assertEqual(LIB.Test_GetCcer(side) & 17, 0)

    def test_starting_one_side_keeps_other_stopped(self):
        LIB.Motion_SetTarget(2500, 0)
        LIB.Test_SetTick(10)
        LIB.Motion_SlewTick(10)
        self.assertEqual(LIB.Test_GetCen(0), 1)
        self.assertEqual(LIB.Test_GetCen(1), 0)
        self.assertEqual(LIB.Test_GetFrequency(2), 0)

    def test_two_uart_callbacks_do_not_mix_protocols(self):
        binary = packet(self.channels)
        text = b"hello\r\n"
        for i, byte in enumerate(binary):
            LIB.Test_UartInject(3, byte)
            if i < len(text):
                LIB.Test_UartInject(1, text[i])
        self.assertEqual(self.stats().valid_frames, 1)
        out = C.create_string_buffer(200)
        length = C.c_uint16()
        self.assertTrue(LIB.Protocol_TakeFrame(out, len(out), C.byref(length)))
        self.assertEqual(out.value, b"hello")
        self.assertEqual(length.value, 5)

    def test_low_frequency_updates_preserve_pwm_edges(self):
        LIB.Test_ResetPulseModel()
        for i in range(125):  # 1 second, frequent small changes at 50-55 Hz.
            for side, channel in ((0, 0), (1, 2)):
                LIB.Bsp_MotorPulse_SetFreq(channel, 50 + (i % 2) * 5)
                LIB.Test_AdvancePulseUs(side, 8000)
        for side in (0, 1):
            self.assertGreaterEqual(LIB.Test_GetPulseEdges(side), 45)
            self.assertLessEqual(LIB.Test_GetPulseEdges(side), 60)
        LIB.Motion_StopImmediate()
        before = [LIB.Test_GetPulseEdges(side) for side in (0, 1)]
        for side in (0, 1):
            LIB.Test_AdvancePulseUs(side, 100000)
            self.assertEqual(LIB.Test_GetPulseEdges(side), before[side])

    def test_small_live_rc_jitter_still_produces_low_speed_steps(self):
        self.arm()
        LIB.Test_ResetPulseModel()
        for i in range(125):
            self.channels[THROTTLE] = 1560 + (i % 2)  # Approx. 55-60 Hz after deadband.
            self.tick(8)
            for side in (0, 1):
                LIB.Test_AdvancePulseUs(side, 8000)
        self.assertEqual(self.status().state, DRIVING)
        for side in (0, 1):
            self.assertGreaterEqual(LIB.Test_GetPulseEdges(side), 45)
            self.assertLessEqual(LIB.Test_GetPulseEdges(side), 65)

    def test_uart_errors_rearm_both_ports(self):
        before = LIB.Bsp_Uart_RcGetErrorCount()
        LIB.Test_UartError(1)
        LIB.Test_UartError(3)
        self.assertEqual(LIB.Bsp_Uart_RcGetErrorCount(), before + 1)
        for byte in packet(self.channels):
            LIB.Test_UartInject(3, byte)
        self.assertEqual(self.stats().valid_frames, 1)
        for byte in b"after-error\r\n":
            LIB.Test_UartInject(1, byte)
        out = C.create_string_buffer(200)
        length = C.c_uint16()
        self.assertTrue(LIB.Protocol_TakeFrame(out, len(out), C.byref(length)))
        self.assertEqual(out.value, b"after-error")


if __name__ == "__main__":
    LIB = build_library()
    unittest.main(verbosity=2)
