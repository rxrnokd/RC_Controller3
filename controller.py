#!/usr/bin/env python3
"""Raspberry Pi 4 RC controller. Usage and calibration: README.md."""
import argparse
from collections import deque
import contextlib
import copy
import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import re
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent


def number(value, low, high, name="value"):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name}: {low}..{high} 범위의 유한한 숫자가 필요합니다")
    return value


def clamp(x, low, high):
    return max(low, min(high, x))


def approach(x, target, step):
    return x + clamp(target - x, -step, step)


def validate_config(c):
    pins = []
    def pin(value):
        if type(value) is not int or not 0 <= value <= 27:
            raise ValueError("GPIO는 BCM 정수 0..27이어야 합니다")
        pins.append(value)
    def boolean(value):
        if type(value) is not bool:
            raise ValueError("확인/활성화 설정은 true 또는 false여야 합니다")
    boolean(c["hardware_confirmed"])
    number(c["server"]["port"], 1024, 65535, "server.port")
    if type(c["server"]["port"]) is not int or not isinstance(c["server"]["host"], str):
        raise ValueError("서버 host/port 형식 오류")
    ctrl = c["control"]
    bounds = {"hz": (20, 100), "command_timeout_s": (.2, .5), "max_rpm": (1, 280),
              "max_duty": (.01, 1), "ramp_rpm_s": (1, 400), "kp": (0, .1),
              "ki": (0, .2), "rpm_filter_s": (.02, .5), "stall_timeout_s": (.2, 2),
              "reversal_pause_s": (.2, 2)}
    for key, bounds_ in bounds.items():
        number(ctrl[key], *bounds_, key)
    if len(c["motors"]) != 2:
        raise ValueError("모터 설정은 2개가 필요합니다")
    for m in c["motors"]:
        for key in ("pwm_a", "pwm_b", "encoder_a", "encoder_b"):
            pin(m[key])
        number(m["encoder_counts_per_rev"], 1, 1000000, "counts/rev")
        number(m["wheel_diameter_mm"], 1, 1000, "wheel diameter")
        number(m["wheel_ratio"], .01, 1000, "wheel ratio")
        boolean(m["encoder_confirmed"])
        if m["motor_sign"] not in (-1, 1) or m["encoder_sign"] not in (-1, 1):
            raise ValueError("motor_sign/encoder_sign은 -1 또는 1")
    s = c["servo"]
    pin(s["gpio"])
    boolean(s["calibrated"])
    for key in ("min_us", "center_us", "max_us"):
        number(s[key], 500, 2500, key)
    if not s["min_us"] < s["center_us"] < s["max_us"]:
        raise ValueError("서보 min < center < max 필요")
    number(s["us_per_degree"], .1, 100)
    number(s["max_angle_deg"], 1, 90)
    number(s["rate_deg_s"], 1, 180)
    if s["max_angle_deg"] * s["us_per_degree"] > min(s["center_us"] - s["min_us"], s["max_us"] - s["center_us"]):
        raise ValueError("서보 명령각 범위가 펄스 제한을 초과합니다")
    mt = c["mt6701"]
    for key in ("a", "b", "z"):
        pin(mt[key])
    if mt["ppr"] is not None:
        number(mt["ppr"], 1, 1024, "MT6701 PPR")
        if type(mt["ppr"]) is not int:
            raise ValueError("MT6701 PPR은 정수입니다")
    number(mt["glitch_us"], 0, 100)
    if s["sign"] not in (-1, 1) or mt["sign"] not in (-1, 1):
        raise ValueError("sign은 -1 또는 1")
    u = c["ultrasonic"]
    boolean(u["enabled"])
    if len(u["sensors"]) != 4:
        raise ValueError("초음파 설정은 4개가 필요합니다")
    for sensor in u["sensors"]:
        pin(sensor["trig"])
        pin(sensor["echo"])
    number(u["trigger_us"], 10, 100)
    number(u["timeout_s"], .01, .04)
    number(u["slot_s"], .06, 1)
    number(u["speed_of_sound_m_s"], 300, 380)
    number(u["min_m"], .01, 1)
    number(u["max_m"], u["min_m"], 5)
    if u["max_m"] * 2 / u["speed_of_sound_m_s"] >= u["timeout_s"]:
        raise ValueError("초음파 timeout이 최대 거리 왕복 시간보다 짧습니다")
    if len(pins) != len(set(pins)):
        raise ValueError("중복 GPIO가 있습니다")
    l = c["lidar"]
    boolean(l["enabled"])
    number(l["baud"], 9600, 2000000)
    number(l["min_m"], .01, 1)
    number(l["max_m"], l["min_m"], 12)
    number(l["min_quality"], 0, 255)
    number(l["angle_offset_deg"], -360, 360)
    number(l["stale_s"], .3, 5)
    for key in ("executable", "port"):
        if not isinstance(l[key], str) or not l[key]:
            raise ValueError("LiDAR 경로 설정 오류")
    r = c["recording"]
    boolean(r["enabled"])
    boolean(r["include_lidar"])
    number(r["hz"], .2, 10)
    number(r["max_bytes"], 100000, 100000000)
    number(r["backup_count"], 1, 10)
    return c


class Quadrature:
    # A leading B is positive: 00 -> 10 -> 11 -> 01 -> 00.
    STEPS = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)

    def __init__(self, a=0, b=0, sign=1):
        self.lock = threading.Lock()
        self.state = (a << 1) | b
        self.sign = sign
        self.count = self.travel = self.invalid = self.z_count = 0
        self.zero_count = 0
        self.zeroed = False
        self.index_count = None
        self.last_edge = 0.0

    def edge(self, channel, level):
        if level not in (0, 1):
            return
        with self.lock:
            new = ((self.state & 1) | (level << 1)) if channel == 0 else ((self.state & 2) | level)
            if (new ^ self.state) == 3:
                self.invalid += 1
            step = self.STEPS[self.state * 4 + new] * self.sign
            self.count += step
            self.travel += abs(step)
            if step:
                self.last_edge = time.monotonic()
            self.state = new

    def index(self):
        with self.lock:
            self.z_count += 1
            self.index_count = self.count

    def zero(self):
        with self.lock:
            self.zero_count = self.count
            self.zeroed = True

    def snapshot(self):
        with self.lock:
            return {"count": self.count, "travel_counts": self.travel, "invalid": self.invalid,
                    "a": self.state >> 1, "b": self.state & 1, "z_count": self.z_count,
                    "index_count": self.index_count, "zero_count": self.zero_count,
                    "zeroed": self.zeroed, "last_edge": self.last_edge}


class PI:
    def __init__(self, kp, ki):
        self.kp, self.ki, self.integral = kp, ki, 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, target, measured, dt, limit):
        if abs(target) < .01:
            self.reset()
            return 0.0
        direction = 1 if target > 0 else -1
        error = abs(target) - measured * direction
        proposed = clamp(self.integral + self.ki * error * dt, 0, limit)
        raw = self.kp * error + proposed
        # Conditional integration: prevent wind-up at both limits.
        if 0 <= raw <= limit or (raw > limit and error < 0) or (raw < 0 and error > 0):
            self.integral = proposed
        return direction * clamp(self.kp * error + self.integral, 0, limit)


class Hardware:
    def __init__(self, c):
        import pigpio
        self.pigpio = pigpio
        self.c, self.callbacks, self.script = c, [], None
        self.heartbeat = 0
        self.watchdog_check = 0.0
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod에 연결하지 못했습니다. README의 시작 명령을 확인하세요")
        # Avoid a wedged daemon blocking cleanup/control indefinitely.
        self.pi.sl.s.settimeout(.2)
        try:
            for m in c["motors"]:
                for pin in (m["pwm_a"], m["pwm_b"]):
                    self.pi.set_mode(pin, pigpio.OUTPUT)
                    self.pi.set_PWM_dutycycle(pin, 0)
                    self.pi.set_PWM_range(pin, 1000)
                    self.pi.set_PWM_frequency(pin, 1000)
            self.pi.set_servo_pulsewidth(c["servo"]["gpio"], 0)
            self.encoders = [self._encoder(m["encoder_a"], m["encoder_b"], m["encoder_sign"], 0)
                             for m in c["motors"]]
            mt = c["mt6701"]
            self.mt = self._encoder(mt["a"], mt["b"], mt["sign"], mt["glitch_us"])
            self.pi.set_mode(mt["z"], pigpio.INPUT)
            self.pi.set_pull_up_down(mt["z"], pigpio.PUD_DOWN)
            self.callbacks.append(self.pi.callback(mt["z"], pigpio.RISING_EDGE, lambda g, l, t: self.mt.index()))
            # Runs in pigpiod, not Python. Repeatedly forces zero after 500 ms
            # without a changed heartbeat, including after SIGKILL.
            zeros = " ".join(f"p {m[k]} 0" for m in c["motors"] for k in ("pwm_a", "pwm_b"))
            script = (f"ld v0 0 ld v1 0 tag 0 mils 20 lda p0 cmp v0 jnz 1 "
                      f"inr v1 lda v1 cmp 25 jm 0 {zeros} s {c['servo']['gpio']} 0 jmp 0 "
                      "tag 1 ld v0 p0 ld v1 0 jmp 0")
            self.script = self.pi.store_script(script)
            deadline = time.monotonic() + 1
            while self.pi.script_status(self.script)[0] == pigpio.PI_SCRIPT_INITING:
                if time.monotonic() > deadline:
                    raise RuntimeError("pigpio 안전 스크립트 초기화 실패")
                time.sleep(.01)
            self.pi.run_script(self.script, [0])
        except BaseException:
            self.close()
            raise

    def _encoder(self, a, b, sign, glitch):
        for pin in (a, b):
            self.pi.set_mode(pin, self.pigpio.INPUT)
            self.pi.set_pull_up_down(pin, self.pigpio.PUD_DOWN)
            self.pi.set_glitch_filter(pin, int(glitch))
        q = Quadrature(self.pi.read(a), self.pi.read(b), sign)
        self.callbacks.append(self.pi.callback(a, self.pigpio.EITHER_EDGE, lambda g, l, t: q.edge(0, l)))
        self.callbacks.append(self.pi.callback(b, self.pigpio.EITHER_EDGE, lambda g, l, t: q.edge(1, l)))
        return q

    def keepalive(self):
        now = time.monotonic()
        if now - self.watchdog_check > .5:
            if self.pi.script_status(self.script)[0] != self.pigpio.PI_SCRIPT_RUNNING:
                raise RuntimeError("pigpio 안전 타이머가 실행 중이 아닙니다")
            self.watchdog_check = now
        self.heartbeat = (self.heartbeat + 1) % 1000000000
        self.pi.update_script(self.script, [self.heartbeat])

    def motor(self, i, duty):
        m = self.c["motors"][i]
        value = duty * m["motor_sign"]
        active, inactive = (m["pwm_a"], m["pwm_b"]) if value >= 0 else (m["pwm_b"], m["pwm_a"])
        self.pi.set_PWM_dutycycle(inactive, 0)
        self.pi.set_PWM_dutycycle(active, round(abs(value) * 1000))

    def servo(self, angle):
        s = self.c["servo"]
        pulse = round(clamp(s["center_us"] + s["sign"] * angle * s["us_per_degree"], s["min_us"], s["max_us"]))
        self.pi.set_servo_pulsewidth(s["gpio"], pulse)
        return pulse

    def stop(self):
        # Attempt every pin even if one daemon command fails.
        errors = []
        for m in self.c["motors"]:
            for pin in (m["pwm_a"], m["pwm_b"]):
                try:
                    self.pi.set_PWM_dutycycle(pin, 0)
                except Exception as exc:
                    errors.append(exc)
        try:
            self.pi.set_servo_pulsewidth(self.c["servo"]["gpio"], 0)
        except Exception as exc:
            errors.append(exc)
        if errors:
            raise RuntimeError(f"GPIO 정지 명령 실패: {errors[0]}")

    def close(self):
        stopped = False
        try:
            self.stop()
            stopped = True
        except Exception:
            logging.exception("GPIO 정지 실패; daemon watchdog 유지")
        for cb in self.callbacks:
            with contextlib.suppress(Exception):
                cb.cancel()
        if stopped and self.script is not None:
            with contextlib.suppress(Exception):
                self.pi.stop_script(self.script)
                self.pi.delete_script(self.script)
        self.pi.stop()


class Ultrasound:
    def __init__(self, hw, c, stop_event):
        self.c, self.stop_event, self.lock = c, stop_event, threading.Lock()
        self.pi = hw.pi
        self.active = None
        self.rise = None
        self.done = threading.Event()
        self.values = [{"name": s["name"], "distance_m": None, "pulse_us": None,
                        "status": "대기" if c["enabled"] else "비활성", "updated": None} for s in c["sensors"]]
        self.callbacks = []
        if c["enabled"]:
            for i, s in enumerate(c["sensors"]):
                self.pi.set_mode(s["trig"], hw.pigpio.OUTPUT)
                self.pi.write(s["trig"], 0)
                self.pi.set_mode(s["echo"], hw.pigpio.INPUT)
                self.pi.set_pull_up_down(s["echo"], hw.pigpio.PUD_DOWN)
                self.callbacks.append(self.pi.callback(s["echo"], hw.pigpio.EITHER_EDGE,
                                      lambda g, l, t, index=i: self.echo(index, l, t)))

    def echo(self, i, level, tick):
        with self.lock:
            if self.active != i:
                return
            if level == 1:
                self.rise = tick
            elif level == 0 and self.rise is not None:
                width = (tick - self.rise) & 0xFFFFFFFF
                distance = width * self.c["speed_of_sound_m_s"] / 2000000
                valid = self.c["min_m"] <= distance <= self.c["max_m"]
                self.values[i].update(distance_m=round(distance, 4) if valid else None,
                                      pulse_us=width, status="정상" if valid else "범위 밖", updated=time.monotonic())
                self.active = None
                self.done.set()

    def run(self):
        if not self.c["enabled"]:
            return
        try:
            while not self.stop_event.is_set():
                for i, s in enumerate(self.c["sensors"]):
                    started = time.monotonic()
                    if self.stop_event.is_set():
                        return
                    with self.lock:
                        self.active, self.rise = i, None
                        self.done.clear()
                    if self.pi.read(s["echo"]) == 0:
                        self.pi.gpio_trigger(s["trig"], int(self.c["trigger_us"]), 1)
                        self.done.wait(self.c["timeout_s"])
                    with self.lock:
                        if self.active is not None:
                            self.values[i].update(distance_m=None, pulse_us=None, status="응답 없음", updated=time.monotonic())
                            self.active = None
                    self.stop_event.wait(max(0, self.c["slot_s"] - (time.monotonic() - started)))
        except Exception as exc:
            with self.lock:
                for v in self.values:
                    v.update(distance_m=None, status=f"오류: {exc}")
            logging.exception("초음파 수집 중지")

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            values = copy.deepcopy(self.values)
        for v in values:
            v["age_s"] = None if v["updated"] is None else round(now - v["updated"], 3)
            if v["age_s"] is not None and v["age_s"] > 1:
                v.update(distance_m=None, status="오래된 데이터")
        return values

    def close(self):
        for cb in self.callbacks:
            cb.cancel()


LIDAR_RE = re.compile(r"^\s*(S\s+)?theta:\s*([+-]?[\d.]+)\s+Dist:\s*([+-]?[\d.]+)\s+Q:\s*(\d+)\s*$", re.I)


def parse_lidar_line(line):
    match = LIDAR_RE.match(line)
    if not match:
        return None
    try:
        angle, mm, quality = float(match[2]), float(match[3]), int(match[4])
        if not math.isfinite(angle) or not math.isfinite(mm) or not 0 <= angle <= 360 or mm < 0:
            return None
        return bool(match[1]), angle % 360, mm / 1000, quality
    except ValueError:
        return None


class Lidar:
    def __init__(self, c, stop_event):
        self.c, self.stop_event, self.lock = c, stop_event, threading.Lock()
        self.process = None
        self.points, self.pending = [], {}
        self.last_angle = None
        self.updated = None
        self.scan_id = 0
        self.status = "대기" if c["enabled"] else "비활성: SDK 경로 설정 후 활성화"
        self.diagnostics = deque(maxlen=4)

    def consume(self, line):
        parsed = parse_lidar_line(line)
        if parsed is None:
            if line.strip():
                with self.lock:
                    self.diagnostics.append(line.strip()[:200])
            return
        sync, angle, distance, quality = parsed
        # ultra_simple uses ascendScanData. A retained S flag can occur mid-scan;
        # use the sorted angle wrap to avoid splitting one rotation twice.
        boundary = self.last_angle is not None and angle + 180 < self.last_angle
        with self.lock:
            if boundary:
                self.points = list(self.pending.values())
                self.pending.clear()
                self.updated = time.monotonic()
                self.scan_id += 1
                self.status = "정상" if self.points else "유효 거리 없음"
            self.last_angle = angle
            if self.c["min_m"] <= distance <= self.c["max_m"] and quality >= self.c["min_quality"]:
                a = (angle + self.c["angle_offset_deg"]) % 360
                self.pending[int(a * 2) % 720] = [round(a, 2), round(distance, 4), quality]

    def run(self):
        if not self.c["enabled"]:
            return
        try:
            exe = Path(self.c["executable"]).expanduser()
            if not exe.is_absolute() or not exe.is_file() or not os.access(exe, os.X_OK):
                raise RuntimeError("실행 가능한 ultra_simple의 절대 경로를 설정하세요")
            stdbuf = shutil.which("stdbuf")
            if not stdbuf:
                raise RuntimeError("stdbuf가 필요합니다: sudo apt install coreutils")
            cmd = [stdbuf, "-oL", "-eL", str(exe), "--channel", "--serial", self.c["port"], str(self.c["baud"])]
            with self.lock:
                if self.stop_event.is_set():
                    return
                self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                text=True, encoding="utf-8", errors="replace", bufsize=1,
                                                start_new_session=True)
            with self.process.stdout:
                for line in iter(lambda: self.process.stdout.readline(4096), ""):
                    if self.stop_event.is_set():
                        break
                    self.consume(line)
            if not self.stop_event.is_set():
                raise RuntimeError(f"ultra_simple 종료 (code={self.process.poll()})")
        except Exception as exc:
            with self.lock:
                self.status = f"오류: {exc}"
                self.points = []
            logging.error("LiDAR: %s", exc)

    def snapshot(self):
        with self.lock:
            age = None if self.updated is None else time.monotonic() - self.updated
            stale = age is None or age > self.c["stale_s"]
            status = self.status
            if stale and status in ("정상", "대기", "유효 거리 없음"):
                status = "데이터 수신 대기/지연"
            return {"points": [] if stale else self.points.copy(), "status": status,
                    "age_s": None if age is None else round(age, 3), "scan_id": self.scan_id,
                    "diagnostics": list(self.diagnostics)}

    def close(self):
        with self.lock:
            p = self.process
        if p and p.poll() is None:
            for sig, timeout in ((signal.SIGINT, 2), (signal.SIGTERM, 1), (signal.SIGKILL, 1)):
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(p.pid, sig)
                try:
                    p.wait(timeout)
                    break
                except subprocess.TimeoutExpired:
                    pass


class Controller:
    def __init__(self, c, hw, stop_event):
        self.c, self.hw, self.stop_event = c, hw, stop_event
        self.lock = threading.RLock()
        self.armed = self.estop = False
        self.fatal = False
        self.reason = "시작: 구동 비활성"
        self.owner = self.lease = None
        self.seq = -1
        self.last_command = 0.0
        self.requested = self.target = self.steering = self.steer_target = 0.0
        self.duty_limit = c["control"]["max_duty"]
        self.servo_us = 0
        self.last_direction = 0
        self.reverse_until = 0.0
        self.still_since = 0.0
        self.previous_counts = [q.snapshot()["count"] for q in hw.encoders]
        self.rpms = [0.0, 0.0]
        self.duties = [0.0, 0.0]
        self.stall_since = [None, None]
        self.wrong_since = [None, None]
        self.pi = [PI(c["control"]["kp"], c["control"]["ki"]) for _ in range(2)]
        self.blockers = []
        if not c["hardware_confirmed"]:
            self.blockers.append("전원·3.3V 신호·배선 확인 후 hardware_confirmed=true")
        if not all(m["encoder_confirmed"] for m in c["motors"]):
            self.blockers.append("모터 x4 counts/rev 확인 후 encoder_confirmed=true")
        if not c["servo"]["calibrated"]:
            self.blockers.append("서보 중립/기구 범위 확인 후 calibrated=true")

    def disarm(self, reason, emergency=False):
        with self.lock:
            self.armed = False
            self.estop = self.estop or emergency
            self.reason, self.owner, self.lease = reason, None, None
            self.requested = self.target = self.steer_target = 0.0
            self.duties = [0.0, 0.0]
            self.servo_us = 0
            for pi in self.pi:
                pi.reset()
            self.hw.stop()

    def command(self, data):
        if not isinstance(data, dict):
            raise ValueError("명령은 JSON 객체여야 합니다")
        op = data.get("op")
        with self.lock:
            if op in ("stop", "estop"):
                self.disarm("비상정지" if op == "estop" else "사용자 정지", op == "estop")
                return {"ok": True}
            now = time.monotonic()
            number(data.get("seen"), now - .4, now + .01, "telemetry timestamp")
            client = data.get("client")
            if not isinstance(client, str) or not 8 <= len(client) <= 80:
                raise ValueError("client 식별자 오류")
            if op == "reset":
                if self.armed or self.fatal:
                    raise ValueError("구동 중이거나 하드웨어 오류입니다. 정지/프로그램 재시작 필요")
                self.estop = False
                self.reason = "정지 잠금 해제: 재활성화 필요"
            elif op == "zero_mt":
                if self.armed:
                    raise ValueError("MT 영점은 구동 비활성 상태에서 설정하세요")
                self.hw.mt.zero()
            elif op == "arm":
                if self.blockers or self.estop or self.fatal or self.armed:
                    raise ValueError("활성화 불가: " + "; ".join(self.blockers or [self.reason]))
                if max(abs(r) for r in self.rpms) > 5:
                    raise ValueError("바퀴가 정지한 후 활성화하세요")
                self.owner, self.lease = client, secrets.token_urlsafe(18)
                self.seq, self.last_command = -1, now
                self.armed, self.reason = True, "조종 활성"
                self.requested = self.target = self.steering = self.steer_target = 0.0
                self.last_direction = 0
                self.stall_since = [None, None]
                self.wrong_since = [None, None]
                return {"ok": True, "lease": self.lease, "armed_at": now}
            elif op == "drive":
                if not self.armed or client != self.owner or data.get("lease") != self.lease:
                    raise ValueError("조종 권한 없음: 다시 활성화하세요")
                if now - self.last_command > self.c["control"]["command_timeout_s"]:
                    self.disarm("조종 신호 시간 초과")
                    raise ValueError("조종 신호 시간 초과: 다시 활성화하세요")
                seq = data.get("seq")
                if type(seq) is not int or seq <= self.seq:
                    raise ValueError("중복/역순 명령")
                rpm = number(data.get("rpm"), -self.c["control"]["max_rpm"], self.c["control"]["max_rpm"], "rpm")
                steer = number(data.get("steer"), -self.c["servo"]["max_angle_deg"], self.c["servo"]["max_angle_deg"], "steer")
                duty = number(data.get("duty_limit"), 0, self.c["control"]["max_duty"], "duty_limit")
                if type(data.get("deadman")) is not bool:
                    raise ValueError("deadman은 boolean이어야 합니다")
                self.requested = rpm if data["deadman"] else 0.0
                self.steer_target = steer if data["deadman"] else 0.0
                self.duty_limit, self.seq, self.last_command = duty, seq, now
            else:
                raise ValueError("알 수 없는 명령")
        return {"ok": True}

    def step(self, now, dt):
        ctrl = self.c["control"]
        with self.lock:
            for i, (m, q) in enumerate(zip(self.c["motors"], self.hw.encoders)):
                count = q.snapshot()["count"]
                rpm = (count - self.previous_counts[i]) * 60 / m["encoder_counts_per_rev"] / dt
                self.previous_counts[i] = count
                self.rpms[i] += dt / (ctrl["rpm_filter_s"] + dt) * (rpm - self.rpms[i])
            if self.armed and (dt > .15 or now - self.last_command > ctrl["command_timeout_s"]):
                self.disarm("제어 루프 지연" if dt > .15 else "조종 신호 시간 초과")
            self.hw.keepalive()
            if not self.armed:
                return
            direction = (self.requested > 0) - (self.requested < 0)
            if max(abs(r) for r in self.rpms) < 5:
                if not self.still_since:
                    self.still_since = now
            else:
                self.still_since = 0
            reversing = bool(direction and self.last_direction and direction != self.last_direction)
            if reversing:
                if not self.reverse_until:
                    self.reverse_until = now + ctrl["reversal_pause_s"]
                if now >= self.reverse_until and self.still_since and now - self.still_since >= .15:
                    self.last_direction = direction
                    self.reverse_until = 0
                    reversing = False
            else:
                self.reverse_until = 0
            if self.requested == 0 or reversing or self.duty_limit == 0:
                self.target = 0.0
            else:
                self.last_direction = direction
                self.target = approach(self.target, self.requested, ctrl["ramp_rpm_s"] * dt)
            for i, q in enumerate(self.hw.encoders):
                duty = self.pi[i].update(self.target, self.rpms[i], dt, self.duty_limit)
                pushing = abs(duty) > .1 and abs(self.target) > 5
                no_motion = pushing and abs(self.rpms[i]) < 2
                wrong = pushing and self.rpms[i] * self.target < -10
                self.stall_since[i] = (self.stall_since[i] or now) if no_motion else None
                self.wrong_since[i] = (self.wrong_since[i] or now) if wrong else None
                if self.stall_since[i] is not None and now - self.stall_since[i] > ctrl["stall_timeout_s"]:
                    self.disarm(f"M{i+1}: 엔코더 무응답/모터 정지", True)
                    return
                if self.wrong_since[i] is not None and now - self.wrong_since[i] > .2:
                    self.disarm(f"M{i+1}: 엔코더/구동 방향 불일치", True)
                    return
                if abs(self.rpms[i]) > max(100, ctrl["max_rpm"] * 1.5):
                    self.disarm(f"M{i+1}: 과속/엔코더 이상", True)
                    return
                self.hw.motor(i, duty)
                self.duties[i] = duty
            s = self.c["servo"]
            self.steering = approach(self.steering, self.steer_target, s["rate_deg_s"] * dt)
            self.servo_us = self.hw.servo(self.steering)

    def run(self):
        previous = time.monotonic()
        period = 1 / self.c["control"]["hz"]
        deadline = previous + period
        try:
            while not self.stop_event.wait(max(0, deadline - time.monotonic())):
                now = time.monotonic()
                self.step(now, max(.0001, now - previous))
                previous = now
                deadline = max(deadline + period, time.monotonic())
        except Exception as exc:
            logging.exception("제어 루프 중지")
            with self.lock:
                self.fatal = True
                with contextlib.suppress(Exception):
                    self.disarm(f"하드웨어 오류: {exc}", True)

    def snapshot(self):
        with self.lock:
            motors = []
            for i, (m, q) in enumerate(zip(self.c["motors"], self.hw.encoders)):
                e = q.snapshot()
                metres = math.pi * m["wheel_diameter_mm"] / 1000 / m["encoder_counts_per_rev"] / m["wheel_ratio"]
                motors.append({**e, "name": m["name"], "rpm": round(self.rpms[i], 2),
                               "duty": round(self.duties[i], 4), "distance_m": round(e["travel_counts"] * metres, 4),
                               "signed_distance_m": round(e["count"] * metres, 4),
                               "calibrated": m["encoder_confirmed"]})
            mt = self.hw.mt.snapshot()
            ppr = self.c["mt6701"]["ppr"]
            mt["ppr"] = ppr
            mt["relative_deg"] = round((mt["count"] - mt["zero_count"]) * 360 / (ppr * 4), 3) if ppr else None
            mt["since_index_deg"] = round((mt["count"] - mt["index_count"]) * 360 / (ppr * 4), 3) if ppr and mt["index_count"] is not None else None
            return {"monotonic": time.monotonic(), "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    "armed": self.armed, "estop": self.estop, "fatal": self.fatal, "reason": self.reason,
                    "owner": self.owner, "blockers": self.blockers, "motors": motors,
                    "target_rpm": round(self.target, 2), "requested_rpm": self.requested,
                    "distance_m": round(sum(m["distance_m"] for m in motors) / 2, 4),
                    "servo": {"command_deg": round(self.steering, 2) if self.servo_us else None,
                              "pulse_us": self.servo_us, "measured_deg": None}, "mt6701": mt,
                    "limits": {"max_rpm": self.c["control"]["max_rpm"], "max_duty": self.c["control"]["max_duty"],
                               "max_angle_deg": self.c["servo"]["max_angle_deg"]}}


class Runtime:
    def __init__(self, c):
        self.c = c
        self.stop_event = threading.Event()
        self.hw = Hardware(c)
        self.threads = []
        self.record_error = None
        try:
            self.controller = Controller(c, self.hw, self.stop_event)
            self.ultrasound = Ultrasound(self.hw, c["ultrasonic"], self.stop_event)
            self.lidar = Lidar(c["lidar"], self.stop_event)
        except BaseException:
            self.hw.close()
            raise

    def start(self):
        for name, function in (("control", self.controller.run), ("ultrasound", self.ultrasound.run),
                               ("lidar", self.lidar.run), ("recording", self.record)):
            thread = threading.Thread(target=function, name=name, daemon=True)
            thread.start()
            self.threads.append(thread)

    def snapshot(self):
        state = self.controller.snapshot()
        state.update(ultrasonic=self.ultrasound.snapshot(), lidar=self.lidar.snapshot(),
                     recording={"enabled": self.c["recording"]["enabled"], "error": self.record_error})
        return state

    def record(self):
        c = self.c["recording"]
        if not c["enabled"]:
            return
        handler = None
        try:
            (ROOT / "logs").mkdir(exist_ok=True)
            handler = RotatingFileHandler(ROOT / "logs" / "telemetry.jsonl", maxBytes=int(c["max_bytes"]),
                                          backupCount=int(c["backup_count"]), encoding="utf-8")
            # Use the handler's stream directly so disk errors are observable in UI.
            while not self.stop_event.is_set():
                state = self.snapshot()
                if not c["include_lidar"]:
                    state["lidar"].pop("points", None)
                line = json.dumps(state, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
                if handler.stream.tell() + len(line.encode("utf-8")) >= c["max_bytes"]:
                    handler.doRollover()
                handler.stream.write(line)
                handler.stream.flush()
                self.stop_event.wait(1 / c["hz"])
        except Exception as exc:
            self.record_error = str(exc)
            logging.exception("기록 중지")
        finally:
            if handler:
                handler.close()

    def close(self):
        self.stop_event.set()
        with contextlib.suppress(Exception):
            self.controller.disarm("프로그램 종료")
        self.lidar.close()
        for t in self.threads:
            t.join(timeout=1)
        self.ultrasound.close()
        self.hw.close()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, runtime, token):
        self.runtime, self.token = runtime, token
        self.slots = threading.BoundedSemaphore(12)
        self.streams = threading.BoundedSemaphore(4)
        super().__init__(address, Handler)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(1)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def log_message(self, *_):
        pass

    def headers_for(self, status, content_type, length=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        if length is not None:
            self.send_header("Content-Length", str(length))
        self.end_headers()

    def reply(self, status, value):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.headers_for(status, "application/json; charset=utf-8", len(body))
        self.wfile.write(body)

    def authenticated(self):
        token = self.headers.get("X-RC-Token", "")
        if not secrets.compare_digest(token.encode(), self.server.token.encode()):
            self.reply(401, {"error": "접속 키를 확인하세요"})
            return False
        origin = self.headers.get("Origin")
        if origin and origin != "http://" + self.headers.get("Host", ""):
            self.reply(403, {"error": "다른 웹사이트에서 보낸 요청은 허용하지 않습니다"})
            return False
        return True

    def do_GET(self):
        route = urlsplit(self.path).path
        if route == "/":
            body = (ROOT / "index.html").read_bytes()
            self.headers_for(200, "text/html; charset=utf-8", len(body))
            self.wfile.write(body)
        elif route in ("/api/state", "/api/events") and self.authenticated():
            if route == "/api/state":
                self.reply(200, self.server.runtime.snapshot())
                return
            if not self.server.streams.acquire(blocking=False):
                self.reply(503, {"error": "모니터링 연결은 최대 4개입니다"})
                return
            try:
                self.headers_for(200, "text/event-stream; charset=utf-8")
                while not self.server.runtime.stop_event.is_set():
                    body = json.dumps(self.server.runtime.snapshot(), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                    self.wfile.write(("data: " + body + "\n\n").encode("utf-8"))
                    self.wfile.flush()
                    self.server.runtime.stop_event.wait(.1)
            except (OSError, ConnectionError):
                pass
            finally:
                self.server.streams.release()
        elif route not in ("/api/state", "/api/events"):
            self.reply(404, {"error": "Not found"})

    def do_POST(self):
        if urlsplit(self.path).path != "/api/command":
            self.reply(404, {"error": "Not found"})
            return
        if not self.authenticated():
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 1 <= length <= 2048 or self.headers.get("Transfer-Encoding"):
                raise ValueError("명령 크기 오류")
            if self.headers.get_content_type() != "application/json":
                raise ValueError("application/json 필요")
            data = json.loads(self.rfile.read(length))
            self.reply(200, self.server.runtime.controller.command(data))
        except (ValueError, TypeError, KeyError) as exc:
            self.reply(400, {"error": str(exc)})
        except Exception:
            logging.exception("명령 처리 오류")
            with contextlib.suppress(Exception):
                self.server.runtime.controller.disarm("명령 처리 오류", True)
            self.reply(500, {"error": "명령 처리 실패: 구동 정지"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--check-config", action="store_true", help="GPIO 접근 없이 설정만 검증")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    c = validate_config(json.loads(args.config.read_text(encoding="utf-8-sig")))
    if args.check_config:
        print("설정 형식 정상. 실제 배선/센서 보정 검증은 별도입니다.")
        return
    if sys.platform != "linux":
        raise RuntimeError("실제 제어 서버는 Raspberry Pi 4 Linux에서 실행하세요. 시뮬레이션 모드는 없습니다")
    model = Path("/proc/device-tree/model").read_text().rstrip("\x00")
    if "Raspberry Pi 4" not in model:
        raise RuntimeError(f"이 초기 버전의 지원 대상은 Raspberry Pi 4입니다: {model}")
    import fcntl
    with open(ROOT / ".rc_controller.lock", "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("이 폴더의 제어 프로그램이 이미 실행 중입니다") from None
        token = os.environ.get("RC_TOKEN") or secrets.token_urlsafe(18)
        if len(token) < 12:
            raise ValueError("RC_TOKEN은 12자 이상으로 지정하세요")
        runtime = Runtime(c)
        server = None
        def shutdown_signal(*_):
            runtime.stop_event.set()
        signal.signal(signal.SIGTERM, shutdown_signal)
        signal.signal(signal.SIGINT, shutdown_signal)
        try:
            server = Server((c["server"]["host"], c["server"]["port"]), runtime, token)
            server.timeout = .2
            runtime.start()
            print(f"\n브라우저: http://<Pi IP>:{c['server']['port']}\n접속 키: {token}\nCtrl+C로 종료\n", flush=True)
            while not runtime.stop_event.is_set():
                server.handle_request()
        finally:
            runtime.close()
            if server:
                server.server_close()


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        logging.error("실행 실패: %s", exc)
        sys.exit(1)
