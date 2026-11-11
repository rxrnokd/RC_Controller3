"""Safety and protocol regression tests. No simulator or hardware execution."""
import copy
import http.client
import json
import math
import threading
import time
import unittest
from unittest.mock import Mock, patch

import controller as rc


def config():
    return json.loads((rc.ROOT / "config.json").read_text(encoding="utf-8"))


class OutputSpy:
    """Record calls for unit assertions; no motor/sensor dynamics."""
    def __init__(self):
        self.encoders = [rc.Quadrature(), rc.Quadrature()]
        self.mt = rc.Quadrature()
        self.outputs = [0, 0]
        self.stops = self.beats = 0

    def stop(self):
        self.stops += 1
        self.outputs = [0, 0]

    def keepalive(self):
        self.beats += 1

    def motor(self, i, duty):
        self.outputs[i] = duty

    def servo(self, angle):
        return 1500 + round(angle * 11.111111)


class ConfigTests(unittest.TestCase):
    def test_default_config(self):
        rc.validate_config(config())

    def test_duplicate_gpio(self):
        c = config()
        c["servo"]["gpio"] = 16
        with self.assertRaises(ValueError):
            rc.validate_config(c)

    def test_nonfinite_and_invalid_limits(self):
        for key, value in (("max_rpm", math.nan), ("max_duty", 1.1), ("hz", 0), ("command_timeout_s", 10)):
            with self.subTest(key=key):
                c = config()
                c["control"][key] = value
                with self.assertRaises(ValueError):
                    rc.validate_config(c)

    def test_servo_mechanical_limit(self):
        c = config()
        c["servo"]["max_angle_deg"] = 90
        with self.assertRaises(ValueError):
            rc.validate_config(c)

    def test_confirmation_must_be_bool(self):
        c = config()
        c["hardware_confirmed"] = "false"
        with self.assertRaises(ValueError):
            rc.validate_config(c)


class EncoderTests(unittest.TestCase):
    def test_quadrature_forward_reverse_and_path(self):
        q = rc.Quadrature()
        for ch, level in ((0, 1), (1, 1), (0, 0), (1, 0)):
            q.edge(ch, level)
        self.assertEqual(q.snapshot()["count"], 4)
        q.index()
        q.zero()
        for ch, level in ((1, 1), (0, 1), (1, 0), (0, 0)):
            q.edge(ch, level)
        s = q.snapshot()
        self.assertEqual((s["count"], s["travel_counts"], s["index_count"], s["zero_count"]), (0, 8, 4, 4))

    def test_duplicate_and_timeout_events(self):
        q = rc.Quadrature()
        q.edge(0, 2)
        q.edge(0, 0)
        self.assertEqual(q.count, 0)

    def test_sign(self):
        q = rc.Quadrature(sign=-1)
        q.edge(0, 1)
        self.assertEqual(q.count, -1)


class PITests(unittest.TestCase):
    def test_antiwindup_and_stop(self):
        pi = rc.PI(.003, .006)
        for _ in range(1000):
            self.assertLessEqual(pi.update(200, 0, .02, .35), .35)
        self.assertLessEqual(pi.integral, .35)
        self.assertEqual(pi.update(0, 100, .02, .35), 0)
        self.assertEqual(pi.integral, 0)

    def test_reverse_and_no_active_reverse_braking(self):
        pi = rc.PI(.003, .006)
        self.assertLess(pi.update(-50, -10, .02, .35), 0)
        self.assertEqual(pi.update(-50, -500, .02, .35), 0)


class LidarTests(unittest.TestCase):
    def test_sdk_output_units(self):
        self.assertEqual(rc.parse_lidar_line("S  theta: 035.25 Dist: 00123.50 Q: 47 \n"), (True, 35.25, .1235, 47))

    def test_invalid_output(self):
        for text in ("SDK version 2", "theta: nan Dist: 100 Q: 2", "theta: 370 Dist: 20 Q: 1", "theta: 1 Dist: -2 Q: 3", "theta: 1.2.3 Dist: 50 Q: 3"):
            self.assertIsNone(rc.parse_lidar_line(text))

    def test_scan_boundary_not_mid_scan_s_and_filtering(self):
        lidar = rc.Lidar(config()["lidar"], threading.Event())
        for line in ("theta: 0 Dist: 1000 Q: 20", "S theta: 90 Dist: 2000 Q: 20", "theta: 180 Dist: 0 Q: 0", "theta: 350 Dist: 13000 Q: 30", "theta: 0 Dist: 1100 Q: 20"):
            lidar.consume(line)
        s = lidar.snapshot()
        self.assertEqual(s["scan_id"], 1)
        self.assertEqual(len(s["points"]), 2)
        self.assertEqual(s["points"][0][1], 1)

    def test_stale_scan_is_cleared(self):
        lidar = rc.Lidar(config()["lidar"], threading.Event())
        lidar.updated = time.monotonic() - 10
        lidar.points = [[1, 1, 1]]
        self.assertEqual(lidar.snapshot()["points"], [])

    def test_bounded_scan_storage(self):
        lidar = rc.Lidar(config()["lidar"], threading.Event())
        for i in range(10000):
            lidar.consume(f"theta: {i * .0359} Dist: 1000 Q: 20")
        self.assertLessEqual(len(lidar.pending), 720)


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.c = config()
        self.c["hardware_confirmed"] = self.c["servo"]["calibrated"] = True
        for m in self.c["motors"]:
            m["encoder_confirmed"] = True
        self.hw = OutputSpy()
        self.control = rc.Controller(self.c, self.hw, threading.Event())
        self.client = "test-client-one"
        self.seq = 0
        self.clock = patch("controller.time.monotonic", return_value=100.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def cmd(self, op, **extra):
        return self.control.command({"op": op, "client": self.client, "seen": self.now.return_value, **extra})

    def arm(self):
        self.lease = self.cmd("arm")["lease"]

    def drive(self, rpm=40, **extra):
        self.seq += 1
        data = dict(lease=self.lease, seq=self.seq, rpm=rpm, steer=0, duty_limit=.35, deadman=True)
        data.update(extra)
        return self.cmd("drive", **data)

    def test_uncalibrated_startup_blocks_arm(self):
        ctl = rc.Controller(config(), self.hw, threading.Event())
        with self.assertRaises(ValueError):
            ctl.command({"op": "arm", "client": self.client, "seen": 100})
        self.assertEqual(self.hw.outputs, [0, 0])

    def test_deadman_release_stops_output(self):
        self.arm()
        self.drive()
        self.control.step(100.02, .02)
        self.assertGreater(self.hw.outputs[0], 0)
        self.drive(deadman=False)
        self.control.step(100.04, .02)
        self.assertEqual(self.hw.outputs, [0, 0])

    def test_command_timeout_disarms_and_cannot_resume(self):
        self.arm()
        self.drive()
        self.control.step(100.5, .02)
        self.assertFalse(self.control.armed)
        with self.assertRaises(ValueError):
            self.drive()

    def test_timeout_in_handler_before_next_control_tick(self):
        self.arm()
        self.now.return_value = 100.5
        with self.assertRaises(ValueError):
            self.drive()
        self.assertFalse(self.control.armed)

    def test_emergency_latches_until_reset(self):
        self.arm()
        self.cmd("estop")
        with self.assertRaises(ValueError):
            self.arm()
        self.cmd("reset")
        self.arm()
        self.assertTrue(self.control.armed)

    def test_other_client_and_old_lease_rejected(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.drive(client="other-client")
        old = self.lease
        self.cmd("stop")
        self.arm()
        with self.assertRaises(ValueError):
            self.drive(lease=old)

    def test_stale_and_nonfinite_and_duplicate_commands(self):
        self.arm()
        self.drive()
        for extra in ({"seen": 99}, {"rpm": math.nan}, {"rpm": math.inf}, {"seq": 1}, {"deadman": "false"}, {"duty_limit": 1}):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                self.drive(**extra)

    def test_loop_overrun_stops(self):
        self.arm()
        self.drive()
        self.control.step(100.2, .2)
        self.assertFalse(self.control.armed)

    def test_stalled_motor_trips(self):
        self.arm()
        for step in range(1, 121):
            self.now.return_value = 100 + step * .02
            if not self.control.armed:
                break
            self.drive(rpm=80)
            self.control.step(self.now.return_value, .02)
        self.assertTrue(self.control.estop)
        self.assertEqual(self.hw.outputs, [0, 0])

    def test_reversal_requires_pause_and_measured_stop(self):
        self.arm()
        self.drive()
        self.control.step(100.02, .02)
        self.drive(rpm=-40)
        self.control.step(100.04, .02)
        self.assertEqual(self.hw.outputs, [0, 0])
        self.now.return_value = 100.2
        self.drive(rpm=-40)
        self.control.step(100.2, .02)
        self.assertEqual(self.hw.outputs, [0, 0])
        self.now.return_value = 100.4
        self.drive(rpm=-40)
        self.control.step(100.4, .02)
        self.assertLess(self.hw.outputs[0], 0)

    def test_distance_and_mt_unknown_ppr(self):
        self.hw.encoders[0].count = self.hw.encoders[0].travel = 1320
        self.hw.encoders[1].count = self.hw.encoders[1].travel = 1320
        s = self.control.snapshot()
        self.assertAlmostEqual(s["distance_m"], math.pi * .075, places=4)
        self.assertIsNone(s["mt6701"]["relative_deg"])
        self.assertIsNone(s["servo"]["measured_deg"])

    def test_mt_known_ppr_and_manual_zero(self):
        self.c["mt6701"]["ppr"] = 256
        self.hw.mt.count = 256
        self.assertEqual(self.control.snapshot()["mt6701"]["relative_deg"], 90)
        self.cmd("zero_mt")
        self.assertEqual(self.control.snapshot()["mt6701"]["relative_deg"], 0)


class EchoTests(unittest.TestCase):
    def test_tick_wrap_distance_and_timeout_state(self):
        # Exercise the real callback without initializing GPIO.
        u = rc.Ultrasound.__new__(rc.Ultrasound)
        u.c = config()["ultrasonic"]
        u.lock = threading.Lock()
        u.done = threading.Event()
        u.active, u.rise = 0, None
        u.values = [{"name": "one", "distance_m": None, "updated": None}]
        u.echo(1, 1, 200)  # inactive sensor must not affect this measurement
        self.assertIsNone(u.rise)
        u.echo(0, 1, 0xFFFFFF00)
        u.echo(0, 0, (0xFFFFFF00 + 2000) & 0xFFFFFFFF)
        self.assertAlmostEqual(u.values[0]["distance_m"], .343, places=3)
        self.assertTrue(u.done.is_set())
        u.values[0]["updated"] -= 2
        self.assertIsNone(u.snapshot()[0]["distance_m"])


class HardwareSafetyTests(unittest.TestCase):
    def test_shutdown_attempts_all_outputs_despite_one_failure(self):
        hw = rc.Hardware.__new__(rc.Hardware)
        hw.c = config()
        hw.pi = Mock()
        hw.pi.set_PWM_dutycycle.side_effect = [RuntimeError("socket"), None, None, None]
        with self.assertRaises(RuntimeError):
            hw.stop()
        self.assertEqual(hw.pi.set_PWM_dutycycle.call_count, 4)
        hw.pi.set_servo_pulsewidth.assert_called_once_with(13, 0)

    def test_missing_watchdog_stops_heartbeat(self):
        hw = rc.Hardware.__new__(rc.Hardware)
        hw.pi = Mock()
        hw.pigpio = Mock(PI_SCRIPT_RUNNING=2)
        hw.pi.script_status.return_value = (4, [])
        hw.script, hw.watchdog_check, hw.heartbeat = 1, 0, 0
        with self.assertRaises(RuntimeError):
            hw.keepalive()
        hw.pi.update_script.assert_not_called()


class HttpTests(unittest.TestCase):
    def setUp(self):
        class Fixture:
            stop_event = threading.Event()
            controller = rc.Controller(config(), OutputSpy(), stop_event)
            def snapshot(self):
                return {"source": "protocol-test", "armed": False}
        self.fixture = Fixture()
        self.server = rc.Server(("127.0.0.1", 0), self.fixture, "test-access-key")
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.fixture.stop_event.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, method, route, data=None, auth=True, extra=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        headers = {"Content-Type": "application/json"}
        if auth:
            headers["X-RC-Token"] = "test-access-key"
        headers.update(extra or {})
        connection.request(method, route, json.dumps(data) if data is not None else None, headers)
        response = connection.getresponse()
        result = response.status, response.read()
        connection.close()
        return result

    def test_auth_and_origin(self):
        self.assertEqual(self.request("GET", "/api/state", auth=False)[0], 401)
        self.assertEqual(self.request("GET", "/api/state")[0], 200)
        self.assertEqual(self.request("POST", "/api/command", {"op": "stop"}, extra={"Origin": "http://evil.invalid"})[0], 403)

    def test_stop_and_malformed_command(self):
        self.assertEqual(self.request("POST", "/api/command", {"op": "estop"})[0], 200)
        self.assertTrue(self.fixture.controller.estop)
        self.assertEqual(self.request("POST", "/api/command", ["drive"])[0], 400)

    def test_sse_and_static_root(self):
        status, body = self.request("GET", "/", auth=False)
        self.assertEqual(status, 200)
        self.assertIn(b"<canvas", body)
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        connection.request("GET", "/api/events", headers={"X-RC-Token": "test-access-key"})
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertTrue(response.readline().startswith(b"data: "))
        response.close()
        connection.close()


if __name__ == "__main__":
    unittest.main()
