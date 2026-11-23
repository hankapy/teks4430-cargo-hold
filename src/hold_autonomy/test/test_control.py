"""Controller tests (no ROS): PID, vent interlock, rover state machine."""

from hold_autonomy import config as cfg
from hold_autonomy.control_core import (
    PID, VentController, RoverController, led_commands, CLOSED, CONTROL_CH4, PURGE, VENT_MIN,
)
from hold_autonomy.sensor_model import GasReading


def test_pid_antiwindup():
    pid = PID(1.0, 1.0, 0.0, 0.0, 1.0)
    for _ in range(100):
        pid.update(10.0, 0.0, 0.1)  # saturated high for 10 s
    assert pid.integral < 2.0  # did not wind up
    assert pid.update(0.0, 0.5, 0.1) < 1.0  # leaves saturation at once


def test_vent_interlock_fan_waits_for_damper():
    v = VentController()
    fan, damper = v.update(0.0, PURGE, 0.0, 10.0, damper_pos=0.0, fan_speed=0.0)
    assert damper == cfg.DAMPER_OPEN and fan == 0.0
    fan, damper = v.update(0.1, PURGE, 0.0, 10.0, damper_pos=cfg.DAMPER_OPEN, fan_speed=0.0)
    assert fan == cfg.FAN_NOMINAL


def test_vent_interlock_damper_waits_for_fan_stop():
    v = VentController()
    fan, damper = v.update(0.0, CLOSED, 0.0, 10.0, damper_pos=cfg.DAMPER_OPEN, fan_speed=15.0)
    assert fan == 0.0 and damper == cfg.DAMPER_OPEN
    fan, damper = v.update(0.1, CLOSED, 0.0, 10.0, damper_pos=cfg.DAMPER_OPEN, fan_speed=0.0)
    assert damper == 0.0


def test_vent_pi_direction_and_minimum():
    v = VentController()
    open_ = cfg.DAMPER_OPEN
    v.update(0.0, CONTROL_CH4, 10.0, 10.0, open_, 0.0)
    fan_low, _ = v.update(0.1, CONTROL_CH4, 5.0, 10.0, open_, 0.0)  # below setpoint
    assert abs(fan_low - VENT_MIN * cfg.FAN_NOMINAL) < 1e-9
    fan_high, _ = v.update(0.2, CONTROL_CH4, 16.0, 10.0, open_, 0.0)  # above setpoint
    assert fan_high > fan_low


def test_led_mapping():
    assert led_commands(0) == (1, 0)
    assert led_commands(2) == (2, 0)
    assert led_commands(3) == (3, 1)


def simulate_rover(ctrl, seconds, priority=-1, block_probe=False):
    dt, t, pos, vel, probe = 0.032, 0.0, 0.0, 0.0, 0.0
    samples = []
    while t < seconds:
        reading = GasReading(1.0, 0.0, 20.9, 15.0)
        v_cmd, p_cmd, s = ctrl.update(t, pos, vel, probe, priority, reading)
        if s:
            samples.append(s.zone)
        step = cfg.ROVER_ACCEL * dt
        vel = v_cmd if abs(v_cmd - vel) <= step else vel + step * (1 if v_cmd > vel else -1)
        pos += vel * dt
        if not block_probe:
            probe += max(-cfg.PROBE_SPEED * dt, min(cfg.PROBE_SPEED * dt, p_cmd - probe))
        t += dt
    return samples


def test_rover_round_robin_visits_all_zones():
    samples = simulate_rover(RoverController(), 60.0)
    assert samples[:4] == [0, 1, 2, 3]


def test_rover_priority_zone_every_other_visit():
    samples = simulate_rover(RoverController(), 120.0, priority=2)
    assert samples[0::2][:4] == [2, 2, 2, 2]
    assert set(samples[1::2]) == {0, 1, 3}


def test_rover_timeout_goes_to_fault():
    ctrl = RoverController()
    simulate_rover(ctrl, 20.0, block_probe=True)  # probe never reaches depth
    assert ctrl.faults >= 1
