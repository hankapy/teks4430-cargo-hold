"""Unit tests for perception_core with synthetic data (no ROS)."""

from hold_autonomy import process_model as pm
from hold_autonomy.perception_core import Perception, G_ALARM


def feed_exhaust(p, t0, t1, co=0.0, ch4=0.0, dt=0.05):
    t = t0
    while t <= t1:
        p.on_exhaust(t, co, ch4, pm.O2_AIR)
        t += dt


def test_co_generation_independent_of_ventilation():
    """Same true generation g, different ventilation: estimates agree."""
    g = 2.0
    for vf in (0.0, 1.0):
        p = Perception()
        p.on_actuator(vf)
        k = pm.air_exchange(vf)
        co_ss = g / k  # steady state for constant generation
        feed_exhaust(p, 0.0, 1.0)
        p.on_zone_sample(1.0, 2, co_ss, 0.0, pm.O2_AIR, pm.T_AMB)
        feed_exhaust(p, 1.0, 5.0)
        p.on_zone_sample(5.0, 2, co_ss, 0.0, pm.O2_AIR, pm.T_AMB)
        assert abs(p.zones[2].co_gen - g) < 0.05, vf


def test_selfheat_needs_two_consecutive_estimates():
    p = Perception()
    feed_exhaust(p, 0.0, 1.0)
    p.on_zone_sample(1.0, 1, 0.0, 0.0, pm.O2_AIR, pm.T_AMB)
    feed_exhaust(p, 1.0, 5.0)
    p.on_zone_sample(5.0, 1, 4 * 4 * G_ALARM, 0.0, pm.O2_AIR, pm.T_AMB)
    assert not p.state().selfheat
    feed_exhaust(p, 5.0, 9.0)
    p.on_zone_sample(9.0, 1, 8 * 4 * G_ALARM, 0.0, pm.O2_AIR, pm.T_AMB)
    s = p.state()
    assert s.selfheat and s.hotspot_zone == 1


def test_mixing_correction_lowers_neighbour_estimate():
    """CO arriving from a hot neighbour is not counted as local generation."""
    p = Perception()
    feed_exhaust(p, 0.0, 1.0)
    p.on_zone_sample(1.0, 1, 60.0, 0.0, pm.O2_AIR, pm.T_AMB)  # hot neighbour
    p.on_zone_sample(1.0, 0, 10.0, 0.0, pm.O2_AIR, pm.T_AMB)
    feed_exhaust(p, 1.0, 5.0)
    p.on_zone_sample(5.0, 0, 12.0, 0.0, pm.O2_AIR, pm.T_AMB)
    raw = (12.0 - 10.0) / 4.0 + pm.K_LEAK * 11.0
    assert p.zones[0].co_gen < raw - 1.0


def test_methane_uses_max_of_exhaust_and_recent_samples():
    p = Perception()
    feed_exhaust(p, 0.0, 2.0, ch4=5.0)
    p.on_zone_sample(2.0, 0, 0.0, 12.0, pm.O2_AIR, pm.T_AMB)
    assert abs(p.state().ch4_lel - 12.0) < 1e-6
    feed_exhaust(p, 2.0, 4.0, ch4=5.0)  # sample now 2 h old
    assert p.state().ch4_lel < 6.0


def test_exhaust_drift_flagged_as_sensor_fault():
    p = Perception()
    t = 0.0
    for k in range(12):  # zones read 0 ppm, exhaust drifts upwards
        feed_exhaust(p, t, t + 2.0, co=2.0 * k)
        t += 2.0
        for z in range(4):
            p.on_zone_sample(t, z, 0.0, 0.0, pm.O2_AIR, pm.T_AMB)
    assert p.state().sensor_fault


def test_raw_reliability_after_vent_stop():
    p = Perception()
    feed_exhaust(p, 0.0, 0.1)
    p.on_actuator(1.0)
    feed_exhaust(p, 0.1, 2.0)
    p.on_actuator(0.0)
    feed_exhaust(p, 2.0, 4.0)
    assert not p.state().raw_reliable
    feed_exhaust(p, 4.0, 6.5)
    assert p.state().raw_reliable
