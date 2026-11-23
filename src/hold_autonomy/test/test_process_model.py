"""Unit tests for the zonal process model and sensor models (no ROS)."""

import numpy as np

from hold_autonomy import config as cfg
from hold_autonomy import process_model as pm
from hold_autonomy.sensor_model import ProbeSensor, port_at

DT = 1.0 / 60.0


def run(cargo, vent, hours):
    h = pm.HoldState()
    t = 0.0
    while t < hours:
        h = pm.step(h, pm.CARGOES[cargo], vent, t, DT)
        t += DT
    return h


def test_limestone_stays_inert():
    h = run("limestone", 0.0, 120)
    for z in h.zones:
        assert abs(z.T - pm.T_AMB) < 1e-6 and z.CO < 1e-6 and z.CH4 < 1e-6


def test_ventilation_limits_methane():
    assert max(z.CH4 for z in run("coal_methane", 0.0, 48).zones) > pm.CH4_LIMIT
    assert max(z.CH4 for z in run("coal_methane", 1.0, 48).zones) < pm.CH4_LIMIT


def test_ventilation_accelerates_self_heating():
    closed = run("coal_selfheat", 0.0, 72).zones[1].T
    vented = run("coal_selfheat", 1.0, 72).zones[1].T
    assert vented > closed + 10


def test_hotspot_is_local():
    h = run("coal_selfheat", 0.0, 48)
    co = [z.CO for z in h.zones]
    assert int(np.argmax(co)) == 1
    assert co[1] > 1.5 * co[cfg.EXHAUST_ZONE]


def test_exhaust_air_blends_with_ventilation():
    h = run("coal_selfheat", 0.0, 48)
    closed = pm.exhaust_air(h, 0.0).CO
    vented = pm.exhaust_air(h, 1.0).CO
    assert abs(closed - h.zones[cfg.EXHAUST_ZONE].CO) < 1e-9
    assert abs(vented - np.mean([z.CO for z in h.zones])) < 1e-9


def test_air_exchange_bounds():
    assert pm.air_exchange(0.0) == pm.K_LEAK
    assert pm.air_exchange(1.0) == pm.K_VENT
    assert pm.air_exchange(5.0) == pm.K_VENT


def test_probe_reads_zone_only_in_port():
    assert port_at(cfg.PORT_POS[2], cfg.PROBE_DOWN) == 2
    assert port_at(cfg.PORT_POS[2] + 0.2, cfg.PROBE_DOWN) == -1
    assert port_at(cfg.PORT_POS[2], 0.0) == -1

    h = run("coal_selfheat", 0.0, 48)
    probe = ProbeSensor(np.random.default_rng(0), 0, 0, 0, 0)
    for _ in range(100):  # 3.2 s settled in port 1
        r = probe.update(h, cfg.PORT_POS[1], cfg.PROBE_DOWN, 0.032)
    assert abs(r.co - h.zones[1].CO) < 0.1
    for _ in range(100):  # retracted: back to deck air
        r = probe.update(h, cfg.PORT_POS[1], 0.0, 0.032)
    assert r.co < 0.1
