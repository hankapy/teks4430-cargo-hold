"""Behaviour tree tests with mock world states (no ROS)."""

from hold_autonomy.perception_core import PerceivedState, G_ALARM
from hold_autonomy import decision_core as dc


def state(ch4=0.0, selfheat=False, zone=1, gen=2 * G_ALARM, temp=16.0, fault=False):
    s = PerceivedState()
    s.ch4_lel = s.ch4_exhaust = ch4
    if selfheat:
        s.selfheat, s.hotspot_zone = True, zone
        s.zone_co_gen[zone] = gen
        s.zone_suspect[zone] = 2
    s.temp_max_c = temp
    s.sensor_fault = fault
    s.fault_text = "fault" if fault else ""
    return s


def test_normal_closed_no_alert():
    c = dc.DecisionMaker().tick(state())
    assert c.vent_mode == dc.CLOSED and c.alert_level == dc.NONE and c.priority_zone == -1


def test_methane_control_and_hysteresis():
    dm = dc.DecisionMaker()
    assert dm.tick(state(ch4=6.0)).vent_mode == dc.CONTROL_CH4
    assert dm.tick(state(ch4=4.0)).vent_mode == dc.CONTROL_CH4  # latched until < 3
    assert dm.tick(state(ch4=2.0)).vent_mode == dc.CLOSED


def test_purge_latch():
    dm = dc.DecisionMaker()
    assert dm.tick(state(ch4=21.0)).vent_mode == dc.PURGE
    assert dm.tick(state(ch4=16.0)).vent_mode == dc.PURGE  # latched until < 15
    assert dm.tick(state(ch4=14.0)).vent_mode == dc.CONTROL_CH4


def test_selfheat_contains_and_prioritises_zone():
    c = dc.DecisionMaker().tick(state(selfheat=True, zone=2))
    assert c.vent_mode == dc.CLOSED and c.alert_level == dc.WARNING
    assert c.priority_zone == 2 and c.alert_zone == 2


def test_severe_selfheat_requires_crew():
    c = dc.DecisionMaker().tick(state(selfheat=True, gen=5 * G_ALARM))
    assert c.alert_level == dc.CREW_ACTION


def test_conflict_raises_setpoint_and_crew_action():
    c = dc.DecisionMaker().tick(state(ch4=8.0, selfheat=True))
    assert c.vent_mode == dc.CONTROL_CH4
    assert c.ch4_setpoint == dc.CH4_SETPOINT_CONFLICT
    assert c.alert_level == dc.CREW_ACTION


def test_methane_emergency_overrides_selfheat():
    c = dc.DecisionMaker().tick(state(ch4=22.0, selfheat=True))
    assert c.vent_mode == dc.PURGE and c.alert_level == dc.CREW_ACTION


def test_sensor_fault_gives_advisory():
    c = dc.DecisionMaker().tick(state(fault=True))
    assert c.vent_mode == dc.CLOSED and c.alert_level == dc.ADVISORY
