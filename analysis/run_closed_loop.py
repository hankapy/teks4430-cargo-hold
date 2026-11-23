"""Closed-loop simulation without ROS or Webots.

Wires together the same logic classes that the ROS 2 nodes use
(process model, sensor models, perception, behaviour tree, vent and rover
controllers) and emulates the Webots actuators (fan acceleration, damper
speed, rover acceleration, probe speed). Compares the autonomous system with
the manual IMSBC procedure.

Usage: python3 analysis/run_closed_loop.py
"""

import csv
import os
import sys
from dataclasses import dataclass

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "hold_autonomy"))

from hold_autonomy import config as cfg  # noqa: E402
from hold_autonomy import process_model as pm  # noqa: E402
from hold_autonomy.sensor_model import ExhaustSensor, ProbeSensor  # noqa: E402
from hold_autonomy.perception_core import Perception  # noqa: E402
from hold_autonomy.decision_core import DecisionMaker  # noqa: E402
from hold_autonomy.control_core import VentController, RoverController  # noqa: E402

OUT_DIR = os.path.join(HERE, "out")
DT_S = 0.032  # Webots basic time step
TIME_SCALE = 600.0  # process s per simulation s
NODE_EVERY = 3  # perception / decision / vent run at ~10 Hz
DURATION_H = 120.0


@dataclass
class Scenario:
    name: str
    cargo: str
    drift_ppm_h: float = 0.0
    drift_start_h: float = 24.0


SCENARIOS = [
    Scenario("coal_methane", "coal_methane"),
    Scenario("coal_selfheat", "coal_selfheat"),
    Scenario("coal_both", "coal_both"),
    Scenario("limestone", "limestone"),
    Scenario("limestone_co_drift", "limestone", drift_ppm_h=1.0),
]


def approach(x, target, rate, dt):
    step = rate * dt
    return target if abs(target - x) <= step else x + step * np.sign(target - x)


class Actuators:
    """Emulates the Webots motors."""

    def __init__(self):
        self.fan = self.damper = 0.0
        self.pos = cfg.HOME_POS
        self.vel = 0.0
        self.probe = 0.0

    def step(self, fan_cmd, damper_cmd, vel_cmd, probe_cmd, dt):
        self.fan = approach(self.fan, fan_cmd, cfg.FAN_ACCEL, dt)
        self.damper = approach(self.damper, damper_cmd, cfg.DAMPER_SPEED, dt)
        self.vel = approach(self.vel, vel_cmd, cfg.ROVER_ACCEL, dt)
        self.pos = min(max(self.pos + self.vel * dt, 0.0), cfg.RAIL_LENGTH)
        self.probe = approach(self.probe, probe_cmd, cfg.PROBE_SPEED, dt)

    @property
    def vent_fraction(self):
        return min(max(self.fan / cfg.FAN_NOMINAL, 0.0), 1.0) * min(max(self.damper / cfg.DAMPER_OPEN, 0.0), 1.0)


def first_time(t, mask):
    idx = np.flatnonzero(mask)
    return float(t[idx[0]]) if idx.size else None


def run_autonomous(sc: Scenario, seed=1):
    rng = np.random.default_rng(seed)
    cargo = pm.CARGOES[sc.cargo]
    hold = pm.HoldState()
    exhaust = ExhaustSensor(rng, drift_ppm_h=sc.drift_ppm_h, drift_start_h=sc.drift_start_h)
    probe = ProbeSensor(rng)
    per = Perception()
    dm = DecisionMaker()
    vent = VentController()
    rover = RoverController()
    act = Actuators()

    fan_cmd = damper_cmd = vel_cmd = probe_cmd = 0.0
    priority = -1
    ctx = None
    t_s, t_h, i = 0.0, 0.0, 0
    dt_h = DT_S * TIME_SCALE / 3600.0
    log, samples, rover_trace = [], [], []
    crew_h = fault_h = None
    vent_on_prev, vent_switches = False, 0

    while t_h < DURATION_H:
        # actuators and process
        act.step(fan_cmd, damper_cmd, vel_cmd, probe_cmd, DT_S)
        vf = act.vent_fraction
        hold = pm.step(hold, cargo, vf, t_h, dt_h)
        t_s += DT_S
        t_h += dt_h
        reading = probe.update(hold, act.pos, act.probe, DT_S)

        # rover node (every step)
        vel_cmd, probe_cmd, sample = rover.update(t_s, act.pos, act.vel, act.probe, priority, reading)
        if sample is not None:
            per.on_zone_sample(t_h, sample.zone, sample.co, sample.ch4, sample.o2, sample.surface_temp)
            samples.append((t_h, sample.zone, sample.co))
        if t_s <= 120.0:
            rover_trace.append((t_s, act.pos, act.vel, act.probe, rover.state))

        # perception, decision, vent control (~10 Hz)
        if i % NODE_EVERY == 0:
            per.on_actuator(vf)
            ex = exhaust.read(hold, vf, t_h)
            per.on_exhaust(t_h, ex.co, ex.ch4, ex.o2)
            st = per.state()
            ctx = dm.tick(st)
            priority = ctx.priority_zone
            fan_cmd, damper_cmd = vent.update(t_h, ctx.vent_mode, st.ch4_exhaust, ctx.ch4_setpoint, act.damper, act.fan)
            if ctx.alert_level >= 3 and crew_h is None:
                crew_h = t_h
            if st.sensor_fault and fault_h is None:
                fault_h = t_h
            on = vf > 0.02
            vent_switches += int(on != vent_on_prev)
            vent_on_prev = on

        if i % 47 == 0:  # ~0.25 process h
            zs = hold.zones
            log.append([t_h] + [z.T for z in zs] + [z.CO for z in zs] + [z.CH4 for z in zs]
                       + [vf, ctx.vent_mode if ctx else 0, ctx.alert_level if ctx else 0,
                          per.state().co_exhaust, per.state().ch4_exhaust])
        i += 1

    data = np.array(log)
    st = per.state()
    true_hot = int(np.argmax(cargo.profile)) if cargo.reactivity > 0 else -1
    n = cfg.N_ZONES
    metrics = {
        "scenario": sc.name, "system": "autonomous",
        "detect_h": per.first_confirm_h,
        "detect_exhaust_only_h": per.exhaust_detect_h,
        "hotspot_found": st.hotspot_zone, "hotspot_true": true_hot,
        "crew_action_h": crew_h, "fault_h": fault_h,
        "max_T": data[:, 1:1 + n].max(),
        "max_CH4": data[:, 1 + 2 * n:1 + 3 * n].max(),
        "h_CH4_over_20": (data[:, 1 + 2 * n:1 + 3 * n].max(axis=1) > pm.CH4_LIMIT).sum() * (log[1][0] - log[0][0]),
        "mean_vent": data[:, 1 + 3 * n].mean(),
        "vent_switches": vent_switches,
        "rover_visits": sum(rover.visits), "rover_faults": rover.faults,
        "t_T60_h": first_time(data[:, 0], data[:, 1:1 + n].max(axis=1) >= 60.0),
    }
    return data, samples, rover_trace, metrics


def run_manual(sc: Scenario, seed=1):
    """Manual IMSBC procedure: vent first 24 h, crew samples all ports daily
    (twice daily if CO > 30 ppm), vent kept on if CH4 > 20 % LEL except
    SETTLE_H before sampling."""
    rng = np.random.default_rng(seed)
    cargo = pm.CARGOES[sc.cargo]
    hold = pm.HoldState()
    dt = 1.0 / 60.0
    t, next_sample = 0.0, 32.0
    ch4_high = twice = False
    hist, detect, log = [], None, []
    while t < DURATION_H:
        if t >= next_sample:
            co = max(z.CO for z in hold.zones) + rng.normal(0, 1.0)
            ch4 = max(z.CH4 for z in hold.zones) + rng.normal(0, 1.0)
            ch4_high = ch4 > pm.CH4_LIMIT
            hist.append(co)
            twice = twice or co > pm.CO_TWICE_DAILY
            rising = len(hist) >= 3 and hist[-1] > hist[-2] > hist[-3] and co > pm.CO_TWICE_DAILY
            if detect is None and (co >= pm.CO_ALARM or rising):
                detect = t
            next_sample = t + (12.0 if twice else 24.0)
        vent = 1.0 if t < 24.0 or (ch4_high and next_sample - t > pm.SETTLE_H) else 0.0
        hold = pm.step(hold, cargo, vent, t, dt)
        t += dt
        if int(t * 60) % 15 == 0:
            log.append([t] + [z.T for z in hold.zones] + [z.CH4 for z in hold.zones] + [vent])
    data = np.array(log)
    n = cfg.N_ZONES
    return data, {
        "scenario": sc.name, "system": "IMSBC_manual", "detect_h": detect,
        "max_T": data[:, 1:1 + n].max(), "max_CH4": data[:, 1 + n:1 + 2 * n].max(),
        "h_CH4_over_20": (data[:, 1 + n:1 + 2 * n].max(axis=1) > pm.CH4_LIMIT).sum() * 0.25,
        "mean_vent": data[:, -1].mean(),
        "t_T60_h": first_time(data[:, 0], data[:, 1:1 + n].max(axis=1) >= 60.0),
    }


def plot_scenario(sc, data, samples, manual, path):
    n = cfg.N_ZONES
    t = data[:, 0]
    fig, ax = plt.subplots(5, 1, figsize=(9, 12), sharex=True)
    for z in range(n):
        ax[0].plot(t, data[:, 1 + z], label=f"zone {z}")
        ax[1].plot(t, data[:, 1 + n + z], label=f"zone {z}")
    ax[0].plot(manual[:, 0], manual[:, 1:1 + n].max(axis=1), "k--", lw=1, label="manual: max")
    if samples:
        s = np.array(samples)
        for z in range(n):
            m = s[:, 1] == z
            ax[1].plot(s[m, 0], s[m, 2], ".", ms=3, color=f"C{z}")
    ax[1].plot(t, data[:, -2], "k:", lw=1, label="exhaust sensor (filtered)")
    ax[2].plot(t, data[:, 1 + 2 * n:1 + 3 * n].max(axis=1), label="autonomous: max zone")
    ax[2].plot(manual[:, 0], manual[:, 1 + n:1 + 2 * n].max(axis=1), "k--", lw=1, label="manual: max zone")
    ax[2].axhline(pm.CH4_LIMIT, color="r", lw=0.8, ls="--")
    ax[2].axhline(10.0, color="grey", lw=0.8, ls=":")
    ax[3].plot(t, data[:, 1 + 3 * n], label="vent fraction")
    ax[3].plot(manual[:, 0], manual[:, -1], "k--", lw=1, label="manual")
    ax[4].step(t, data[:, 2 + 3 * n], where="post", label="vent mode (0 closed, 1 PI, 2 purge)")
    ax[4].step(t, data[:, 3 + 3 * n], where="post", label="alert level")
    labels = ["hotspot T (C)", "CO (ppm)", "CH4 (% LEL)", "ventilation", "decision"]
    for a, l in zip(ax, labels):
        a.set_ylabel(l)
        a.grid(alpha=0.3)
        a.legend(fontsize=7, loc="upper right")
    ax[0].set_title(sc.name)
    ax[-1].set_xlabel("time since loading (h)")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def plot_rover(trace, path):
    tr = np.array([r[:4] for r in trace], dtype=float)
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(tr[:, 0], tr[:, 1])
    for p in cfg.PORT_POS:
        ax[0].axhline(p, color="grey", lw=0.6, ls=":")
    ax[1].plot(tr[:, 0], tr[:, 2])
    ax[2].plot(tr[:, 0], tr[:, 3])
    for a, l in zip(ax, ["rover position (m)", "velocity (m/s)", "probe depth (m)"]):
        a.set_ylabel(l)
        a.grid(alpha=0.3)
    ax[0].set_title("Sampling rover: first 120 s of simulation")
    ax[-1].set_xlabel("simulation time (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def fmt(v, p=1):
    return "-" if v is None else (f"{v:.{p}f}" if isinstance(v, float) else str(v))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = []
    for sc in SCENARIOS:
        data, samples, trace, m = run_autonomous(sc)
        mdata, mm = run_manual(sc)
        plot_scenario(sc, data, samples, mdata, os.path.join(OUT_DIR, f"{sc.name}.png"))
        if sc.name == "coal_selfheat":
            plot_rover(trace, os.path.join(OUT_DIR, "rover_trace.png"))
        rows += [m, mm]
        print(f"\n{sc.name}")
        print(f"  manual     detect {fmt(mm['detect_h'])} h, max T {mm['max_T']:.1f}, max CH4 {mm['max_CH4']:.1f}, "
              f"h CH4>20 {mm['h_CH4_over_20']:.1f}, mean vent {mm['mean_vent']:.2f}, T>=60 at {fmt(mm['t_T60_h'])} h")
        print(f"  autonomous detect {fmt(m['detect_h'])} h (exhaust only {fmt(m['detect_exhaust_only_h'])} h), "
              f"hotspot {m['hotspot_found']} (true {m['hotspot_true']}), crew action {fmt(m['crew_action_h'])} h, "
              f"fault {fmt(m['fault_h'])} h")
        print(f"             max T {m['max_T']:.1f}, max CH4 {m['max_CH4']:.1f}, h CH4>20 {m['h_CH4_over_20']:.1f}, "
              f"mean vent {m['mean_vent']:.2f}, vent switches {m['vent_switches']}, "
              f"rover visits {m['rover_visits']}, rover faults {m['rover_faults']}, T>=60 at {fmt(m['t_T60_h'])} h")

    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(OUT_DIR, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    main()
