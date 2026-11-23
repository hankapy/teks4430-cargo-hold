"""Sensor models used by plant_sim (no ROS).

ExhaustSensor  fixed gas sensor in the exhaust trunk: noise, optional CO drift
ProbeSensor    gas + IR sensor on the rover probe: reads zone air only when
               the probe is lowered into a sampling port, first-order lag
"""

import math
from dataclasses import dataclass

from hold_autonomy import config as cfg
from hold_autonomy import process_model as pm

PROBE_TAU_S = 0.5  # sensor time constant (simulation s)


@dataclass
class GasReading:
    co: float
    ch4: float
    o2: float
    surface_temp: float = pm.T_AMB


class ExhaustSensor:
    def __init__(self, rng, noise_co=0.5, noise_ch4=0.5, noise_o2=0.05, drift_ppm_h=0.0, drift_start_h=48.0):
        self.rng = rng
        self.noise = (noise_co, noise_ch4, noise_o2)
        self.drift_ppm_h = drift_ppm_h
        self.drift_start_h = drift_start_h

    def read(self, hold: pm.HoldState, vent_fraction: float, t_h: float) -> GasReading:
        air = pm.exhaust_air(hold, vent_fraction)
        drift = self.drift_ppm_h * max(t_h - self.drift_start_h, 0.0)
        n = self.noise
        return GasReading(
            co=max(0.0, air.CO + drift + self.rng.normal(0, n[0])),
            ch4=max(0.0, air.CH4 + self.rng.normal(0, n[1])),
            o2=air.O2 + self.rng.normal(0, n[2]),
        )


def port_at(rover_pos: float, probe_depth: float) -> int:
    """Zone index whose port the probe is in, or -1."""
    if probe_depth < cfg.PROBE_IN_PORT:
        return -1
    for i, p in enumerate(cfg.PORT_POS):
        if abs(rover_pos - p) <= cfg.PORT_TOL:
            return i
    return -1


class ProbeSensor:
    def __init__(self, rng, noise_co=0.5, noise_ch4=0.5, noise_o2=0.05, noise_t=0.2):
        self.rng = rng
        self.noise = (noise_co, noise_ch4, noise_o2, noise_t)
        self.value = GasReading(0.0, 0.0, pm.O2_AIR, pm.T_AMB)  # starts in deck air

    def update(self, hold: pm.HoldState, rover_pos: float, probe_depth: float, dt_s: float) -> GasReading:
        zone = port_at(rover_pos, probe_depth)
        if zone >= 0:
            z = hold.zones[zone]
            target = GasReading(z.CO, z.CH4, z.O2, pm.surface_temp(z))
        else:
            target = GasReading(0.0, 0.0, pm.O2_AIR, pm.T_AMB)  # open deck air
        a = 1.0 - math.exp(-dt_s / PROBE_TAU_S) if dt_s > 0 else 0.0
        v = self.value
        self.value = GasReading(
            v.co + a * (target.co - v.co),
            v.ch4 + a * (target.ch4 - v.ch4),
            v.o2 + a * (target.o2 - v.o2),
            v.surface_temp + a * (target.surface_temp - v.surface_temp),
        )
        n = self.noise
        v = self.value
        return GasReading(
            max(0.0, v.co + self.rng.normal(0, n[0])),
            max(0.0, v.ch4 + self.rng.normal(0, n[1])),
            v.o2 + self.rng.normal(0, n[2]),
            v.surface_temp + self.rng.normal(0, n[3]),
        )
