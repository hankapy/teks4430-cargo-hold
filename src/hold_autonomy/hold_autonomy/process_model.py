"""Simplified zonal process model for one cargo hold. No ROS dependencies.

The hold headspace is split into N_ZONES zones along its length. Each zone
has hotspot temperature T (C), O2 (% vol), CO (ppm) and CH4 (% LEL).
Neighbouring zones exchange gas through mixing. Surface ventilation
exchanges air in every zone.

Qualitative model: not calibrated against real cargo data.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple

from hold_autonomy.config import N_ZONES, EXHAUST_ZONE

# Air exchange rates (1/h)
K_VENT = 2.0  # full surface ventilation
K_LEAK = 0.02  # closed hold leakage
M_MIX = 0.05  # mixing between neighbouring zones

T_AMB = 15.0  # bulk cargo / ambient temp (C)
T_MAX = 150.0  # model not valid above (open fire)
O2_AIR = 20.9
EA_R = 6000.0  # activation energy / R (K)

HEAT_GAIN = 0.20  # C per unit oxidation
HEAT_LOSS = 0.012  # 1/h, conduction to bulk cargo
O2_USE = 0.5  # % O2 per unit oxidation
CO_YIELD = 2.5  # ppm per unit oxidation
SURFACE_FACTOR = 0.2  # surface temp sees 20 % of hotspot rise

# IMSBC thresholds
CH4_LIMIT = 20.0  # % LEL
CO_TWICE_DAILY = 30.0  # ppm
CO_ALARM = 50.0  # ppm
SETTLE_H = 4.0  # vent stopped before sampling


@dataclass
class Cargo:
    name: str
    ch4_source: float  # initial CH4 release per zone (% LEL / h)
    ch4_tau: float  # release decay time (h)
    reactivity: float  # oxidation rate at T_AMB, 21 % O2 (rel. units / h)
    profile: Tuple[float, ...] = (1.0,) * N_ZONES  # reactivity factor per zone


HOTSPOT = (0.15, 1.0, 0.15, 0.15)  # hotspot in zone 1, far from exhaust

CARGOES = {
    "coal_methane": Cargo("coal_methane", 12.0, 60.0, 0.0),
    "coal_selfheat": Cargo("coal_selfheat", 0.0, 1.0, 3.0, HOTSPOT),
    "coal_both": Cargo("coal_both", 12.0, 60.0, 3.0, HOTSPOT),
    "limestone": Cargo("limestone", 0.0, 1.0, 0.0),
}


@dataclass
class Zone:
    T: float = T_AMB
    O2: float = O2_AIR
    CO: float = 0.0
    CH4: float = 0.0


@dataclass
class HoldState:
    zones: List[Zone] = field(default_factory=lambda: [Zone() for _ in range(N_ZONES)])


def air_exchange(vent_fraction: float) -> float:
    """Air exchange rate (1/h) for ventilation fraction 0..1."""
    f = min(max(float(vent_fraction), 0.0), 1.0)
    return K_LEAK + (K_VENT - K_LEAK) * f


def oxidation_rate(z: Zone, reactivity: float) -> float:
    # Arrhenius rate, linear in O2
    tk = z.T + 273.15
    tk0 = T_AMB + 273.15
    return reactivity * math.exp(EA_R * (1.0 / tk0 - 1.0 / tk)) * max(z.O2, 0.0) / O2_AIR


def _mix(values, i):
    left = values[i - 1] if i > 0 else values[i]
    right = values[i + 1] if i < len(values) - 1 else values[i]
    return M_MIX * (left + right - 2.0 * values[i])


def step(h: HoldState, cargo: Cargo, vent_fraction: float, t_h: float, dt_h: float) -> HoldState:
    """Advance the model one explicit Euler step."""
    k = air_exchange(vent_fraction)
    ch4_src = cargo.ch4_source * math.exp(-t_h / cargo.ch4_tau)
    o2 = [z.O2 for z in h.zones]
    co = [z.CO for z in h.zones]
    ch4 = [z.CH4 for z in h.zones]
    new = []
    for i, z in enumerate(h.zones):
        r = oxidation_rate(z, cargo.reactivity * cargo.profile[i])
        co_frac = min(max((z.T - T_AMB) / 20.0, 0.0), 1.0)  # CO/CO2 ratio grows with T
        dT = HEAT_GAIN * r - HEAT_LOSS * (z.T - T_AMB)
        dO2 = k * (O2_AIR - z.O2) - O2_USE * r + _mix(o2, i)
        dCO = CO_YIELD * r * co_frac - k * z.CO + _mix(co, i)
        dCH4 = ch4_src - k * z.CH4 + _mix(ch4, i)
        new.append(Zone(
            T=min(z.T + dT * dt_h, T_MAX),
            O2=min(max(z.O2 + dO2 * dt_h, 0.0), O2_AIR),
            CO=max(z.CO + dCO * dt_h, 0.0),
            CH4=max(z.CH4 + dCH4 * dt_h, 0.0),
        ))
    return HoldState(new)


def exhaust_air(h: HoldState, vent_fraction: float) -> Zone:
    """Air at the fixed exhaust trunk sensor.

    Ventilated: mixed air from all zones. Closed: local air above EXHAUST_ZONE.
    """
    w = min(max(vent_fraction, 0.0), 1.0)
    n = len(h.zones)
    loc = h.zones[EXHAUST_ZONE]

    def blend(attr):
        mean = sum(getattr(z, attr) for z in h.zones) / n
        return w * mean + (1.0 - w) * getattr(loc, attr)

    return Zone(T=blend("T"), O2=blend("O2"), CO=blend("CO"), CH4=blend("CH4"))


def surface_temp(z: Zone) -> float:
    """Cargo surface temperature seen by the rover IR sensor."""
    return T_AMB + SURFACE_FACTOR * (z.T - T_AMB)
