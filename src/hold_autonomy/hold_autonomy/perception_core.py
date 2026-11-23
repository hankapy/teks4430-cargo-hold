"""Perception logic (no ROS). Builds a world model of the hold.

Inputs:
  exhaust sensor   continuous CO, CH4, O2 at the exhaust trunk
  zone samples     intermittent CO, CH4, O2, surface temp from the rover
  actuator state   measured ventilation fraction

Key estimate: CO generation rate in each zone from a mass balance,
  g = dCO/dt + k(vent) * CO - m * (CO_left + CO_right - 2 CO)
which does not depend on how much the hold is ventilated. The mixing term
uses the latest samples of the neighbouring zones.
"""

from dataclasses import dataclass, field
from typing import List, Optional

from hold_autonomy.config import N_ZONES, EXHAUST_ZONE
from hold_autonomy import process_model as pm

G_ALARM = pm.CO_ALARM * pm.K_LEAK  # ppm/h, equals 50 ppm steady state in closed hold
CONFIRM_SAMPLES = 2  # consecutive zone estimates above G_ALARM
EXHAUST_TAU_H = 1.0  # EMA filter time constant for CO and O2 (process h)
CH4_TAU_H = 0.2  # faster filter for CH4, used by the PI loop
EXHAUST_DERIV_H = 1.0  # derivative window (process h)
MIN_SAMPLE_GAP_H = 0.2  # shorter gaps give a noisy derivative
FAULT_PPM = 5.0  # exhaust vs zone residual limit
FAULT_COUNT = 2
ZONE_MAX_AGE_H = 8.0  # zone samples older than this are not used for fault checks
CH4_ZONE_AGE_H = 1.0


@dataclass
class ZoneEstimate:
    co: float = 0.0
    ch4: float = 0.0
    o2: float = pm.O2_AIR
    surface_temp: float = pm.T_AMB
    co_gen: float = 0.0
    t_h: Optional[float] = None  # time of last sample
    k_int: float = 0.0  # cumulative air exchange at last sample
    suspect: int = 0  # consecutive estimates above G_ALARM


@dataclass
class PerceivedState:
    t_h: float = 0.0
    vent_fraction: float = 0.0
    h_since_vent_stop: float = 0.0
    raw_reliable: bool = False
    ch4_lel: float = 0.0  # for decisions: max of exhaust and recent zone samples
    ch4_exhaust: float = 0.0  # filtered, for the PI loop
    co_exhaust: float = 0.0
    co_gen_exhaust: float = 0.0
    o2_exhaust: float = pm.O2_AIR
    zone_co: List[float] = field(default_factory=lambda: [0.0] * N_ZONES)
    zone_co_gen: List[float] = field(default_factory=lambda: [0.0] * N_ZONES)
    zone_temp: List[float] = field(default_factory=lambda: [pm.T_AMB] * N_ZONES)
    zone_age_h: List[float] = field(default_factory=lambda: [-1.0] * N_ZONES)
    zone_suspect: List[int] = field(default_factory=lambda: [0] * N_ZONES)
    temp_max_c: float = pm.T_AMB
    selfheat: bool = False
    hotspot_zone: int = -1
    exhaust_selfheat: bool = False  # diagnostic: exhaust-only estimate
    sensor_fault: bool = False
    fault_text: str = ""


class Perception:
    def __init__(self):
        self.vent_fraction = 0.0
        self.t_h = None
        self.k_int = 0.0  # integral of air exchange over process time
        self.vent_stop_t = 0.0
        self.vent_on = False
        self.ch4_f = None
        self.co_f = None
        self.o2_f = None
        self.co_hist = []  # (t, co_f)
        self.co_gen_exhaust = 0.0
        self.exhaust_over_since = None
        self.exhaust_detect_h = None
        self.zones = [ZoneEstimate() for _ in range(N_ZONES)]
        self.res_ema = 0.0
        self.res_count = 0
        self.sensor_fault = False
        self.first_confirm_h = None

    # ------------------------------------------------------------ inputs

    def on_actuator(self, vent_fraction: float):
        self.vent_fraction = min(max(vent_fraction, 0.0), 1.0)
        on = self.vent_fraction > 0.05
        if self.vent_on and not on and self.t_h is not None:
            self.vent_stop_t = self.t_h
        self.vent_on = on

    def on_exhaust(self, t_h: float, co: float, ch4: float, o2: float):
        if self.t_h is None:
            self.t_h = t_h
            self.ch4_f, self.co_f, self.o2_f = ch4, co, o2
        dt = max(t_h - self.t_h, 0.0)
        self.k_int += pm.air_exchange(self.vent_fraction) * dt
        self.t_h = t_h

        self.ch4_f += min(dt / CH4_TAU_H, 1.0) * (ch4 - self.ch4_f)
        a = min(dt / EXHAUST_TAU_H, 1.0)
        self.co_f += a * (co - self.co_f)
        self.o2_f += a * (o2 - self.o2_f)

        # exhaust CO generation estimate (diagnostic / backup)
        self.co_hist.append((t_h, self.co_f))
        while self.co_hist and t_h - self.co_hist[0][0] > EXHAUST_DERIV_H:
            self.co_hist.pop(0)
        if len(self.co_hist) > 1 and t_h - self.co_hist[0][0] > 0.5 * EXHAUST_DERIV_H:
            dco = (self.co_f - self.co_hist[0][1]) / (t_h - self.co_hist[0][0])
            self.co_gen_exhaust = dco + pm.air_exchange(self.vent_fraction) * self.co_f
        if self.co_gen_exhaust > G_ALARM and not self.sensor_fault:
            if self.exhaust_over_since is None:
                self.exhaust_over_since = t_h
            if self.exhaust_detect_h is None and t_h - self.exhaust_over_since >= 1.0:
                self.exhaust_detect_h = t_h
        else:
            self.exhaust_over_since = None

    def on_zone_sample(self, t_h: float, zone: int, co: float, ch4: float, o2: float, surface_temp: float):
        if not 0 <= zone < N_ZONES:
            return
        z = self.zones[zone]
        k_now = self.k_int
        if z.t_h is not None and t_h - z.t_h >= MIN_SAMPLE_GAP_H:
            dt = t_h - z.t_h
            k_avg = (k_now - z.k_int) / dt
            co_mean = 0.5 * (co + z.co)
            z.co_gen = (co - z.co) / dt + k_avg * co_mean - self._mixing_inflow(zone, co_mean)
            z.suspect = z.suspect + 1 if z.co_gen > G_ALARM else 0
            if z.suspect >= CONFIRM_SAMPLES and self.first_confirm_h is None:
                self.first_confirm_h = t_h
        z.co, z.ch4, z.o2, z.surface_temp = co, ch4, o2, surface_temp
        z.t_h, z.k_int = t_h, k_now
        self._check_exhaust_sensor()

    def _mixing_inflow(self, zone: int, co_here: float) -> float:
        """CO flow from neighbouring zones (ppm/h), from their latest samples."""
        flow = 0.0
        for j in (zone - 1, zone + 1):
            if 0 <= j < N_ZONES and self.zones[j].t_h is not None:
                flow += pm.M_MIX * (self.zones[j].co - co_here)
        return flow

    # ------------------------------------------------------------ fault check

    def _check_exhaust_sensor(self):
        """Compare exhaust CO with the value expected from zone samples."""
        if self.co_f is None or self.t_h is None:
            return
        fresh = [z for z in self.zones if z.t_h is not None and self.t_h - z.t_h <= ZONE_MAX_AGE_H]
        loc = self.zones[EXHAUST_ZONE]
        if len(fresh) < N_ZONES or loc.t_h is None:
            return
        w = self.vent_fraction
        expected = w * sum(z.co for z in fresh) / len(fresh) + (1.0 - w) * loc.co
        res = self.co_f - expected
        self.res_ema += 0.3 * (res - self.res_ema)
        self.res_count = self.res_count + 1 if abs(self.res_ema) > FAULT_PPM else 0
        if self.res_count >= FAULT_COUNT:
            self.sensor_fault = True

    # ------------------------------------------------------------ output

    def state(self) -> PerceivedState:
        s = PerceivedState()
        t = self.t_h or 0.0
        s.t_h = t
        s.vent_fraction = self.vent_fraction
        s.h_since_vent_stop = 0.0 if self.vent_on else t - self.vent_stop_t
        s.raw_reliable = (not self.vent_on) and s.h_since_vent_stop >= pm.SETTLE_H
        s.ch4_exhaust = self.ch4_f or 0.0
        s.co_exhaust = self.co_f or 0.0
        s.o2_exhaust = self.o2_f if self.o2_f is not None else pm.O2_AIR
        s.co_gen_exhaust = self.co_gen_exhaust

        recent_ch4 = [z.ch4 for z in self.zones if z.t_h is not None and t - z.t_h <= CH4_ZONE_AGE_H]
        s.ch4_lel = max([s.ch4_exhaust] + recent_ch4)

        for i, z in enumerate(self.zones):
            s.zone_co[i] = z.co
            s.zone_co_gen[i] = z.co_gen
            s.zone_temp[i] = z.surface_temp
            s.zone_age_h[i] = -1.0 if z.t_h is None else t - z.t_h
            s.zone_suspect[i] = z.suspect
        s.temp_max_c = max(s.zone_temp)

        confirmed = [i for i, z in enumerate(self.zones) if z.suspect >= CONFIRM_SAMPLES]
        if confirmed:
            s.selfheat = True
            s.hotspot_zone = max(confirmed, key=lambda i: self.zones[i].co_gen)
        s.exhaust_selfheat = self.exhaust_detect_h is not None
        s.sensor_fault = self.sensor_fault
        if self.sensor_fault:
            s.fault_text = f"exhaust CO sensor disagrees with rover samples ({self.res_ema:+.1f} ppm)"
        return s
