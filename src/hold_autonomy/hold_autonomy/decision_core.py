"""Decision logic (no ROS): a py_trees behaviour tree.

Root (Sequence)
  PlanSampling            sets rover priority zone, always SUCCESS
  CheckSensorHealth       sets advisory on sensor fault, always SUCCESS
  Decide (Selector, priority order)
    MethaneEmergency  -> Purge          CH4 >= 20 % LEL (latched to 15)
    MethanePresent    -> ControlMethane CH4 > 5 % LEL (latched to 3), PI loop;
                                        setpoint raised to 15 % LEL if self-heating
    SelfHeating       -> Contain        close hold, starve oxygen
    Monitor                             normal state

Inerting is never automated: the tree raises CREW_ACTION and the crew decides.
"""

from dataclasses import dataclass

import py_trees
from py_trees.common import Status

from hold_autonomy.perception_core import PerceivedState, G_ALARM

# Vent modes (match VentCommand.msg)
CLOSED, CONTROL_CH4, PURGE = 0, 1, 2
# Alert levels (match Alert.msg)
NONE, ADVISORY, WARNING, CREW_ACTION = 0, 1, 2, 3

CH4_PURGE_ON, CH4_PURGE_OFF = 20.0, 15.0
CH4_CTRL_ON, CH4_CTRL_OFF = 5.0, 3.0
CH4_SETPOINT = 10.0
CH4_SETPOINT_CONFLICT = 15.0  # self-heating + methane: less air, smaller margin to 20 % LEL
SURFACE_T_CREW = 30.0  # C, hotspot clearly developing
G_CREW = 4.0 * G_ALARM


@dataclass
class Context:
    """Shared data between the node and the tree (blackboard)."""

    state: PerceivedState = None
    purge_latched: bool = False
    methane_latched: bool = False
    vent_mode: int = CLOSED
    ch4_setpoint: float = CH4_SETPOINT
    reason: str = ""
    alert_level: int = NONE
    alert_text: str = ""
    alert_zone: int = -1
    priority_zone: int = -1
    advisory: str = ""


class _Base(py_trees.behaviour.Behaviour):
    def __init__(self, name, ctx: Context):
        super().__init__(name)
        self.ctx = ctx


def _selfheat_text(s: PerceivedState) -> str:
    return f"self-heating suspected in zone {s.hotspot_zone} (CO generation {s.zone_co_gen[s.hotspot_zone]:.1f} ppm/h)"


def _severe(s: PerceivedState) -> bool:
    z = s.hotspot_zone
    return s.temp_max_c >= SURFACE_T_CREW or (z >= 0 and s.zone_co_gen[z] >= G_CREW)


class PlanSampling(_Base):
    def update(self):
        s = self.ctx.state
        if s.selfheat:
            self.ctx.priority_zone = s.hotspot_zone
        elif max(s.zone_suspect) > 0:
            self.ctx.priority_zone = max(range(len(s.zone_co_gen)), key=lambda i: s.zone_co_gen[i])
        else:
            self.ctx.priority_zone = -1
        return Status.SUCCESS


class CheckSensorHealth(_Base):
    def update(self):
        s = self.ctx.state
        self.ctx.advisory = s.fault_text if s.sensor_fault else ""
        return Status.SUCCESS


class MethaneEmergency(_Base):
    def update(self):
        ch4 = self.ctx.state.ch4_lel
        if ch4 >= CH4_PURGE_ON:
            self.ctx.purge_latched = True
        elif ch4 < CH4_PURGE_OFF:
            self.ctx.purge_latched = False
        return Status.SUCCESS if self.ctx.purge_latched else Status.FAILURE


class MethanePresent(_Base):
    def update(self):
        ch4 = self.ctx.state.ch4_lel
        if ch4 > CH4_CTRL_ON:
            self.ctx.methane_latched = True
        elif ch4 < CH4_CTRL_OFF:
            self.ctx.methane_latched = False
        return Status.SUCCESS if self.ctx.methane_latched else Status.FAILURE


class SelfHeating(_Base):
    def update(self):
        return Status.SUCCESS if self.ctx.state.selfheat else Status.FAILURE


class Purge(_Base):
    def update(self):
        c, s = self.ctx, self.ctx.state
        c.vent_mode, c.reason = PURGE, f"CH4 {s.ch4_lel:.1f} % LEL above limit"
        if s.selfheat:
            c.alert_level = CREW_ACTION
            c.alert_text = f"{_selfheat_text(s)} while methane forces full ventilation: consider inerting"
            c.alert_zone = s.hotspot_zone
        else:
            c.alert_level, c.alert_text, c.alert_zone = WARNING, f"methane {s.ch4_lel:.1f} % LEL, full ventilation", -1
        return Status.SUCCESS


class ControlMethane(_Base):
    def update(self):
        c, s = self.ctx, self.ctx.state
        c.vent_mode = CONTROL_CH4
        c.ch4_setpoint = CH4_SETPOINT_CONFLICT if s.selfheat else CH4_SETPOINT
        c.reason = f"CH4 {s.ch4_lel:.1f} % LEL, minimum ventilation, setpoint {c.ch4_setpoint:.0f} % LEL"
        if s.selfheat:
            c.alert_level = CREW_ACTION
            c.alert_text = f"{_selfheat_text(s)} while methane requires ventilation: consider inerting"
            c.alert_zone = s.hotspot_zone
        else:
            c.alert_level, c.alert_text, c.alert_zone = ADVISORY, "methane present, controlled ventilation", -1
        return Status.SUCCESS


class Contain(_Base):
    def update(self):
        c, s = self.ctx, self.ctx.state
        c.vent_mode, c.reason = CLOSED, "self-heating: hold closed to limit oxygen"
        c.alert_level = CREW_ACTION if _severe(s) else WARNING
        c.alert_text = _selfheat_text(s)
        if c.alert_level == CREW_ACTION:
            c.alert_text += ", developing: consider inerting"
        c.alert_zone = s.hotspot_zone
        return Status.SUCCESS


class Monitor(_Base):
    def update(self):
        c = self.ctx
        c.vent_mode, c.reason = CLOSED, "no hazard detected"
        c.alert_level = ADVISORY if c.advisory else NONE
        c.alert_text, c.alert_zone = c.advisory, -1
        return Status.SUCCESS


def build_tree(ctx: Context) -> py_trees.trees.BehaviourTree:
    def seq(name, *children):
        s = py_trees.composites.Sequence(name, memory=False)
        s.add_children(list(children))
        return s

    decide = py_trees.composites.Selector("Decide", memory=False)
    decide.add_children([
        seq("Methane emergency", MethaneEmergency("CH4 >= 20 % LEL?", ctx), Purge("Purge", ctx)),
        seq("Methane control", MethanePresent("CH4 present?", ctx), ControlMethane("Control CH4 (PI)", ctx)),
        seq("Self-heating", SelfHeating("Self-heating?", ctx), Contain("Contain", ctx)),
        Monitor("Monitor", ctx),
    ])
    root = seq("Root", PlanSampling("Plan sampling", ctx), CheckSensorHealth("Check sensors", ctx), decide)
    return py_trees.trees.BehaviourTree(root)


class DecisionMaker:
    """Convenience wrapper: tick the tree with a new perceived state."""

    def __init__(self):
        self.ctx = Context()
        self.tree = build_tree(self.ctx)

    def tick(self, state: PerceivedState) -> Context:
        self.ctx.state = state
        self.tree.tick()
        return self.ctx

    def ascii(self) -> str:
        return py_trees.display.unicode_tree(self.tree.root, show_status=True)
