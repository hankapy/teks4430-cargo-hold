# Ventilation decision (hold_manager -> vent_control)
uint8 CLOSED=0
uint8 CONTROL_CH4=1    # PI keeps CH4 at setpoint with least airflow
uint8 PURGE=2          # full ventilation
uint8 mode
float64 ch4_setpoint
string reason
