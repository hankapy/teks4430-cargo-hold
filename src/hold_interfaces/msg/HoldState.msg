# World model of the hold (perception -> hold_manager, vent_control)
float64 t_h
float64 vent_fraction
float64 h_since_vent_stop
bool raw_reliable            # exhaust CO usable without correction
float64 ch4_lel              # max of exhaust and recent zone samples
float64 ch4_exhaust          # filtered, used by the PI loop
float64 co_exhaust
float64 co_gen_exhaust
float64 o2_exhaust
float64[] zone_co
float64[] zone_co_gen        # estimated CO generation (ppm/h)
float64[] zone_temp          # cargo surface temp
float64[] zone_age_h         # -1 = never sampled
int32[] zone_suspect
float64 temp_max_c
bool selfheat
int32 hotspot_zone           # -1 = none
bool exhaust_selfheat        # diagnostic: exhaust-only estimate
bool sensor_fault
string fault_text
