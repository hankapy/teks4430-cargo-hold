# Alert to crew (hold_manager -> vent_control)
uint8 NONE=0
uint8 ADVISORY=1
uint8 WARNING=2
uint8 CREW_ACTION=3     # human decision required, e.g. inerting
uint8 level
string text
int32 zone              # -1 = whole hold
