# Measured actuator state from Webots position sensors (hold_driver)
float64 sim_time_s      # Webots simulation time, the time base of the system
float64 fan_speed_rad_s
float64 damper_pos_rad
float64 vent_fraction   # 0 = closed, 1 = full surface ventilation
float64 rover_pos_m     # along the rail, 0 = home
float64 rover_vel_m_s
float64 probe_depth_m   # 0 = retracted
