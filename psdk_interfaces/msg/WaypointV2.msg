float64 longitude              # unit: degrees
float64 latitude               # unit: degrees  
float32 relative_height        # unit: m, relative to takeoff height
uint8 waypoint_type           # see E_DJIWaypointV2FlightPathMode
uint8 heading_mode            # see E_DJIWaypointV2HeadingMode
uint16 use_local_cruise_vel   # 0: use local cruise speed, 1: don't use
uint16 use_local_max_vel      # 0: use local max speed, 1: don't use
uint16 damping_distance       # unit: m
float32 heading               # unit: degrees, range [-180, 180]
uint8 turn_mode              # see E_DJIWaypointV2TurnMode
WaypointV2RelativePosition point_of_interest
float32 max_flight_speed     # unit: m/s, range [2,15]
float32 auto_flight_speed    # unit: m/s, range [-15,15]