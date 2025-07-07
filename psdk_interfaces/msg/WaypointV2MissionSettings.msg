uint32 mission_id
uint8 repeat_times
uint8 finished_action          # see E_DJIWaypointV2MissionFinishedAction
float32 max_flight_speed       # unit: m/s, range [2,15]
float32 auto_flight_speed      # unit: m/s, range [-15,15]
uint8 action_when_rc_lost      # see E_DJIWaypointV2MissionActionWhenRcLost
uint8 goto_first_waypoint_mode # see E_DJIWaypointV2MissionGotoFirstWaypointMode
WaypointV2[] waypoints
WaypointV2Action[] actions