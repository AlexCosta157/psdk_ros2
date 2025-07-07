uint8 event                 # event type
uint32 fc_timestamp         # flight controller timestamp
# Event data (union) - content depends on event type
uint8 interrupt_reason      # for event 0x01
uint8 recover_process       # for event 0x02
uint8 exit_reason          # for event 0x03
uint16 waypoint_index      # for event 0x10
WaypointV2MissionExecEvent mission_exec_event  # for event 0x11
uint8 avoid_state          # for event 0x12
WaypointV2ActionExecEvent action_exec_event    # for event 0x30