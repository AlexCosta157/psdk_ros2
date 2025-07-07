uint8 actuator_type          # see E_DJIWaypointV2ActionActuatorType
uint8 actuator_index
# Union data - only one will be used based on actuator type
WaypointV2CameraActuatorParam camera_param
WaypointV2GimbalActuatorParam gimbal_param
WaypointV2AircraftControlParam aircraft_control_param