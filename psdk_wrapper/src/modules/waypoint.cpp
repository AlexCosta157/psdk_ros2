
/**
 * @file waypoint_v2.cpp
 *
 * @brief Implementation of the WaypointV2Module class
 *
 * @authors [Your Name]
 * Contact: [your.email@unmanned.life]
 *
 */

#include "psdk_wrapper/modules/waypoint.hpp"

namespace psdk_ros2
{

// Static instance pointer for callbacks
WaypointV2Module* WaypointV2Module::instance_ = nullptr;

WaypointV2Module::WaypointV2Module(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))
{
  RCLCPP_INFO(get_logger(), "Creating WaypointV2Module");
  instance_ = this;
}

WaypointV2Module::~WaypointV2Module()
{
  RCLCPP_INFO(get_logger(), "Destroying WaypointV2Module");
  instance_ = nullptr;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring WaypointV2Module");
  /*
  // Create services
  waypoint_v2_upload_mission_service_ = create_service<WaypointV2UploadMission>(
      "psdk_ros2/waypoint_v2_upload_mission",
      std::bind(&WaypointV2Module::waypoint_v2_upload_mission_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  waypoint_v2_start_service_ = create_service<WaypointV2Start>(
      "psdk_ros2/waypoint_v2_start",
      std::bind(&WaypointV2Module::waypoint_v2_start_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  waypoint_v2_stop_service_ = create_service<WaypointV2Stop>(
      "psdk_ros2/waypoint_v2_stop",
      std::bind(&WaypointV2Module::waypoint_v2_stop_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  waypoint_v2_pause_service_ = create_service<WaypointV2Pause>(
      "psdk_ros2/waypoint_v2_pause",
      std::bind(&WaypointV2Module::waypoint_v2_pause_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  waypoint_v2_resume_service_ = create_service<WaypointV2Resume>(
      "psdk_ros2/waypoint_v2_resume",
      std::bind(&WaypointV2Module::waypoint_v2_resume_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  waypoint_v2_get_global_cruise_speed_service_ = create_service<WaypointV2GetGlobalCruiseSpeed>(
      "psdk_ros2/waypoint_v2_get_global_cruise_speed",
      std::bind(&WaypointV2Module::waypoint_v2_get_global_cruise_speed_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  waypoint_v2_set_global_cruise_speed_service_ = create_service<WaypointV2SetGlobalCruiseSpeed>(
      "psdk_ros2/waypoint_v2_set_global_cruise_speed",
      std::bind(&WaypointV2Module::waypoint_v2_set_global_cruise_speed_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  // Create publishers
  waypoint_v2_mission_event_pub_ = create_publisher<psdk_interfaces::msg::WaypointV2MissionEvent>(
      "psdk_ros2/waypoint_v2_mission_event", 10);

  waypoint_v2_mission_state_pub_ = create_publisher<psdk_interfaces::msg::WaypointV2MissionState>(
      "psdk_ros2/waypoint_v2_mission_state", 10);
*/
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating WaypointV2Module");
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating WaypointV2Module");
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up WaypointV2Module");
  
  waypoint_v2_upload_mission_service_.reset();
  waypoint_v2_start_service_.reset();
  waypoint_v2_stop_service_.reset();
  waypoint_v2_pause_service_.reset();
  waypoint_v2_resume_service_.reset();
  waypoint_v2_get_global_cruise_speed_service_.reset();
  waypoint_v2_set_global_cruise_speed_service_.reset();
  waypoint_v2_mission_event_pub_.reset();
  waypoint_v2_mission_state_pub_.reset();
  
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down WaypointV2Module");
  return CallbackReturn::SUCCESS;
}

bool WaypointV2Module::init()
{
  if (is_module_initialized_) {
    RCLCPP_INFO(get_logger(), "Waypoint V2 module already initialized, skipping.");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Initializing waypoint V2 module");
  
  // Add a try-catch block to handle potential crashes
  try {
    T_DjiReturnCode return_code = DjiWaypointV2_Init();
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_WARN(get_logger(),
                   "Could not initialize waypoint V2 module. Error code: %ld. This may be normal if waypoint v2 is not supported.",
                   return_code);
      // Don't fail if waypoint v2 is not supported
      return true;
    }
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Exception during waypoint V2 initialization. Continuing without waypoint v2.");
    return true;
  }

bool
WaypointV2Module::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing waypoint V2 module");
  T_DjiReturnCode return_code = DjiWaypointV2_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize waypoint V2 module. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

void
WaypointV2Module::waypoint_v2_upload_mission_cb(
    const std::shared_ptr<WaypointV2UploadMission::Request> request,
    const std::shared_ptr<WaypointV2UploadMission::Response> response)
{
  // Convert ROS message to DJI structure
  T_DjiWayPointV2MissionSettings mission_settings;
  mission_settings.missionID = request->mission_settings.mission_id;
  mission_settings.repeatTimes = request->mission_settings.repeat_times;
  mission_settings.finishedAction = static_cast<E_DJIWaypointV2MissionFinishedAction>(
      request->mission_settings.finished_action);
  mission_settings.maxFlightSpeed = request->mission_settings.max_flight_speed;
  mission_settings.autoFlightSpeed = request->mission_settings.auto_flight_speed;
  mission_settings.actionWhenRcLost = static_cast<E_DJIWaypointV2MissionActionWhenRcLost>(
      request->mission_settings.action_when_rc_lost);
  mission_settings.gotoFirstWaypointMode = static_cast<E_DJIWaypointV2MissionGotoFirstWaypointMode>(
      request->mission_settings.goto_first_waypoint_mode);

  // Convert waypoints
  mission_settings.missTotalLen = request->mission_settings.waypoints.size();
  std::vector<T_DjiWaypointV2> waypoints(mission_settings.missTotalLen);
  
  for (size_t i = 0; i < request->mission_settings.waypoints.size(); ++i)
  {
    const auto& ros_wp = request->mission_settings.waypoints[i];
    auto& dji_wp = waypoints[i];
    
    dji_wp.longitude = ros_wp.longitude;
    dji_wp.latitude = ros_wp.latitude;
    dji_wp.relativeHeight = ros_wp.relative_height;
    dji_wp.waypointType = static_cast<E_DJIWaypointV2FlightPathMode>(ros_wp.waypoint_type);
    dji_wp.headingMode = static_cast<E_DJIWaypointV2HeadingMode>(ros_wp.heading_mode);
    dji_wp.config.useLocalCruiseVel = ros_wp.use_local_cruise_vel;
    dji_wp.config.useLocalMaxVel = ros_wp.use_local_max_vel;
    dji_wp.dampingDistance = ros_wp.damping_distance;
    dji_wp.heading = ros_wp.heading;
    dji_wp.turnMode = static_cast<E_DJIWaypointV2TurnMode>(ros_wp.turn_mode);
    dji_wp.pointOfInterest.positionX = ros_wp.point_of_interest.position_x;
    dji_wp.pointOfInterest.positionY = ros_wp.point_of_interest.position_y;
    dji_wp.pointOfInterest.positionZ = ros_wp.point_of_interest.position_z;
    dji_wp.maxFlightSpeed = ros_wp.max_flight_speed;
    dji_wp.autoFlightSpeed = ros_wp.auto_flight_speed;
  }
  mission_settings.mission = waypoints.data();

  // Convert actions
  mission_settings.actionList.actionNum = request->mission_settings.actions.size();
  std::vector<T_DJIWaypointV2Action> actions(mission_settings.actionList.actionNum);
  
  for (size_t i = 0; i < request->mission_settings.actions.size(); ++i)
  {
    const auto& ros_action = request->mission_settings.actions[i];
    auto& dji_action = actions[i];
    
    dji_action.actionId = ros_action.action_id;
    
    // Convert trigger
    dji_action.trigger.actionTriggerType = ros_action.trigger.action_trigger_type;
    switch (dji_action.trigger.actionTriggerType)
    {
      case DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_SAMPLE_REACH_POINT:
        dji_action.trigger.sampleReachPointTriggerParam.waypointIndex = 
            ros_action.trigger.sample_reach_point_param.waypoint_index;
        dji_action.trigger.sampleReachPointTriggerParam.terminateNum = 
            ros_action.trigger.sample_reach_point_param.terminate_num;
        break;
      case DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED:
        dji_action.trigger.associateTriggerParam.actionAssociatedType = 
            ros_action.trigger.associate_param.action_associated_type;
        dji_action.trigger.associateTriggerParam.waitTimeUint = 
            ros_action.trigger.associate_param.wait_time_unit;
        dji_action.trigger.associateTriggerParam.waitingTime = 
            ros_action.trigger.associate_param.waiting_time;
        dji_action.trigger.associateTriggerParam.actionIdAssociated = 
            ros_action.trigger.associate_param.action_id_associated;
        break;
      case DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_TRAJECTORY:
        dji_action.trigger.trajectoryTriggerParam.startIndex = 
            ros_action.trigger.trajectory_param.start_index;
        dji_action.trigger.trajectoryTriggerParam.endIndex = 
            ros_action.trigger.trajectory_param.end_index;
        break;
      case DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_INTERVAL:
        dji_action.trigger.intervalTriggerParam.startIndex = 
            ros_action.trigger.interval_param.start_index;
        dji_action.trigger.intervalTriggerParam.interval = 
            ros_action.trigger.interval_param.interval;
        dji_action.trigger.intervalTriggerParam.actionIntervalType = 
            ros_action.trigger.interval_param.action_interval_type;
        break;
    }
    
    // Convert actuator
    dji_action.actuator.actuatorType = ros_action.actuator.actuator_type;
    dji_action.actuator.actuatorIndex = ros_action.actuator.actuator_index;
    
    switch (dji_action.actuator.actuatorType)
    {
      case DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA:
        // Convert camera actuator parameters
        dji_action.actuator.cameraActuatorParam.operationType = 
            ros_action.actuator.camera_param.operation_type;
        // Add specific camera parameter conversions based on operation type
        break;
      case DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL:
        // Convert gimbal actuator parameters
        dji_action.actuator.gimbalActuatorParam.operationType = 
            ros_action.actuator.gimbal_param.operation_type;
        dji_action.actuator.gimbalActuatorParam.rotation.x = 
            ros_action.actuator.gimbal_param.rotation.x;
        dji_action.actuator.gimbalActuatorParam.rotation.y = 
            ros_action.actuator.gimbal_param.rotation.y;
        dji_action.actuator.gimbalActuatorParam.rotation.z = 
            ros_action.actuator.gimbal_param.rotation.z;
        dji_action.actuator.gimbalActuatorParam.rotation.ctrl_mode = 
            ros_action.actuator.gimbal_param.rotation.ctrl_mode;
        dji_action.actuator.gimbalActuatorParam.rotation.rollCmdIgnore = 
            ros_action.actuator.gimbal_param.rotation.roll_cmd_ignore;
        dji_action.actuator.gimbalActuatorParam.rotation.pitchCmdIgnore = 
            ros_action.actuator.gimbal_param.rotation.pitch_cmd_ignore;
        dji_action.actuator.gimbalActuatorParam.rotation.yawCmdIgnore = 
            ros_action.actuator.gimbal_param.rotation.yaw_cmd_ignore;
        dji_action.actuator.gimbalActuatorParam.rotation.absYawModeRef = 
            ros_action.actuator.gimbal_param.rotation.abs_yaw_mode_ref;
        dji_action.actuator.gimbalActuatorParam.rotation.durationTime = 
            ros_action.actuator.gimbal_param.rotation.duration_time;
        break;
      case DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL:
        // Convert aircraft control actuator parameters
        dji_action.actuator.aircraftControlActuatorParam.operationType = 
            ros_action.actuator.aircraft_control_param.operation_type;
        // Add specific aircraft control parameter conversions based on operation type
        break;
    }
  }
  mission_settings.actionList.actions = actions.data();

  T_DjiReturnCode return_code = DjiWaypointV2_UploadMission(&mission_settings);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Upload waypoint mission failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Waypoint mission uploaded successfully");
  response->success = true;
}

void
WaypointV2Module::waypoint_v2_start_cb(
    const std::shared_ptr<WaypointV2Start::Request> request,
    const std::shared_ptr<WaypointV2Start::Response> response)
{
  (void)request;
  T_DjiReturnCode return_code = DjiWaypointV2_Start();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Start waypoint mission failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Waypoint mission started successfully");
  response->success = true;
}

void
WaypointV2Module::waypoint_v2_stop_cb(
    const std::shared_ptr<WaypointV2Stop::Request> request,
    const std::shared_ptr<WaypointV2Stop::Response> response)
{
  (void)request;
  T_DjiReturnCode return_code = DjiWaypointV2_Stop();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Stop waypoint mission failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Waypoint mission stopped successfully");
  response->success = true;
}

void
WaypointV2Module::waypoint_v2_pause_cb(
    const std::shared_ptr<WaypointV2Pause::Request> request,
    const std::shared_ptr<WaypointV2Pause::Response> response)
{
  (void)request;
  T_DjiReturnCode return_code = DjiWaypointV2_Pause();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Pause waypoint mission failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Waypoint mission paused successfully");
  response->success = true;
}

void
WaypointV2Module::waypoint_v2_resume_cb(
    const std::shared_ptr<WaypointV2Resume::Request> request,
    const std::shared_ptr<WaypointV2Resume::Response> response)
{
  (void)request;
  T_DjiReturnCode return_code = DjiWaypointV2_Resume();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Resume waypoint mission failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Waypoint mission resumed successfully");
  response->success = true;
}

void
WaypointV2Module::waypoint_v2_get_global_cruise_speed_cb(
    const std::shared_ptr<WaypointV2GetGlobalCruiseSpeed::Request> request,
    const std::shared_ptr<WaypointV2GetGlobalCruiseSpeed::Response> response)
{
  (void)request;
  T_DjiWaypointV2GlobalCruiseSpeed cruise_speed;
  T_DjiReturnCode return_code = DjiWaypointV2_GetGlobalCruiseSpeed(&cruise_speed);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Get global cruise speed failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  
  response->cruise_speed = cruise_speed;
  response->success = true;
  RCLCPP_INFO(get_logger(), "Global cruise speed: %.2f m/s", cruise_speed);
}

void
WaypointV2Module::waypoint_v2_set_global_cruise_speed_cb(
    const std::shared_ptr<WaypointV2SetGlobalCruiseSpeed::Request> request,
    const std::shared_ptr<WaypointV2SetGlobalCruiseSpeed::Response> response)
{
  T_DjiWaypointV2GlobalCruiseSpeed cruise_speed = request->cruise_speed;
  T_DjiReturnCode return_code = DjiWaypointV2_SetGlobalCruiseSpeed(cruise_speed);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Set global cruise speed failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Global cruise speed set to %.2f m/s", cruise_speed);
  response->success = true;
}

T_DjiReturnCode
WaypointV2Module::waypoint_v2_mission_event_callback(T_DjiWaypointV2MissionEventPush eventData)
{
  if (instance_ == nullptr)
  {
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  }

  auto event_msg = psdk_interfaces::msg::WaypointV2MissionEvent();
  event_msg.event = eventData.event;
  event_msg.fc_timestamp = eventData.FCTimestamp;

  // Fill event-specific data based on event type
  switch (eventData.event)
  {
    case 0x01: // mission interrupt event
      event_msg.interrupt_reason = eventData.data.interruptReason;
      break;
    case 0x02: // mission resume event
      event_msg.recover_process = eventData.data.recoverProcess;
      break;
    case 0x03: // mission stop event
      event_msg.exit_reason = eventData.data.exitReason;
      break;
    case 0x10: // mission arrival event
      event_msg.waypoint_index = eventData.data.waypointIndex;
      break;
    case 0x11: // route finished event
      event_msg.mission_exec_event.current_mission_exec_times = 
          eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes;
      event_msg.mission_exec_event.finished_all_miss_exec_times = 
          eventData.data.T_DjiWaypointV2MissionExecEvent.finishedAllMissExecTimes;
      break;
    case 0x12: // avoid obstacle event
      event_msg.avoid_state = eventData.data.avoidState;
      break;
    case 0x30: // action switch event
      event_msg.action_exec_event.action_id = 
          eventData.data.T_DjiWaypointV2ActionExecEvent.actionId;
      event_msg.action_exec_event.pre_actuator_state = 
          eventData.data.T_DjiWaypointV2ActionExecEvent.preActuatorState;
      event_msg.action_exec_event.cur_actuator_state = 
          eventData.data.T_DjiWaypointV2ActionExecEvent.curActuatorState;
      event_msg.action_exec_event.result = 
          eventData.data.T_DjiWaypointV2ActionExecEvent.result;
      break;
  }

  instance_->waypoint_v2_mission_event_pub_->publish(event_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
WaypointV2Module::waypoint_v2_mission_state_callback(T_DjiWaypointV2MissionStatePush stateData)
{
  if (instance_ == nullptr)
  {
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  }

  auto state_msg = psdk_interfaces::msg::WaypointV2MissionState();
  state_msg.cur_waypoint_index = stateData.curWaypointIndex;
  state_msg.state = stateData.state;
  state_msg.velocity = stateData.velocity;

  instance_->waypoint_v2_mission_state_pub_->publish(state_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

}  // namespace psdk_ros2