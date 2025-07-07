/**
 * @file waypoint_v2.hpp
 *
 * @brief Header file for the WaypointV2Module class
 *
 * @authors [Your Name]
 * Contact: [your.email@.life]
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WAYPOINT_V2_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WAYPOINT_V2_HPP_

#include <dji_waypoint_v2.h>  //NOLINT

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

#include "psdk_interfaces/msg/waypoint_v2_mission_event.hpp"
#include "psdk_interfaces/msg/waypoint_v2_mission_state.hpp"
#include "psdk_interfaces/srv/waypoint_v2_upload_mission.hpp"
#include "psdk_interfaces/srv/waypoint_v2_start.hpp"
#include "psdk_interfaces/srv/waypoint_v2_stop.hpp"
#include "psdk_interfaces/srv/waypoint_v2_pause.hpp"
#include "psdk_interfaces/srv/waypoint_v2_resume.hpp"
#include "psdk_interfaces/srv/waypoint_v2_get_global_cruise_speed.hpp"
#include "psdk_interfaces/srv/waypoint_v2_set_global_cruise_speed.hpp"
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{

class WaypointV2Module : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using WaypointV2UploadMission = psdk_interfaces::srv::WaypointV2UploadMission;
  using WaypointV2Start = psdk_interfaces::srv::WaypointV2Start;
  using WaypointV2Stop = psdk_interfaces::srv::WaypointV2Stop;
  using WaypointV2Pause = psdk_interfaces::srv::WaypointV2Pause;
  using WaypointV2Resume = psdk_interfaces::srv::WaypointV2Resume;
  using WaypointV2GetGlobalCruiseSpeed = psdk_interfaces::srv::WaypointV2GetGlobalCruiseSpeed;
  using WaypointV2SetGlobalCruiseSpeed = psdk_interfaces::srv::WaypointV2SetGlobalCruiseSpeed;

  /**
   * @brief Construct a new WaypointV2Module object
   * @param node_name Name of the node
   */
  explicit WaypointV2Module(const std::string& name);

  /**
   * @brief Destroy the WaypointV2Module object
   */
  ~WaypointV2Module();

  /**
   * @brief Configures the WaypointV2Module. Creates the ROS 2 publishers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the WaypointV2Module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Cleans the WaypointV2Module. Resets the ROS 2 publishers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);

  /**
   * @brief Deactivates the WaypointV2Module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Shuts down the WaypointV2Module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the WaypointV2Module.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the WaypointV2Module
   * @return true/false
   */
  bool deinit();

 private:
  /**
   * @brief Upload waypoint v2 mission
   * @param request WaypointV2UploadMission service request containing mission settings
   * @param response WaypointV2UploadMission service response
   */
  void waypoint_v2_upload_mission_cb(
      const std::shared_ptr<WaypointV2UploadMission::Request> request,
      const std::shared_ptr<WaypointV2UploadMission::Response> response);

  /**
   * @brief Start waypoint v2 mission
   * @param request WaypointV2Start service request
   * @param response WaypointV2Start service response
   */
  void waypoint_v2_start_cb(
      const std::shared_ptr<WaypointV2Start::Request> request,
      const std::shared_ptr<WaypointV2Start::Response> response);

  /**
   * @brief Stop waypoint v2 mission
   * @param request WaypointV2Stop service request
   * @param response WaypointV2Stop service response
   */
  void waypoint_v2_stop_cb(
      const std::shared_ptr<WaypointV2Stop::Request> request,
      const std::shared_ptr<WaypointV2Stop::Response> response);

  /**
   * @brief Pause waypoint v2 mission
   * @param request WaypointV2Pause service request
   * @param response WaypointV2Pause service response
   */
  void waypoint_v2_pause_cb(
      const std::shared_ptr<WaypointV2Pause::Request> request,
      const std::shared_ptr<WaypointV2Pause::Response> response);

  /**
   * @brief Resume waypoint v2 mission
   * @param request WaypointV2Resume service request
   * @param response WaypointV2Resume service response
   */
  void waypoint_v2_resume_cb(
      const std::shared_ptr<WaypointV2Resume::Request> request,
      const std::shared_ptr<WaypointV2Resume::Response> response);

  /**
   * @brief Get global cruise speed
   * @param request WaypointV2GetGlobalCruiseSpeed service request
   * @param response WaypointV2GetGlobalCruiseSpeed service response
   */
  void waypoint_v2_get_global_cruise_speed_cb(
      const std::shared_ptr<WaypointV2GetGlobalCruiseSpeed::Request> request,
      const std::shared_ptr<WaypointV2GetGlobalCruiseSpeed::Response> response);

  /**
   * @brief Set global cruise speed
   * @param request WaypointV2SetGlobalCruiseSpeed service request
   * @param response WaypointV2SetGlobalCruiseSpeed service response
   */
  void waypoint_v2_set_global_cruise_speed_cb(
      const std::shared_ptr<WaypointV2SetGlobalCruiseSpeed::Request> request,
      const std::shared_ptr<WaypointV2SetGlobalCruiseSpeed::Response> response);

  /**
   * @brief Static callback for waypoint v2 mission events
   * @param eventData Mission event data from DJI PSDK
   * @return DJI return code
   */
  static T_DjiReturnCode waypoint_v2_mission_event_callback(T_DjiWaypointV2MissionEventPush eventData);

  /**
   * @brief Static callback for waypoint v2 mission state
   * @param stateData Mission state data from DJI PSDK  
   * @return DJI return code
   */
  static T_DjiReturnCode waypoint_v2_mission_state_callback(T_DjiWaypointV2MissionStatePush stateData);

  // ROS2 Services
  rclcpp::Service<WaypointV2UploadMission>::SharedPtr waypoint_v2_upload_mission_service_;
  rclcpp::Service<WaypointV2Start>::SharedPtr waypoint_v2_start_service_;
  rclcpp::Service<WaypointV2Stop>::SharedPtr waypoint_v2_stop_service_;
  rclcpp::Service<WaypointV2Pause>::SharedPtr waypoint_v2_pause_service_;
  rclcpp::Service<WaypointV2Resume>::SharedPtr waypoint_v2_resume_service_;
  rclcpp::Service<WaypointV2GetGlobalCruiseSpeed>::SharedPtr waypoint_v2_get_global_cruise_speed_service_;
  rclcpp::Service<WaypointV2SetGlobalCruiseSpeed>::SharedPtr waypoint_v2_set_global_cruise_speed_service_;

  // ROS2 Publishers
  rclcpp::Publisher<psdk_interfaces::msg::WaypointV2MissionEvent>::SharedPtr waypoint_v2_mission_event_pub_;
  rclcpp::Publisher<psdk_interfaces::msg::WaypointV2MissionState>::SharedPtr waypoint_v2_mission_state_pub_;

  const rmw_qos_profile_t& qos_profile_{rmw_qos_profile_services_default};

  bool is_module_initialized_{false};
  static WaypointV2Module* instance_; // For static callback access
};

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WAYPOINT_V2_HPP_