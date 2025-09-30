#pragma once

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>

#include "input.h"
// #include "ThrustCurve.h"
#include "controller.h"

namespace px4ctrl {

/**
 * @brief Configuration structure for automatic takeoff and landing operations
 * 
 * Contains timing parameters and state information for managing autonomous
 * takeoff and landing sequences.
 */
struct AutoTakeoffLandConfig
{
	bool landed{true};
	ros::Time toggle_start_time;
	std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTOR_SPINUP_DURATION_SEC = 3.0; ///< Motors idle running duration before takeoff
	static constexpr double DELAY_TRIGGER_DURATION_SEC = 2.0;  ///< Time delay when reaching target height
};

/**
 * @brief Finite State Machine controller for PX4-based multirotor aircraft
 * 
 * This class implements a comprehensive flight control system that manages
 * different flight modes including manual control, autonomous hover, command
 * following, automatic takeoff, and automatic landing.
 * 
 * The FSM ensures safe transitions between states and provides multiple
 * layers of safety checks and emergency procedures.
 * 
 * @note This class is designed to be used with PX4 flight controller
 *       via MAVROS interface.
 */
class PX4CtrlFSM
{
public:
	Parameter_t &param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;

	LinearControl &controller;

	ros::Publisher traj_start_trigger_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub; //debug
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;

	quadrotor_msgs::Px4ctrlDebug debug_msg; //debug

	Eigen::Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;

	enum class FlightState : uint8_t
	{
		ManualControl = 1, // px4ctrl is deactivated. FCU is controlled by the remote controller only
		AutoHover, // px4ctrl is activated, it will keep the drone hover from odom measurements while waiting for commands from PositionCommand topic.
		CommandControl,	// px4ctrl is activated, and controlling the drone.
		AutoTakeoff,
		AutoLand
	};

	/**
	 * @brief Constructs the PX4 control finite state machine
	 * @param param Reference to system parameters
	 * @param controller Reference to the linear controller instance
	 */
	PX4CtrlFSM(Parameter_t &param, LinearControl &controller);
	
	/**
	 * @brief Main processing function - should be called at regular intervals
	 * 
	 * Executes the finite state machine logic, processes sensor data,
	 * and generates control outputs based on current flight state.
	 */
	void process();
	
	/**
	 * @brief Check if RC (Remote Controller) data has been recently received
	 * @param now_time Current ROS time for timeout checking
	 * @return true if RC data is current and valid
	 */
	bool isRCReceived(const ros::Time &now_time);
	
	/**
	 * @brief Check if position command data has been recently received
	 * @param now_time Current ROS time for timeout checking
	 * @return true if command data is current and valid
	 */
	bool isCommandReceived(const ros::Time &now_time);
	
	/**
	 * @brief Check if odometry data has been recently received
	 * @param now_time Current ROS time for timeout checking
	 * @return true if odometry data is current and valid
	 */
	bool isOdometryReceived(const ros::Time &now_time);
	
	/**
	 * @brief Check if IMU data has been recently received
	 * @param now_time Current ROS time for timeout checking
	 * @return true if IMU data is current and valid
	 */
	bool isIMUReceived(const ros::Time &now_time);
	
	/**
	 * @brief Check if battery data has been recently received
	 * @param now_time Current ROS time for timeout checking
	 * @return true if battery data is current and valid
	 */
	bool isBatteryReceived(const ros::Time &now_time);
	
	/**
	 * @brief Check if new odometry data is available for processing
	 * @return true if new odometry data has been received since last check
	 */
	bool hasNewOdometry();
	
	/**
	 * @brief Get current flight state
	 * @return Current flight state enum value
	 */
	FlightState getState() const { return state_; }
	
	/**
	 * @brief Check if the aircraft is currently landed
	 * @return true if aircraft is detected as landed
	 */
	bool getLanded() const { return takeoff_land_.landed; }

private:
	FlightState state_; // Should only be changed in PX4CtrlFSM::process() function!
	AutoTakeoffLandConfig takeoff_land_;

	// ---- control related ----
	Desired_State_t getHoverDesiredState();
	Desired_State_t getCommandDesiredState();

	// ---- auto takeoff/land ----
	void setMotorsIdling(const Imu_Data_t &imu, Controller_Output_t &u);
	void detectLanding(const FlightState state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	void setStartPoseForTakeoffLand(const Odom_Data_t &odom);
	Desired_State_t getRotorSpeedUpDesiredState(const ros::Time now);
	Desired_State_t getTakeoffLandDesiredState(const double speed);

	// ---- tools ----
	void setHoverWithOdometry();
	void setHoverWithRC();

	bool toggleOffboardMode(bool enable); // It will only try to toggle once, so not blocked.
	bool toggleArmDisarm(bool arm); // It will only try to toggle once, so not blocked.
	void rebootFlightController();

	void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);
};

}  // namespace px4ctrl