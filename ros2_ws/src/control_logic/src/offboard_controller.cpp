/***************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#include <memory>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
//#include "safmc_msgs/msg/twist_id.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// Publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// Subscribers
		vehicle_control_mode_subscriber_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, std::bind(&OffboardControl::vehicle_control_mode_callback, this, _1));
		keyboard_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&OffboardControl::keyboard_callback, this, _1));
		vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&OffboardControl::vehicle_odometry_callback, this, _1));
		front_camera_subscriber_ = this->create_subscription<safmc_msgs::msg::TwistID>("/image_converter/cam1/coordinates", qos, std::bind(&OffboardControl::front_camera_callback, this, _1));
		bottom_camera_subscriber_ = this->create_subscription<safmc_msgs::msg::TwistID>("/image_converter/cam0/coordinates", qos, std::bind(&OffboardControl::bottom_camera_callback, this, _1));
		
		flight_timer_ = 0;
		is_offboard_active = false;
		is_offboard_already_active = false;

		drone.forward = 0.0;
		drone.right = 0.0;
		drone.up = 0.0;
		drone.yaw = 0.0;

		auto timer_callback = [this]() -> void {

			// Arm vehicle if offboard mode has been activated from RC
			if (is_offboard_active && is_offboard_already_active == false) {
				RCLCPP_INFO(this->get_logger(), "Offboard Mode Activated");
				RCLCPP_INFO(this->get_logger(), "Arming Vehicle");
				// Arm the vehicle
				this->arm();
				is_offboard_already_active = true; // this ensures vehicle is only armed once.
			}

			if (!is_offboard_already_active && flight_timer_ % 100 == 0) {
				RCLCPP_INFO(this->get_logger(), "Waiting for Offboard Mode to Activate");
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			// publish_offboard_control_mode() has to be called a few times before PX4 lets us arm
			publish_offboard_control_mode();

			if (!is_offboard_active) return;

			/*==========================================
			=            Counter increments            =
			==========================================*/
			flight_timer_++; // increment counter
			// TODO: rest of counters				
			
			// Logging
			if (flight_timer_ % 100 == 0) {
				RCLCPP_INFO(this->get_logger(), "drone.forward: %.2f", drone.forward);
				RCLCPP_INFO(this->get_logger(), "drone.right: %.2f", drone.right);
				RCLCPP_INFO(this->get_logger(), "drone.up: %.2f", drone.up);
				RCLCPP_INFO(this->get_logger(), "drone.yaw: %.2f", drone.yaw);
			}

			// Give 5 seconds for takeoff
			if (flight_timer_ < 500) {
				RCLCPP_INFO(this->get_logger(), "Taking Off");
				publish_trajectory_setpoint(0.0, 0.0, 1.0, 0.0);
				drone.up = 1.0;
				return;
			}

			// If our setpoint altitude is negative, we're trying to land
			// Send command as such
			if (drone.up < 0.0) {
				if(landing_command_received && (flight_timer_ - landing_command_received_time > 500)){ //Counts until 5s have passed since it was told to land, then disarm
					this->disarm();
				}
				return;
			}

			/*=============================================================
			=            Modify setpoint based on state machine           =
			=============================================================*/

			// TODO: implement a task queue:
			// a std::Vector containing the queue of higher-level tasks to perform
			// e.g. linear search, zigzag search, etc.
			// The current task will enter a particular sub-statemachine (e.g. SEARCHING_FORWARD)
			// currently temporarily initialized to SEARCHING_FORWARD

			switch (current_state) {
				case SubState::LOCALISING_TO_MARKER:
					drone.forward = aruco_correction.forward;
					drone.right = aruco_correction.right;
					drone.up = aruco_correction.up;

					if(land_on_localise) {
						if(landing_countdown_timer<1 && !landing_command_received){//if stable localisation, set drone.up into the floor and get ready to disarm
							if(last_seen == 1){
							drone.up = -1.0;
							landing_command_received_time = flight_timer_;
							landing_command_received = true;//this block won't run again
							}
							if(last_seen == 0){
								current_state = SubState::SEARCHING_FORWARD;
							}
						}
					}
					break;
				case SubState::SEARCHING_FORWARD:
					drone.forward = 0.5*cos(aruco_correction.yaw);
					drone.right = 0.5*sin(aruco_correction.yaw);
					never_go_back = true;
					break;
				case SubState::SEARCHING_LEFT:
				case SubState::SEARCHING_RIGHT:
					break;
			}

			// Publish new setpoint
			publish_trajectory_setpoint(drone.forward, drone.right, drone.up, drone.yaw);


		};
		timer_ = this->create_wall_timer(10ms, timer_callback);
	}

	void arm();
	void disarm();

private:

	struct Setpoint {
		float forward;
		float right;
		float up;
		float yaw;
	};

	Setpoint drone;
	Setpoint aruco_correction;

	enum SubState
	{
		LOCALISING_TO_MARKER,
		SEARCHING_FORWARD,
		SEARCHING_LEFT,
		SEARCHING_RIGHT
		// TODO: ...? is this suitable? dunno, this can be changed if
		// separating left/right/forward becomes redundant
	};

	SubState current_state = SubState::SEARCHING_FORWARD;

	// Check if offboard mode is turned on
	void vehicle_control_mode_callback(const VehicleControlMode & msg) {
		//RCLCPP_INFO(this->get_logger(), "%d", msg.flag_control_offboard_enabled);
		is_offboard_active = msg.flag_control_offboard_enabled;
	}

	void keyboard_callback(const geometry_msgs::msg::Twist & msg) {
		drone.forward += msg.linear.x;
		drone.right -= msg.linear.y;
		
		float new_height = drone.up + msg.linear.z;

		// Limit height to 1m
		if (new_height > 1.0) {
			drone.up = 1.0;
		} else {
			drone.up = new_height;
		}

		// degree to radian
		float new_yaw = drone.yaw - msg.angular.z * 10 * 0.0174533;

		if (new_yaw > M_PI) {
			drone.yaw = new_yaw - (2 * M_PI);
		} else if (new_yaw < M_PI) {
			drone.yaw = new_yaw + (2 * M_PI);
		} else {
			drone.yaw = new_yaw;
		}
	}
	void vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg) {
	//void vehicle_odometry_callback(const VehicleOdometry msg) {
		//store latest odometry data
		latest_odometry_data_ = msg;
		
		//std::cout << "x: " << latest_odometry_data_->position[0] << std::endl; 
		//std::cout << "y: " << latest_odometry_data_->position[1] << std::endl; 
		//std::cout << "z: " << latest_odometry_data_->position[2] << std::endl; 
		//std::cout << "qx: " << latest_odometry_data_->q[0] << std::endl; 
		//std::cout << "qy: " << latest_odometry_data_->q[0] << std::endl; 
		//std::cout << "qz: " << latest_odometry_data_->q[0] << std::endl; 
	}

	void bottom_camera_callback(const safmc_msgs::msg::TwistID msg) {
		RCLCPP_INFO(this->get_logger(), "[bottom_camera_callback] called");

		if (latest_odometry_data_ == nullptr) {
			return;
		}
		int last_seen = msg.id;
		if(msg.id == 0 && !never_go_back){
		aruco_correction.right = -msg.twist.linear.x * 0.5;
		aruco_correction.forward = msg.twist.linear.y * 0.5;
		aruco_correction.yaw = msg.twist.angular.x;
		}
		
		// TODO: implement timeout to allow vehicle to move away from marker
		// TODO: reset localisation-lost timer
		// TODO: increment localisation-found timer
		
		if(msg.id == 1){
			current_state = SubState::LOCALISING_TO_MARKER;
			aruco_correction.right = -msg.twist.linear.x * 0.5;
			aruco_correction.forward = msg.twist.linear.y * 0.5;
			aruco_correction.yaw = msg.twist.angular.x;

			first_aruco_detected = true;

			if(land_on_localise){
				// Check if correction distance is small enough
				if (abs(msg.twist.linear.x)<0.1 && abs(msg.twist.linear.y)<0.1) {
					//Countdown to landing
					landing_countdown_timer -= 1;
					std::cout << "Landing countdown timer: " << landing_countdown_timer << std::endl;
				}
				else{
					//If drifts too far, reset the landing countdown
					landing_countdown_timer = landing_countdown_number;
				}
			}
			
			//std::cout << "aruco.x: " << msg.twist.linear.x << std::endl;
			//std::cout << "aruco.y: " << msg.twist.linear.y << std::endl;
			//std::cout << "aruco.z: " << msg.twist.linear.z << std::endl;
			std::cout << "aruco.p: " << msg.twist.angular.x << std::endl;
			//std::cout << "aruco.r: " << msg.twist.angular.y << std::endl;
			//std::cout << "aruco.y: " << msg.twist.angular.z << std::endl;
		}
	}

	void front_camera_callback(const safmc_msgs::msg::TwistID msg) {
		//Get vehicle yaw
		//latest_odometry_data_ ->q;
		RCLCPP_INFO(this->get_logger(), "[front_camera_callback] called");

		if (latest_odometry_data_ == nullptr) {
			return;
		}

		tf2::Quaternion q(
			latest_odometry_data_->q[0],
			latest_odometry_data_->q[1],
			latest_odometry_data_->q[2],
			latest_odometry_data_->q[3]);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		//Set height
		//drone.up = -latest_odometry_data_->position[2] + msg.twist.linear.y;
		//Set other translation
		/*
		float modified_x = msg.twist.linear.x * cos(yaw) - msg.twist.linear.z * sin(yaw);
		float modified_y = msg.twist.linear.x * sin(yaw) + msg.twist.linear.z * cos(yaw);
		float magnitude = sqrt(pow(modified_x, 2.0) + pow(modified_y, 2.0));
		float modified_magnitude = 0.1*(magnitude - 1.0)/magnitude;//CHANGE THIS
		modified_x *= modified_magnitude;
		modified_y *= modified_magnitude;
		drone.right = latest_odometry_data_->position[0] + modified_x;
		drone.forward = latest_odometry_data_->position[1] + modified_y;
		if (msg.twist.angular.y > 0){
			drone.yaw = yaw - 0.01;
		}
		else{
			drone.yaw = yaw + 0.01;
		}
		*/

		drone.right = latest_odometry_data_->position[0] - 0.01 * (msg.twist.linear.x);
		drone.forward = latest_odometry_data_->position[1] + 0.01 * (msg.twist.linear.z - 1);
		std::cout << "forward: " << drone.forward << std::endl;
		std::cout << "right: " << drone.right << std::endl;
		std::cout << "up: " << drone.up << std::endl;
		std::cout << "yaw: " << drone.yaw << std::endl;
	}

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_subscriber_;
	rclcpp::Subscription<safmc_msgs::msg::TwistID>::SharedPtr bottom_camera_subscriber_;
	rclcpp::Subscription<safmc_msgs::msg::TwistID>::SharedPtr front_camera_subscriber_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	bool is_offboard_active;
	bool is_offboard_already_active;
	uint64_t time_at_detection_;

	// TODO: timers
	// timer for localization to give up
	// timer for zigzag 
	
	/*==============================
	=            Timers            =
	==============================*/
	
	uint64_t flight_timer_;   //!< counter for the number of setpoints sent

	// TODO: semaphores
	// currently localizing?
	// what ID are we searching for?
	// currently searching?
	// are we zigzagging?
	
	//Set the drone behaviour	
	bool localise_on_detect = true;

	int landing_countdown_number = 10;
	bool land_on_localise = true;
	
	//Bools for drone behaviour code to use, don't change
	bool first_aruco_detected = false;
	int landing_countdown_timer = landing_countdown_number;
	bool landing_command_received = false;
	int landing_command_received_time = 0;
	int last_seen = 50000;
	std::shared_ptr<VehicleOdometry> latest_odometry_data_;
	bool never_go_back = false;
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float forward, float right, float up, float yaw);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_take_off_command(float height);
	void publish_set_home_command();
	void publish_land_command();
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 21196);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float forward, float right, float up, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {NAN, NAN, -up};
	msg.velocity = {forward, right, NAN};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Land at location
*/
void OffboardControl::publish_land_command()
{
	//this->publish_set_home_command();

	VehicleCommand msg{};
	msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
	msg.param4 = 0.0; // Yaw angle (if magnetometer present), ignored without magnetometer
	msg.param5 = 1.0; // Latitude
	msg.param6 = 0.0; // Logitude
	msg.param7 = 0.0; // Altitude
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Land at location
*/
void OffboardControl::publish_set_home_command()
{
	VehicleCommand msg{};
	msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_HOME;
	msg.param1 = 1; // set to current position
	msg.param2 = NAN;
	msg.param3 = NAN;
	msg.param4 = NAN;
	msg.param5 = NAN;
	msg.param6 = NAN;
	msg.param7 = 0.0;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
