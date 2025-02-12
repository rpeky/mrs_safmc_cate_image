#ifndef OFFBOARDCONTROL_H
#define OFFBOARDCONTROL_H

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>
#include <iostream>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node{
	public:
		OffboardControl();
		void arm();
		void disarm();


	private:
		struct Setpoint{
			float forward;
			float right;
			float up;
			float yaw;
		};

		//to modify after finalising states
		enum class SubState{
			TAKEOFF,
			LOCALISING_TO_MARKER,
			SEARCHING_FORWARD,
			SEARCHING_LEFT,
			SEARCHING_RIGHT,
			KILL_SWITCH
		};

		//drone states
		Setpoint drone{};
		Setpoint aruco_correction{};

		SubState current_state = SubState::TAKEOFF;

		//PX4 states
		bool is_offboard_active = false;
		bool is_offboard_already_active = false;
		bool first_aruco_detected = false;
		bool landing_command_received = false;
		bool landing_command_received = false;
		bool never_go_back = false;

		int landing_command_received_time = 0;
		int last_seen = 50000;

		uint64_t flight_timer_ = 0;

		std::shared_prt<VehicleOdometry> latest_odometry_data_;

		// ROS2 communication
		
		// Publishers
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

		// Subscribers
		rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_subscriber_;
		rclcpp::Subscription<safmc_msgs::msg::TwistID>::SharedPtr bottom_camera_subscriber_;
		rclcpp::Subscription<safmc_msgs::msg::TwistID>::SharedPtr front_camera_subscriber_;

		// odometry
		rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

		//timer
		rclcpp::TimerBase::SharedPtr timer_;


		// fn callbacks
		void vehicle_control_mode_callback(const VehicleControlMode & msg);
		void keyboard_callback(const geometry_msgs::msg::Twist & msg);
		void vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg);
		void bottom_camera_callback(const safmc_msgs::msg::TwistID msg);
		void front_camera_callback(const safmc_msgs::msg::TwistID msg);

		//PX4 publishers
		void publish_offboard_control_mode();
		void publish_trajectory_setpoint(float forward, float right, float up, float yaw);
		void publish_vehicle_command(uint16_t command, float param1, float param2);
		void publish_land_command();
		void publish_set_home_command();


		// timer
		void timer_callback();


#endif //OFFBOARDCONTROL_H
