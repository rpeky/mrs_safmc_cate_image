#include "offboard_controller.h"


class OffboardControl : public rclcpp::Node{
	public:
		OffboardControl() : Node("offboard_control"){
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
			} 
			else {
			drone.up = new_height;
			}

			// degree to radian
			float new_yaw = drone.yaw - msg.angular.z * 10 * 0.0174533;

			if (new_yaw > M_PI) {
				drone.yaw = new_yaw - (2 * M_PI);
			} 
			else if (new_yaw < M_PI) {
				drone.yaw = new_yaw + (2 * M_PI);
			} 
			else {
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
void OffboardControl::arm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode(){
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
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
void OffboardControl::publish_trajectory_setpoint(){
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2){
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

int main(int argc, char *argv[]){
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
