#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "arminator/al5d_controller.hpp"
#include <thread>
#include <chrono>

class ArminatorNode : public rclcpp::Node {
public:
    ArminatorNode() : Node("arminator_node") {
        // Parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 9600);
        this->declare_parameter<double>("publish_rate", 10.0);
        
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        // Initialize AL5D controller
        controller_ = std::make_unique<arminator::AL5DController>(serial_port, baud_rate);
        
        // Publishers
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);
        cartesian_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "cartesian_pose", 10);
        
        // Subscribers
        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_commands", 10,
            std::bind(&ArminatorNode::jointCommandCallback, this, std::placeholders::_1));
        
        cartesian_command_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "cartesian_commands", 10,
            std::bind(&ArminatorNode::cartesianCommandCallback, this, std::placeholders::_1));
        
        // Services
        home_service_ = this->create_service<std_srvs::srv::Empty>(
            "home_robot", 
            std::bind(&ArminatorNode::homeRobotService, this, std::placeholders::_1, std::placeholders::_2));
        
        stop_service_ = this->create_service<std_srvs::srv::Empty>(
            "stop_motion",
            std::bind(&ArminatorNode::stopMotionService, this, std::placeholders::_1, std::placeholders::_2));
        
        torque_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_torque_enable",
            std::bind(&ArminatorNode::setTorqueService, this, std::placeholders::_1, std::placeholders::_2));
        
        open_gripper_service_ = this->create_service<std_srvs::srv::Empty>(
            "open_gripper",
            std::bind(&ArminatorNode::openGripperService, this, std::placeholders::_1, std::placeholders::_2));
        
        close_gripper_service_ = this->create_service<std_srvs::srv::Empty>(
            "close_gripper",
            std::bind(&ArminatorNode::closeGripperService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timer for publishing state
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(timer_period, 
            std::bind(&ArminatorNode::publishState, this));
        
        // Initialize controller
        if (!controller_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize AL5D controller");
            return;
        }
        
        // Set joint state callback
        controller_->setJointStateCallback(
            std::bind(&ArminatorNode::jointStateCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Arminator node initialized successfully");
    }
    
    ~ArminatorNode() {
        if (controller_) {
            controller_->shutdown();
        }
    }

private:
    void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 5) {
            RCLCPP_WARN(this->get_logger(), "Joint command must have at least 5 positions");
            return;
        }
        
        std::vector<double> positions(msg->position.begin(), msg->position.end());
        
        // Convert from radians to degrees if needed
        for (auto& pos : positions) {
            pos = pos * 180.0 / M_PI;
        }
        
        if (!controller_->moveToJointPositions(positions)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute joint command");
        }
    }
    
    void cartesianCommandCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        arminator::CartesianPose pose;
        pose.x = msg->position.x * 1000.0; // Convert m to mm
        pose.y = msg->position.y * 1000.0;
        pose.z = msg->position.z * 1000.0;
        
        // Convert quaternion to Euler angles (simplified)
        // In practice, you'd use a proper quaternion to Euler conversion
        pose.roll = 0.0;
        pose.pitch = 0.0;
        pose.yaw = 0.0;
        
        if (!controller_->moveToCartesianPose(pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian command");
        }
    }
    
    void jointStateCallback(const arminator::JointState& state) {
        // This callback is called when joint state changes
        last_joint_state_ = state;
    }
    
    void publishState() {
        if (!controller_->isConnected()) {
            return;
        }
        
        // Publish joint state
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.header.frame_id = "base_link";
        
        joint_state_msg.name = {"base_joint", "shoulder_joint", "elbow_joint", 
                               "wrist_joint", "wrist_rotate_joint", "gripper_joint"};
        
        auto current_state = controller_->getCurrentJointState();
        
        // Convert degrees to radians for ROS
        joint_state_msg.position.clear();
        joint_state_msg.velocity.clear();
        joint_state_msg.effort.clear();
        
        for (size_t i = 0; i < current_state.positions.size(); ++i) {
            joint_state_msg.position.push_back(current_state.positions[i] * M_PI / 180.0);
            joint_state_msg.velocity.push_back(current_state.velocities[i] * M_PI / 180.0);
            joint_state_msg.effort.push_back(current_state.efforts[i]);
        }
        
        joint_state_pub_->publish(joint_state_msg);
        
        // Publish Cartesian pose
        auto cartesian_pose = controller_->getCurrentCartesianPose();
        auto pose_msg = geometry_msgs::msg::Pose();
        
        pose_msg.position.x = cartesian_pose.x / 1000.0; // Convert mm to m
        pose_msg.position.y = cartesian_pose.y / 1000.0;
        pose_msg.position.z = cartesian_pose.z / 1000.0;
        
        // Convert Euler angles to quaternion (simplified)
        pose_msg.orientation.w = 1.0;
        pose_msg.orientation.x = 0.0;
        pose_msg.orientation.y = 0.0;
        pose_msg.orientation.z = 0.0;
        
        cartesian_pose_pub_->publish(pose_msg);
    }
    
    void homeRobotService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>) {
        bool success = controller_->homeRobot();
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Robot homed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to home robot");
        }
    }
    
    void stopMotionService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                          std::shared_ptr<std_srvs::srv::Empty::Response>) {
        controller_->stopMotion();
        RCLCPP_INFO(this->get_logger(), "Motion stopped");
    }
    
    void setTorqueService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        controller_->setTorqueEnable(request->data);
        response->success = true;
        response->message = request->data ? "Torque enabled" : "Torque disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    void openGripperService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                           std::shared_ptr<std_srvs::srv::Empty::Response>) {
        controller_->openGripper();
        RCLCPP_INFO(this->get_logger(), "Gripper opened");
    }
    
    void closeGripperService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>) {
        controller_->closeGripper();
        RCLCPP_INFO(this->get_logger(), "Gripper closed");
    }
    
    std::unique_ptr<arminator::AL5DController> controller_;
    arminator::JointState last_joint_state_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cartesian_pose_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cartesian_command_sub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr home_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr open_gripper_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr close_gripper_service_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ArminatorNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}