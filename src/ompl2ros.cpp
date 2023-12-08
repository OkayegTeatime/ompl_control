#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "controlPlanning.h"
#include "PostProcessing.h"

class OMPLtoROS : public rclcpp::Node {
public:
    OMPLtoROS() : Node("ompl_to_ros") {
        ompl_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&OMPLtoROS::ompl_callback, this, std::placeholders::_1)
        );
        ackermann_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/car/cmd_ackermann", 10);

        plan_pub = this->create_publisher<nav_msgs::msg::Path>(
            "/plan", 10);

        rclcpp::QoS custom_qos(rclcpp::KeepLast(10)); // Example settings, adjust as needed
        custom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/car/odom", custom_qos, std::bind(&OMPLtoROS::odom_callback, this, std::placeholders::_1));
    }

    bool planControl(double goalX, double goalY) {
        //create world from YAML file
        World *w = new World();
        // World *w = yaml2world("ProblemCntrl.yml");
        RCLCPP_INFO(this->get_logger(), "Initial Pose: %f, %f, %f", current_pose[0], current_pose[1], current_pose[2]);
        
        std::string name = "agent0";
        const std::vector<double> shape{0.3429, 0.25};
        // const std::vector<double> start{current_pose[0], current_pose[1]};
        const std::vector<double> goal{goalX, goalY};
        Agent *a = new Agent(name, "Car", shape, current_pose, goal);
        w->addAgent(a);
        w->setWorldDimensions(15.0, 15.0);
        w->printWorld();
        
        // create simple setup object
        oc::SimpleSetupPtr ss = controlSimpleSetUp(w);

        // set planner and setup
        // ob::PlannerPtr planner = std::make_shared<oc::SST>(ss->getSpaceInformation());
        ob::PlannerPtr planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        ss->getSpaceInformation()->setPropagationStepSize(0.1);
        ss->setPlanner(planner);
        ss->setup();
        
        // solve the instance
        bool solved = ss->solve(30.0);
        if (solved)
            write2sys(ss, w->getAgents());
        std::cout << ss->getSpaceInformation()->getPropagationStepSize() << " Step Size";
        return solved;
    }   
    

private:
    void ompl_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Goal Pose: %f, %f", msg->pose.position.x, msg->pose.position.y);
        ackermann_msgs::msg::AckermannDriveStamped ad;
        bool solved = planControl(msg->pose.position.x, msg->pose.position.y);
        if (solved) {
            RCLCPP_INFO(this->get_logger(), "Sending control commands to Ackermann");
            std::vector<std::vector<double>> matrix = getStates();
            publish_path(matrix);
            for (const auto& row : matrix) {
                ad.drive.speed = row[2];
                ad.drive.steering_angle = row[3];
                ackermann_pub->publish(ad);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            RCLCPP_INFO(this->get_logger(), "All commands sent to Ackermann");

        }
        // Publish the controls
        // ad.drive.speed = 0.25;
        // ad.drive.steering_angle = 0.1;
        // ad.drive.steering_angle_velocity = 0.1;
        // ad.drive.acceleration = 0.5;
        // ackermann_pub->publish(ad);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double yaw = 2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        current_pose = {msg->pose.pose.position.x, msg->pose.pose.position.y, wrapToPi(yaw)};
        // RCLCPP_INFO(this->get_logger(), "Current pose set");

    }

    void publish_path(const std::vector<std::vector<double>>& matrix) {
        auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.frame_id = "odom"; // Set the frame ID according to your use case

        // Creating waypoints as geometry_msgs/PoseStamped and adding to the Path message
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        geometry_msgs::msg::PoseStamped pose_stamped;
        for (const auto& row : matrix) {
            pose_stamped.pose.position.x = row[0];
            pose_stamped.pose.position.y = row[1];
            pose_stamped.pose.orientation.z = std::cos(row[4] / 2.0);
            pose_stamped.pose.orientation.w = std::sin(row[4] / 2.0);
            waypoints.push_back(pose_stamped);
        }
        path_msg->poses = waypoints;
        path_msg->header.stamp = this->now();
        plan_pub->publish(*path_msg);
    }

    std::vector<double> current_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ompl_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto nav2_handler = std::make_shared<OMPLtoROS>();
    rclcpp::spin(nav2_handler);
    rclcpp::shutdown();
    return 0;
}

// rostopic pub /set_goal geometry_msgs/Point "{x: 10.0, y: 20.0, z: 0.0}"