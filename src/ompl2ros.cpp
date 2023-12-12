#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "controlPlanning.h"
#include "PostProcessing.h"

class OMPLtoROS : public rclcpp::Node {
public:
    OMPLtoROS() : Node("ompl_to_ros") {
        // ompl_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/goal_pose", 10,
        //     std::bind(&OMPLtoROS::ompl_callback, this, std::placeholders::_1)
        // );
        ompl_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&OMPLtoROS::ompl_callback, this, std::placeholders::_1)
        );

        ackermann_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/car/cmd_ackermann", 10);

        plan_pub = this->create_publisher<nav_msgs::msg::Path>(
            "/plan", 10);

        poly_pub = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles", 10);
        poly_pub1 = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles1", 10);
        poly_pub2 = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles2", 10);
        poly_pub3 = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles3", 10);
        poly_pub4 = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles4", 10);
        poly_pub5 = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles5", 10);
        poly_pub6 = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles6", 10);
        poly_pub7 = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/obstacles7", 10);


        waypoints_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        

        rclcpp::QoS custom_qos(rclcpp::KeepLast(10)); // Example settings, adjust as needed
        custom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/car/odom", custom_qos, std::bind(&OMPLtoROS::odom_callback, this, std::placeholders::_1));

        image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_uav/image_raw", 10,
            std::bind(&OMPLtoROS::image_callback, this, std::placeholders::_1)
        );
            
        const std::vector<double> shape{0.3429, 0.25};
        const std::vector<double> goal{0.0, 0.0};
        Agent *a = new Agent("agent0", "Car", shape, current_pose, goal);
        w->addAgent(a);
        w->setWorldDimensions({-10, 10, -10, 10});
    }

    bool planControl(double goalX, double goalY) {
        //create world from YAML file
        // World *w = yaml2world("ProblemCntrl.yml");
        RCLCPP_INFO(this->get_logger(), "Initial Pose: %f, %f, %f", current_pose[0], current_pose[1], current_pose[2]);
        w->getAgents()[0]->setStartLocation(current_pose);
        w->getAgents()[0]->setGoalLocation({goalX, goalY});
        w->printWorld();
        
        // create simple setup object
        oc::SimpleSetupPtr ss = controlSimpleSetUp(w);

        // set planner and setup
        ob::PlannerPtr planner = std::make_shared<oc::SST>(ss->getSpaceInformation());
        // ob::PlannerPtr planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        ss->getSpaceInformation()->setPropagationStepSize(timeStep);
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
    void ompl_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Goal Pose: %f, %f", msg->point.x, msg->point.y);
        ackermann_msgs::msg::AckermannDriveStamped ad;
        bool solved = planControl(msg->point.x, msg->point.y);
        if (solved) {
            RCLCPP_INFO(this->get_logger(), "Sending control commands to Ackermann");
            std::vector<std::vector<double>> matrix = getStates();
            auto waypoint = std::make_unique<geometry_msgs::msg::PoseStamped>();
            publish_path(matrix);
            for (const auto& row : matrix) {
                ad.drive.speed = row[2];
                ad.drive.steering_angle = row[3];
                ackermann_pub->publish(ad);
                rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(timeStep*1000)));
            }
            RCLCPP_INFO(this->get_logger(), "All commands sent to Ackermann");
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double yaw = 2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        current_pose = {msg->pose.pose.position.x, msg->pose.pose.position.y, wrapToPi(yaw)};
        // RCLCPP_INFO(this->get_logger(), "Current pose set");
        // std::vector<std::vector<double>> matrix = getStates();
        // auto waypoint = std::make_unique<geometry_msgs::msg::PoseStamped>();
        // waypoint->header.frame_id = "map";
        // for (const auto& row : matrix) {
        //     waypoint->header.stamp = this->now();
        //     waypoint->pose.position.x = row[0];
        //     waypoint->pose.position.y = row[1];
        //     waypoint->pose.orientation.z = std::cos(row[4] / 2.0);
        //     waypoint->pose.orientation.w = std::sin(row[4] / 2.0);
        //     waypoints_publisher_->publish(*waypoint);
        // }
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
            pose_stamped.header.frame_id = "odom";
            // pose_stamped.header.stamp = clock_->now();
            waypoints.push_back(pose_stamped);
        }
        path_msg->poses = waypoints;
        path_msg->header.stamp = this->now();
        plan_pub->publish(*path_msg);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (warming > 2) {
            if (!cv_ptr) { 
                RCLCPP_INFO(this->get_logger(), "SERVER PROCESSING IMAGE");
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                cv::Mat image = cv_ptr->image;
                applyMask(image);
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                // Iterate through contours to find vertices
                cv::Mat polygonImg = cv::Mat::zeros(image.size(), CV_8UC3);
                w->clearObstacles();
                int p = 0;
                for (size_t i = 0; i < contours.size(); ++i) {
                    double area = cv::contourArea(contours[i]);
                    if (area > 1000){
                        std::vector<geometry_msgs::msg::Point32> polygonVertices;
                        geometry_msgs::msg::Point32 vertex;
                        std::string points = "POLYGON((";
                        std::vector<cv::Point> contour = contours[i];
                        std::vector<cv::Point> vertices;
                        cv::approxPolyDP(contour, vertices, 5.0, true);
                        vertices.push_back(vertices[0]);

                        // Draw the contour and its vertices
                        cv::drawContours(polygonImg, std::vector<std::vector<cv::Point>>{vertices}, -1, cv::Scalar(175), 2);
                        for (int j = 0; j < vertices.size(); ++j) {
                            cv::circle(polygonImg, vertices[j], 5, cv::Scalar(0, 0, 255), -1);
                            std::pair<double, double> coord = pixelToCoord(vertices[j]);
                            vertex.x = coord.first;
                            vertex.y = coord.second;
                            vertex.z = 0.0;
                            polygonVertices.push_back(vertex);
                            points += std::to_string(coord.first) + " " + std::to_string(coord.second);
                            if (j == vertices.size() - 1) points += "))";
                            else points += ",";
                        }
                        auto polygonMsg = std::make_unique<geometry_msgs::msg::PolygonStamped>();
                        polygonMsg->header.frame_id = "odom";
                        polygonMsg->header.stamp = this->now();
                        polygonMsg->polygon.points = polygonVertices;
                        if (p == 0) poly_pub->publish(*polygonMsg);
                        if (p == 1) poly_pub1->publish(*polygonMsg);
                        if (p == 2) poly_pub2->publish(*polygonMsg);
                        if (p == 3) poly_pub3->publish(*polygonMsg);
                        if (p == 4) poly_pub4->publish(*polygonMsg);
                        if (p == 5) poly_pub5->publish(*polygonMsg);
                        if (p == 6) poly_pub6->publish(*polygonMsg);
                        if (p == 7) poly_pub7->publish(*polygonMsg);
                        polygon poly;
                        boost::geometry::read_wkt(points, poly);
                        w->addPolygon(poly);
                        p++;
                    }
                }
                cv::imshow("Detected Vertices", polygonImg);
                cv::waitKey(0);
                
                // cv::imwrite("bing.png", cv_ptr->image);
                // cv::imwrite("bong.png", polygonImg);
            } 
        }
        warming++;
    }

    void applyMask(cv::Mat& image) {
        cv::Scalar lowerColor = cv::Scalar(154, 154, 154);  // Lower bound for (155, 155, 155)
        cv::Scalar upperColor = cv::Scalar(156, 156, 156); 
        cv::inRange(image, lowerColor, upperColor, image);
        cv::bitwise_not(image, image);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // Kernel size = 3x3
        cv::dilate(image, image, element);
    }

    std::pair<double, double> pixelToCoord(const cv::Point& pixel) {
        // Given parameters
        float fov_horizontal_rad = 1.25f;
        float fov_vertical_rad = 1.25f;
        int image_width = 1000;  
        int image_height = 1000;
        float captureHeight = 10.0;
        float focal_length_horizontal = image_width / (2 * tan(fov_horizontal_rad / 2));
        float focal_length_vertical = image_height / (2 * tan(fov_vertical_rad / 2));
        float x = ((pixel.x - (image_width / 2)) * captureHeight) / focal_length_horizontal;
        float y = ((pixel.y - (image_height / 2)) * captureHeight) / focal_length_vertical;
        std::pair<double, double> coordinate = {x, -y};
        std::cout << "Coordinate: (" << coordinate.first << ", " << coordinate.second << ")" << std::endl;
        return coordinate;
    }

    std::vector<double> current_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ompl_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoints_publisher_;
    rclcpp::Clock::SharedPtr clock_;
    cv_bridge::CvImagePtr cv_ptr;
    World *w = new World();
    double timeStep = 0.1;
    int warming = 0;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub1;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub2;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub3;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub4;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub5;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub6;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub7;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto nav2_handler = std::make_shared<OMPLtoROS>();
    rclcpp::spin(nav2_handler);
    rclcpp::shutdown();
    return 0;
}

// rostopic pub /set_goal geometry_msgs/Point "{x: 10.0, y: 20.0, z: 0.0}"