#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <nav2_core/controller.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <cmath>
#include <limits>

class DWAPlanner : public nav2_core::Controller {
public:
    DWAPlanner();
    ~DWAPlanner();

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
        node_ = parent;
        auto node = node_.lock();
        if (!node) throw std::runtime_error("Failed to lock node");
        logger_ = node->get_logger();
        clock_ = node->get_clock();
        tf_buffer_ = tf;
        costmap_ros_ = costmap_ros;
        if(!costmap_ros_) throw std::runtime_error("Failed to loack the costmap");
        costmap_ = costmap_ros_->getCostmap();
        
        traj_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_trajectories", 10);
        path_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_global_plan", 10);
        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&DWAPlanner::odomCallback, this, std::placeholders::_1));
        scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DWAPlanner::scanCallback, this, std::placeholders::_1));
    }

    void cleanup() override {}
    void activate() override {}
    void deactivate() override {}

    void setPlan(const nav_msgs::msg::Path &plan) override {
        global_plan_ = plan;
    }

    double checkDistanceFromGoal(){
        if (global_plan_.poses.empty()) {
            RCLCPP_WARN(logger_, "Global plan is empty, cannot compute distance to goal.");
            return std::numeric_limits<double>::max();
        }

        geometry_msgs::msg::TransformStamped odom_to_map;
        try {
            odom_to_map = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(logger_, "Could not transform odom to map: %s", ex.what());
            return std::numeric_limits<double>::max();
        }
        geometry_msgs::msg::PoseStamped robot_pose_odom;
        robot_pose_odom.header.frame_id = "odom";
        robot_pose_odom.header.stamp = clock_->now();
        robot_pose_odom.pose.position.x = current_x_;
        robot_pose_odom.pose.position.y = current_y_;
        robot_pose_odom.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped robot_pose_map;
        tf2::doTransform(robot_pose_odom, robot_pose_map, odom_to_map);

        auto& goal_pose = global_plan_.poses.back().pose;
        double dx = goal_pose.position.x - robot_pose_map.pose.position.x;
        double dy = goal_pose.position.y - robot_pose_map.pose.position.y;

        return std::hypot(dx, dy);
    }

    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                             const geometry_msgs::msg::Twist &speed,
                                                             nav2_core::GoalChecker */*goal_checker*/) override {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = clock_->now();
        cmd.header.frame_id = "base_link";
        
        if(global_plan_.poses.empty()){
            RCLCPP_WARN(logger_,"Global Plan is empty, stopping robot");
            return cmd;
        }
        if(checkDistanceFromGoal()<0.2){
            RCLCPP_INFO(logger_,"Goal reached, stopping robot.");
        }
        
        double best_v = 0.0, best_w = 0.0;
        double min_cost = std::numeric_limits<double>::max();
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (double v = std::max(0.0, current_v_ - 0.1);  v < 0.22 && v <= (current_v_ + 0.1); v += 0.05) {
            for (double w = std::max(-2.84, current_w_ - 0.5); w < 2.84 && w <= (current_w_ + 0.5); w += 0.1) {
                if(!isTrajectorySafe(v,w)){
                    continue;
                }
                double cost = evaluateTrajectory(v, w);
                if (cost < min_cost) {
                    min_cost = cost;
                    best_v = v;
                    best_w = w;
                }
            }
        }
        
        cmd.twist.linear.x = best_v;
        cmd.twist.angular.z = best_w;
        return cmd;
    }

    bool isTrajectorySafe(double l_v, double a_w) {
        double x = current_x_;
        double y = current_y_;
        double theta = current_yaw_;
        
        for (int i = 0.01; i < 0.5; i+=0.01) {
            x += l_v * std::cos(theta) * i;
            y += l_v * std::sin(theta) * i;
            theta += a_w * i;
            
            unsigned int mx, my;
            if (!costmap_->worldToMap(x, y, mx, my)) {
                return false;
            }
            
            if (costmap_->getCost(mx, my) > nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                return false;
            }
        }
        return true;
    }
    double evaluateTrajectory(double v, double w) {
        double x = current_x_ + v * std::cos(current_yaw_) * 0.5;
        double y = current_y_ + v * std::sin(current_yaw_) * 0.5;
        
        double goal_x = global_plan_.poses.back().pose.position.x;
        double goal_y = global_plan_.poses.back().pose.position.y;
        
        double dist_to_goal = std::hypot(goal_x - x, goal_y - y);
        return abs(w)+2*dist_to_goal;
    }

    void setSpeedLimit(const double &speed_limit, const bool &percentage) override {
        max_speed_ = percentage ? max_speed_default_ * (speed_limit / 100.0) : speed_limit;
    }

private:
    rclcpp::Logger logger_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    rclcpp::Rate node_rate_ = rclcpp::Rate(10.0);
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time last_update_time_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D *costmap_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
    
    nav_msgs::msg::Path global_plan_;
    double current_x_ = 0.0, current_y_ = 0.0, current_v_ = 0.0, current_w_ = 0.0, current_yaw_ = 0.0;
    
    double max_speed_default_ = 0.22;
    double max_speed_;
    std::vector<float> laser_ranges_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_v_ = msg->twist.twist.linear.x;
        current_w_ = msg->twist.twist.angular.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        laser_ranges_ = msg->ranges;
    }
};

PLUGINLIB_EXPORT_CLASS(DWAPlanner, nav2_core::Controller)
