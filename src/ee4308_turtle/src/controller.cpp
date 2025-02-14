#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::cleanup() { RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

    // ====================================== LAB 1, PROJ 1 ====================================================

    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        tf_ = tf;
        plugin_name_ = name;

        // initialize parameters
        initParam(node_, plugin_name_ + ".desired_linear_vel", desired_linear_vel_, 0.2);
        initParam(node_, plugin_name_ + ".desired_lookahead_dist", desired_lookahead_dist_, 0.4);
        initParam(node_, plugin_name_ + ".max_angular_vel", max_angular_vel_, 1.0);
        initParam(node_, plugin_name_ + ".max_linear_vel", max_linear_vel_, 0.22);
        initParam(node_, plugin_name_ + ".xy_goal_thres", xy_goal_thres_, 0.05);
        initParam(node_, plugin_name_ + ".yaw_goal_thres", yaw_goal_thres_, 0.25);

        // initialize topics
        // sub_scan_ = node_->create_subscription<some msg type>(
        //     "some topic", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::some_callback, this, std::placeholders::_1));
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }


        // get goal pose (contains the "clicked" goal rotation and position)
        // Global_plan_ type is nav_msgs/msg/PoseStamped[]
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        double distance = std::hypot(
            goal_pose.pose.position.x - pose.pose.position.x,
            goal_pose.pose.position.y - pose.pose.position.y
        );

        // Check if already arrived at goal
        if (xy_goal_thres_ > distance) {
            return writeCmdVel(0, 0);
        }

        // Find the point along the path that is closest to the robot.

        geometry_msgs::msg::PoseStamped closest_pose;

        double previous_distance = 1e9;
        size_t index_current_pose = 0;

        while (index_current_pose < global_plan_.poses.size()) {
            closest_pose = global_plan_.poses[index_current_pose];

            double distance = std::hypot(
                closest_pose.pose.position.x - pose.pose.position.x,
                closest_pose.pose.position.y - pose.pose.position.y
            );

            // If distance decreases, the current pose is still behind the robot
            if (distance >= previous_distance) {
                index_current_pose -= 1;
                break;
            
            } else if (index_current_pose == global_plan_.poses.size() - 1) {
                break;
            }

            index_current_pose += 1;
            previous_distance = distance;
        }

        // From the closest point, find the lookahead point

        geometry_msgs::msg::PoseStamped lookahead_pose;
        size_t index_lookahead_pose = index_current_pose;

        while (index_lookahead_pose < global_plan_.poses.size()) {
            lookahead_pose = global_plan_.poses[index_lookahead_pose];

            double distance = std::hypot(
                lookahead_pose.pose.position.x - closest_pose.pose.position.x,
                lookahead_pose.pose.position.y - closest_pose.pose.position.y
            );

            if (distance >= desired_lookahead_dist_) {
                break;
            } else if (index_lookahead_pose == global_plan_.poses.size() - 1) {
                lookahead_pose = goal_pose;
                break;
            }
            index_lookahead_pose += 1;
        }

        // Transform the lookahead point into the robot frame to get (x', y')
        double delta_x = lookahead_pose.pose.position.x - pose.pose.position.x;
        double delta_y = lookahead_pose.pose.position.y - pose.pose.position.y;

        double phi_r = getYawFromQuaternion(pose.pose.orientation);

        double x_dash = delta_x * std::cos(phi_r) + delta_y * std::sin(phi_r);
        double y_dash = delta_y * std::cos(phi_r) - delta_x * std::sin(phi_r);

        // Calculate the curvature c
        double curvature = (2 * y_dash) / ((x_dash * x_dash) + (y_dash * y_dash));

        // Calc omega from v and c

        double linear_vel = desired_linear_vel_;
        double angular_vel = linear_vel * curvature;

        // Constrain omega to within the largest allowable angular speed
        if (std::abs(angular_vel) > max_angular_vel_ && angular_vel != 0.0) {
            angular_vel = (angular_vel / std::abs(angular_vel)) * max_angular_vel_;
        }

        // Constrain v to within the largest allowable linear speed
        if (std::abs(linear_vel) > desired_linear_vel_ && linear_vel != 0.0) {
            linear_vel = (linear_vel / std::abs(linear_vel)) * desired_linear_vel_;
        }

        return writeCmdVel(linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)