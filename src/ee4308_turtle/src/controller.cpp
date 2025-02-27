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
        initParam(node_, plugin_name_ + ".curvature_thres", curvature_thres_, 0.5); // TODO: tune
        initParam(node_, plugin_name_ + ".proximity_thres", proximity_thres_, 0.7); // TODO: tune
        initParam(node_, plugin_name_ + ".lookahead_gain", lookahead_gain_, 0.25); // TODO: tune

        // initialize topics
        sub_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::lidarCallback, this, std::placeholders::_1));
    }

    void Controller::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ranges_ = msg->ranges;
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

        double goal_yaw = getYawFromQuaternion(goal_pose.pose.orientation);
        double robot_yaw = getYawFromQuaternion(pose.pose.orientation);
        double yaw_error = goal_yaw - robot_yaw;

        // Check if already arrived at goal
        if (xy_goal_thres_ > distance) {
            
            yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
            
            if (std::abs(yaw_error) < yaw_goal_thres_) { //need abs cos atan2 can give negative value also, it jsut finds shortest angle
                return writeCmdVel(0, 0);  // Stop only if both position and yaw are aligned
            } else {
                return writeCmdVel(0, max_angular_vel); 
                //return writeCmdVel(0, std::clamp(yaw_error * 2.0, -max_angular_vel_, max_angular_vel_)); 
            }
        }

        // Find the point along the path that is closest to the robot.
        geometry_msgs::msg::PoseStamped closest_pose;
        size_t i;
        double prev_dist = 1e9;
        for (i = 0; i < global_plan_.poses.size(); i++)
        {
            auto point_pose = global_plan_.poses[i];
            double point_pose_x = point_pose.pose.position.x;
            double point_pose_y = point_pose.pose.position.y;
            double dist = std::hypot(point_pose_x - pose.pose.position.x, point_pose_y - pose.pose.position.y);
            if (dist < prev_dist)
            {
                prev_dist = dist;
            } else if (i > 0) {
                closest_pose = global_plan_.poses[i - 1];
                break;
            } else {
                closest_pose = global_plan_.poses[i];
                break;
            }
        }
        
        // From the closest point, find the lookahead point
        geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose;
        for (size_t j = i + 1; j < global_plan_.poses.size(); j++) 
        {
            auto point_pose = global_plan_.poses[j];
            double point_pose_x = point_pose.pose.position.x;
            double point_pose_y = point_pose.pose.position.y;
            double dist = std::hypot(point_pose_x - closest_pose.pose.position.x, 
                             point_pose_y - closest_pose.pose.position.y);
            
            if (dist >= desired_lookahead_dist_)
            {
                lookahead_pose = point_pose;
                break; // found lookahead, so gonna stop iterating
            }
        }

        // Transform the lookahead point into the robot frame to get (x', y')
        double delta_x = lookahead_pose.pose.position.x - pose.pose.position.x;
        double delta_y = lookahead_pose.pose.position.y - pose.pose.position.y;

        //double lookahead_angle = atan2(delta_x, delta_y);
        //double path_yaw_error = robot_yaw - lookahead_angle;
        //path_yaw_error = std::atan2(std::sin(path_yaw_error), std::cos(path_yaw_error));

        //if (path_yaw_error> M_PI/2)
        //{
        //    return writeCmdVel(0, 1);

        //}

        //std::cout << "delta_x " << delta_x << std::endl;
        //std::cout << "delta_y " << delta_y << std::endl;

        double phi_r = getYawFromQuaternion(pose.pose.orientation);

        double x_dash = delta_x * std::cos(phi_r) + delta_y * std::sin(phi_r);
        double y_dash = delta_y * std::cos(phi_r) - delta_x * std::sin(phi_r);

        double lookahead_angle = std::atan2(delta_y, delta_x);
        double heading_error = std::atan2(std::sin(lookahead_angle - phi_r), std::cos(lookahead_angle - phi_r));
        bool move_backward = (std::abs(heading_error) > M_PI_2);

        // Calculate the curvature c
        double denom_ =  ((x_dash * x_dash) + (y_dash * y_dash)) + 1e-6; // to prevent dividing by 0, if somehow it happens
        double curvature = (2 * y_dash) / denom_;

        //std::cout << "Calculated curvature: " << curvature <<std::endl;


        //double v_c;
        //double angular_vel = desired_linear_vel_ * curvature;

        double v_c = desired_linear_vel_;

        if (move_backward) {
            v_c = -desired_linear_vel_;  // Reverse velocity
            curvature = -curvature;      // Flip turning direction
        }


        // Curvature heuristic
        if (std::abs(curvature) > curvature_thres_) {
            v_c *= curvature_thres_ / std::abs(curvature);
        }

        //std::cout << "V_c is: " << v_c << std::endl;
        double angular_vel = desired_linear_vel_ * curvature; 
        // Obstacle heuristic
        
        float closest_obstacle;;
        if (!scan_ranges_.empty()) {
            closest_obstacle = 1000.0;
            for (float range : scan_ranges_) {
                if (!std::isnan(range) && !std::isinf(range)) {
                    //std::cout << "Current range: " << range << std::endl;
                    if (range < closest_obstacle) {
                        //std::cout <<"here"<<std::endl;
                        closest_obstacle = range;
                    }
                }
            }
        } else {
            //std::cout << "Default dist used" << std::endl;
            closest_obstacle = proximity_thres_;
        }

        //std::cout << "closest dist: " << closest_obstacle << std::endl;
        double linear_vel;

        if (closest_obstacle < proximity_thres_) {
            linear_vel = v_c * closest_obstacle / proximity_thres_;
        } else {
            linear_vel = v_c;
        }


        //std::cout << "linear_vel after obstacle heuristic: " << linear_vel << std::endl;

        // Vary lookahead
        desired_lookahead_dist_ = std::abs(linear_vel) * lookahead_gain_;
        if (desired_lookahead_dist_ < 0.3) {
            desired_lookahead_dist_ = 0.3;
        }
        //std::cout << "desired_lookahead_dist_: " << desired_lookahead_dist_ << std::endl;


        //std::cout << "angular_vel: " << angular_vel << std::endl;

        // Constrain omega to within the largest allowable angular speed
        if (std::abs(angular_vel) > max_angular_vel_) {
            angular_vel = (angular_vel / std::abs(angular_vel)) * max_angular_vel_;
        } else if (std::abs(angular_vel) < 0.1) {
            angular_vel = 0.0;
        }

        // Constrain v to within the largest allowable linear speed
        if (std::abs(linear_vel) > desired_linear_vel_) {
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
