#include "ee4308_turtle/planner.hpp"

namespace ee4308::turtle
{   
    // ================ Potential Moves to Neighboring Cells  ======================
    static const std::array<std::tuple<int, int>, 4> MOVES_ = {{
        {0, 1}, // Right
        {0, -1}, // Left
        {1, 0}, // Down
        {-1, 0} // Up
    }};

    std::vector<std::array<int, 2>> generatePathCoordinate(PlannerNode* node) {
        std::vector<std::array<int, 2>> path_coord;
        PlannerNode* current = node;
        while (current != nullptr) {
            path_coord.push_back({current->mx, current->my});
            current = current->parent;
        }

        std::reverse(path_coord.begin(), path_coord.end());
        return path_coord;
    }

    std::vector<std::array<int, 2>> applySavitskyGolaySmoothing(const std::vector<std::array<int, 2>>& original_path, const std::vector<double>& polynomial_fit_kernel) {
        int n = original_path.size();
        int half_window = polynomial_fit_kernel.size() / 2;

        std::vector<std::array<int, 2>> smoothed_path = original_path;

        for (int i = half_window; i < n - half_window; ++i) {
            double new_x = 0.0, new_y = 0.0;
    
            // Apply Savitzky-Golay kernel
            for (int j = -half_window; j <= half_window; ++j) {
                new_x += polynomial_fit_kernel[j + half_window] * original_path[i + j][0];
                new_y += polynomial_fit_kernel[j + half_window] * original_path[i + j][1];
            }
    
            // Store rounded smoothed values
            smoothed_path[i] = {static_cast<int>(round(new_x)), static_cast<int>(round(new_y))};
        }

        return smoothed_path;
    }

    // ======================== Nav2 Planner Plugin ===============================
    std::vector<double> Planner::generate_polynomial_fit_kernel(int sg_half_window_, int sg_order_) {
        int rows = 2 * sg_half_window_ + 1;
        int cols = sg_order_ + 1;
        Eigen::MatrixXd vandermondeMatrix(rows, cols);

        for (int i = 0; i < rows; ++i) {
            int x = i - sg_half_window_;
            for (int j = 0; j < cols; ++j) {
                vandermondeMatrix(i, j) = std::pow(x, j);
            }
        }

        Eigen::MatrixXd vandermondeMatrix_t = vandermondeMatrix.transpose();
        Eigen::MatrixXd polynomial_fit_kernel = (vandermondeMatrix_t * vandermondeMatrix).inverse() * vandermondeMatrix_t;
        std::vector<double> kernel(rows);

        for (int i = 0; i < rows; ++i) {
            kernel[i] = polynomial_fit_kernel(0, i);
        }

        return kernel;
    }

    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        tf_ = tf;
        plugin_name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros->getGlobalFrameID();

        // initialize parameters
        initParam(node_, plugin_name_ + ".max_access_cost", max_access_cost_, 255);
        initParam(node_, plugin_name_ + ".interpolation_distance", interpolation_distance_, 0.05);
        initParam(node_, plugin_name_ + ".sg_half_window", sg_half_window_, 5);
        initParam(node_, plugin_name_ + ".sg_order", sg_order_, 3);

        polynomial_fit_kernel_ = Planner::generate_polynomial_fit_kernel(sg_half_window_, sg_order_);
    }

    bool is_valid_neighbor(int mx, int my, int max_access_cost_, nav2_costmap_2d::Costmap2D* costmap_) { 
        int size_mx = costmap_->getSizeInCellsX();
        int size_my = costmap_->getSizeInCellsY();

        if (mx < 0 || my < 0 || mx >= size_mx || my >= size_my) {
            return false; // Out of bounds
        }

        int cost = static_cast<int>(costmap_->getCost(mx, my));

        if (cost >= max_access_cost_) { // Near an obstacle
            return false; // Invalid node
        }   
        
        return true;
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        // initializations
        int size_mx = costmap_->getSizeInCellsX();
        int size_my = costmap_->getSizeInCellsY();
        int movement_step_size = 1;
        PlannerNodes nodes(size_mx, size_my); // Store nodes in 1D collapsed array
        OpenList open_list; // Initialize empty open-list, implemented using pq // Essentially frontier

        int start_mx, start_my, goal_mx, goal_my;
        costmap_->worldToMapEnforceBounds(
            start.pose.position.x, start.pose.position.y,
            start_mx, start_my); // convert start positions in world coord to map coord

        costmap_->worldToMapEnforceBounds(
            goal.pose.position.x, goal.pose.position.y,
            goal_mx, goal_my); // convert goal positions in world coord to map coord

        // All nodes in PlannerNodes are already defined with f, g, h as infinity
        
        PlannerNode *start_node = nodes.getNode(start_mx, start_my); // Pointer to start pos
        
        start_node->g = 0; // Initialize start node with 0 g-cost
        start_node->h = std::hypot(goal_mx - start_mx, goal_my - start_my);
        start_node->f = start_node->g + start_node->h;

        open_list.queue(start_node); // Queue start node

        //std::cout << "Start pos: " << start_mx << " " << start_my << std::endl;
        //std::cout << "Goal pos: " << goal_mx << " " << goal_my << std::endl;
        
        while (!open_list.empty()) {
            PlannerNode *current_node = open_list.pop();

            if (current_node->expanded) { // node is explored
                continue;
            } else if (std::hypot(goal_mx - current_node->mx, goal_my - current_node->my) < 2) { // current node is goal (might want to implement tolerancing)
                std::vector<std::array<int, 2>> path_coord = generatePathCoordinate(current_node->parent); // Check whether current_node or the parent is required, as writeToPath already used goal pose
                std::vector<std::array<int, 2>> smoothed_path = applySavitskyGolaySmoothing(path_coord, polynomial_fit_kernel_);
                //return writeToPath(smoothed_path, goal);
                return writeToPath(path_coord, goal);
            }
            // Mark as expanded
            current_node->expanded = true;
            //std::cout<< "Current pos: " << current_node->mx << " " << current_node->my << std::endl;

            for (const std::tuple<int, int>& move : MOVES_) {
                int dx = std::get<0>(move);
                int dy = std::get<1>(move);
                
                int neighbor_mx = current_node->mx + dx * movement_step_size;
                int neighbor_my = current_node->my + dy * movement_step_size;

                if (!is_valid_neighbor(neighbor_mx, neighbor_my, max_access_cost_, costmap_)) {
                    continue;
                }
                PlannerNode *neighbor_node = nodes.getNode(neighbor_mx, neighbor_my);
                int cost_at_neighbor = static_cast<int>(costmap_->getCost(neighbor_mx, neighbor_my));
                //std::cout << "Neighbor position: " << neighbor_mx << " " << neighbor_my << std::endl;
                //std::cout << "Cost at neighbor: " << cost_at_neighbor << std::endl;
                int new_g_cost = current_node->g + movement_step_size * (cost_at_neighbor + 1);
                if (new_g_cost < neighbor_node->g) {
                    neighbor_node->g = new_g_cost;
                    neighbor_node->parent = current_node;
                    neighbor_node->h = std::hypot(goal_mx - neighbor_mx, goal_my - neighbor_my);
                    neighbor_node->f = neighbor_node-> g + neighbor_node->h;
                    open_list.queue(neighbor_node);
                }
            }
        }
        std::vector<std::array<int, 2>> empty_path;
        return writeToPath(empty_path, goal);
    }

    nav_msgs::msg::Path Planner::writeToPath(
        std::vector<std::array<int, 2>> coords,
        geometry_msgs::msg::PoseStamped goal)
    {
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = global_frame_id_;
        path.header.stamp = node_->now();

        for (const auto &coord : coords)
        { 
            // convert map coordinates to world coordiantes
            double wx, wy;
            costmap_->mapToWorld(coord[0], coord[1], wx, wy);
            
            // push the pose into the messages.
            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information. 
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);
        }
        
        // push the goal
        goal.header.frame_id = "";  // remove frame id to prevent incorrect transformations.
        goal.header.stamp = rclcpp::Time();  // remove timestamp from header, otherwise there will be time extrapolation issues.
        path.poses.push_back(goal);

        // return path;
        return path;
    }


    void Planner::cleanup()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::activate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::deactivate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }


    // ====================== Planner Node ===================
    PlannerNode::PlannerNode(int mx, int my) : mx(mx), my(my) {}

    // ======================= Open List Implemetations ===========
    bool OpenListComparator::operator()(PlannerNode *l, PlannerNode *r) const { return l->f > r->f; }

    void OpenList::queue(PlannerNode *node) { pq.push(node); }

    PlannerNode *OpenList::pop()
    {
        if (pq.empty())
            return nullptr;
        PlannerNode *cheapest_node = pq.top();
        pq.pop();
        return cheapest_node;
    }

    bool OpenList::empty() const { return pq.empty(); }

    // ======================== Nodes ===============================
    PlannerNodes::PlannerNodes(int num_cells_x, int num_cells_y)
    {
        size_mx = num_cells_x;
        size_my = num_cells_y;

        nodes.reserve(num_cells_x * num_cells_y);
        for (int mx = 0; mx < size_mx; ++mx)
            for (int my = 0; my < size_my; ++my)
                nodes[mx * size_my + my] = PlannerNode(mx, my);
    }

    PlannerNode *PlannerNodes::getNode(int mx, int my)
    {
        if (mx < 0 || my < 0 || mx >= size_mx || my >= size_my)
            return nullptr;
        return &nodes[mx * size_my + my];
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)