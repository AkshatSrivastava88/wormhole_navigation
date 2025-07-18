// Include the NavigationServer class definition
#include "wormhole_navigation/navigation_server.h"

// Constructor for the NavigationServer class
NavigationServer::NavigationServer(const std::string &db_path)
    : as_(nh_, "navigate_to_goal", boost::bind(&NavigationServer::execute, this, _1), false),
      wormhole_manager_(db_path) // Initialize wormhole manager with the DB path
{
    current_map_ = "map1";  // Default starting map
    as_.start();            // Start the Action Server
    ROS_INFO("Navigation Action Server started successfully. Ready to receive goals.");
}

// Helper function to send a goal to move_base and wait for result
bool NavigationServer::move_base_to(double x, double y, double yaw)
{
    // Create a client to interface with move_base action server
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    
    // Delay for 5 seconds for move_base to be available
    if (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Failed to connect to move_base server within timeout.");
        return false;
    }

    // Define goal message for move_base
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.frame_id = "map";
    mb_goal.target_pose.header.stamp = ros::Time::now();
    mb_goal.target_pose.pose.position.x = x;
    mb_goal.target_pose.pose.position.y = y;

    // Convert yaw angle into quaternion and normalize
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    q.normalize();
    mb_goal.target_pose.pose.orientation = tf2::toMsg(q);

    // Send the goal to move_base and wait for result
    ac.sendGoal(mb_goal);
    ac.waitForResult();

    // Return whether the goal was successfully achieved
    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

// Action server callback that processes received goals
void NavigationServer::execute(const wormhole_navigation::NavigateToGoalGoalConstPtr &goal)
{
    ROS_INFO("Received goal: target_map: %s, target_x: %f, target_y: %f",
             goal->target_map.c_str(), goal->target_x, goal->target_y);

    double yaw = M_PI / 2;  // Default heading for simplicity
    wormhole_navigation::NavigateToGoalResult result;

    // Case 1: Target map is the same as current map — directly go to goal
    if (goal->target_map == current_map_)
    {
        ROS_INFO("Already in target map. Moving directly to goal...");
        if (!move_base_to(goal->target_x, goal->target_y, yaw))
        {
            result.success = false;
            result.message = "Failed to reach goal";
            as_.setAborted(result);
            return;
        }
        result.success = true;
        result.message = "Reached";
        ROS_INFO("Robot successfully reached the goal in map: %s", current_map_.c_str());
        as_.setSucceeded(result);
        return;
    }

    // Case 2: Try direct wormhole from current_map_ to target_map
    auto direct = wormhole_manager_.getWormholeToMap(current_map_, goal->target_map);
    if (direct.first != -9999 && direct.second != -9999)
    {
        ROS_INFO("Found direct wormhole from %s to %s", current_map_.c_str(), goal->target_map.c_str());

        // Move to the wormhole
        if (!move_base_to(direct.first, direct.second, yaw))
        {
            result.success = false;
            result.message = "Failed to reach wormhole";
            as_.setAborted(result);
            return;
        }

        // Switch maps
        map_switcher_.switchToMap(goal->target_map);
        current_map_ = goal->target_map;

        // Move to goal in new map
        if (!move_base_to(goal->target_x, goal->target_y, yaw))
        {
            result.success = false;
            result.message = "Failed to reach goal";
            as_.setAborted(result);
            return;
        }

        result.success = true;
        result.message = "Reached";
        ROS_INFO("Robot successfully reached the goal in map: %s", current_map_.c_str());
        as_.setSucceeded(result);
        return;
    }

    // Case 3: No direct wormhole — go via intermediate map ("map1")
    if (current_map_ != "map1")
    {
        // Step 1: Move from current map to map1
        auto to_intermediate = wormhole_manager_.getWormholeToMap(current_map_, "map1");
        if (to_intermediate.first == -9999 || to_intermediate.second == -9999)
        {
            ROS_ERROR("No wormhole from %s to map1", current_map_.c_str());
            result.success = false;
            result.message = "No wormhole to map1";
            as_.setAborted(result);
            return;
        }

        ROS_INFO("Moving from %s to map1 via (%f, %f)", current_map_.c_str(), to_intermediate.first, to_intermediate.second);

        if (!move_base_to(to_intermediate.first, to_intermediate.second, yaw))
        {
            result.success = false;
            result.message = "Failed to reach intermediate wormhole";
            as_.setAborted(result);
            return;
        }

        map_switcher_.switchToMap("map1");
        current_map_ = "map1";
    }

    // Step 2: Move from map1 to target map
    auto to_final = wormhole_manager_.getWormholeToMap("map1", goal->target_map);
    if (to_final.first == -9999 || to_final.second == -9999)
    {
        ROS_ERROR("No wormhole from map1 to %s", goal->target_map.c_str());
        result.success = false;
        result.message = "No wormhole from map1 to target";
        as_.setAborted(result);
        return;
    }

    ROS_INFO("Moving from map1 to %s via (%f, %f)", goal->target_map.c_str(), to_final.first, to_final.second);

    if (!move_base_to(to_final.first, to_final.second, yaw))
    {
        result.success = false;
        result.message = "Failed to reach final wormhole";
        as_.setAborted(result);
        return;
    }

    map_switcher_.switchToMap(goal->target_map);
    current_map_ = goal->target_map;

    if (!move_base_to(goal->target_x, goal->target_y, yaw))
    {
        result.success = false;
        result.message = "Failed to reach goal";
        as_.setAborted(result);
        return;
    }

    result.success = true;
    result.message = "Reached";
    ROS_INFO("Robot successfully reached the goal in map: %s", current_map_.c_str());
    as_.setSucceeded(result);
}

// ROS node entry point
int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_server_node");
    ros::NodeHandle nh;

    // Load wormhole DB path from parameter or use default
    std::string db_path;
    nh.param<std::string>("wormhole_db_path", db_path,
                          ros::package::getPath("wormhole_navigation") + "/database/wormholes.db");

    ROS_INFO("Starting navigation server with wormhole database at: %s", db_path.c_str());
    NavigationServer navigation_server(db_path);

    ros::spin();

    return 0;
}
