#include "wormhole_navigation/map_switcher.h"

// MapSwitcher class
MapSwitcher::MapSwitcher() : nh_()
{
    // Retrieve the 'map_folder' parameter from the parameter server or use default path
    nh_.param<std::string>("map_folder", map_folder_,
                           ros::package::getPath("wormhole_navigation") + "/maps");

    ROS_INFO("Map switcher initialized with map folder: %s", map_folder_.c_str());

    // Check if map folder path was successfully set
    if (map_folder_.empty())
    {
        ROS_ERROR("Map folder is not set. Please set the 'map_folder' parameter.");
        return;
    }
}

// Method to switch the current map to the one specified by 'map_name'
void MapSwitcher::switchToMap(const std::string &map_name)
{
    // Path to the target YAML map file
    std::string map_yaml_path = map_folder_ + "/" + map_name + ".yaml";

    // Open the map file for validation
    std::ifstream map_file(map_yaml_path);
    if (!map_file.is_open())
    {
        ROS_ERROR("Failed to open map file: %s", map_yaml_path.c_str());
        return;
    }

    // Simple validation: check that the YAML contains an "image:" line
    std::string line;
    bool valid = false;
    while (std::getline(map_file, line))
    {
        if (line.find("image:") != std::string::npos)
        {
            valid = true;
            break;
        }
    }

    // Inavlid map file format if image not found
    if (!valid)
    {
        ROS_ERROR("Invalid map file format: %s", map_yaml_path.c_str());
        return;
    }

    
    map_file.close();

    // Construct the system command to launch the map_server node using the new map
    std::string command = "rosrun map_server map_server " + map_yaml_path;
    
   
    int result = system((command + " &").c_str());

    // duration for node to initialize
    ros::Duration(1.0).sleep();

    if (result == 0)
    {
        ROS_INFO("Successfully switched to map: %s", map_name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to switch to map: %s", map_name.c_str());
    }
}
