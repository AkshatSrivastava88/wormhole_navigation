# 🗺️ Wormhole Navigation System for Multi-Map Robots

**Author:** Akshat Srivastava  
**Test Platform:** ANSCER AR100 (adaptable to any AMR)  
**Contact:** akshatsrivastava8840@gmail.com

---

## 🚀 Overview

This project implements a robust multi-map navigation system for autonomous mobile robots, enabling seamless navigation between separately mapped rooms using a "wormhole" mechanism. The robot can switch maps at predefined transition points (wormholes), such as doorways or hallways, and continue navigation to its goal in the new map. The system is modular, extensible, and designed for easy integration with existing ROS-based navigation stacks.

---

## 📷 Gazebo Simulation with Anscer AR100

![Gazebo Simulation environment with Anscer AR100](https://github.com/AkshatSrivastava88/wormhole_navigation/blob/master/media/gazebo_simulation.png)

---

## Maps for Room 1 and Room 2 

<p align="center">
  <img src="https://github.com/AkshatSrivastava88/wormhole_navigation/blob/master/media/map1.jpg" alt="Image 1" width="45%"/>
  <img src="https://github.com/AkshatSrivastava88/wormhole_navigation/blob/master/media/map2.jpg" alt="Image 2" width="45%"/>
</p>

## 🌀 What is a Wormhole?

A **wormhole** in this context is a special, predefined location in one map that links directly to a corresponding location in another map—typically an overlapping region like a doorway or corridor. When the robot reaches a wormhole, it can "jump" to the corresponding spot in the next map, relaunching the map server and continuing navigation as if the environment were continuous.

---

## 🏛️ System Architecture

The system is composed of several tightly integrated components, each responsible for a key aspect of multi-map navigation:

### 1. Navigation Server

- **Role:**  
  Acts as the central orchestrator for multi-map navigation. Receives navigation goals, determines if a map switch is needed, and coordinates the entire process.
- **Features:**  
  - Receives navigation goals (target x, y, and map name) via a ROS Action Server.
  - Checks if the goal is in the current map or another map.
  - If a map switch is required, determines the optimal wormhole path (direct or via an intermediate map).
  - Manages the sequence: navigate to wormhole → switch map → teleport robot → continue to goal.

### 2. Wormhole Manager

- **Role:**  
  Handles all interactions with the SQLite database that stores wormhole connections.
- **Features:**  
  - Loads wormhole data at startup.
  - Provides fast queries for direct and indirect wormhole connections between maps.
  - Returns precise (x, y) coordinates for navigation and map switching.

### 3. Map Switcher

- **Role:**  
  Dynamically manages the ROS map server, loading the correct map as needed.
- **Features:**  
  - Launches or relaunches the map server with the appropriate map YAML file.
  - Ensures the robot’s localization is consistent after a map switch.
  - Teleports the robot to the corresponding wormhole location in the new map.

### 4. Database (wormholes.db)

- **Role:**  
  Stores all wormhole connections and their coordinates.
- **Schema:**
  | from_map | to_map | from_x | from_y |
  |----------|--------|--------|--------|
  |  TEXT    |  TEXT  |  REAL  |  REAL  |
- **Features:**  
  - Centralized, extensible, and easy to query.
  - Supports direct and indirect navigation paths.

---

## 🔄 Navigation Process

1. **Goal Reception:**  
   The robot receives a navigation goal specifying a target position and map.

2. **Current Map Evaluation:**  
   - If the goal is in the current map, the robot navigates directly using `move_base`.
   - If the goal is in another map, the system plans a path using wormholes.

3. **Path Planning:**  
   - **Direct Wormhole:** If a direct wormhole exists, the robot navigates to it, switches maps, and continues to the goal.
   - **Indirect Path:** If no direct wormhole exists, the robot first navigates to an intermediate map (e.g., a central hub like `map1`), then to the target map.

4. **Map Switching:**  
   - The robot stops the current map server.
   - The map server is relaunched with the new map.
   - The robot’s pose is updated to the corresponding wormhole location in the new map.

5. **Navigation Completion:**  
   - The robot continues to the final goal.
   - The action server provides real-time feedback and a detailed result.

---

## 🗂️ Project Structure

```
wormhole_navigation/
├── action/
│   └── NavigateToGoal.action         # ROS action definition for multi-map navigation goals
├── database/
│   └── wormholes.db                  # SQLite database storing wormhole coordinates
├── include/
│   └── wormhole_navigation/
│       ├── map_switcher.h            # Header for MapSwitcher class
│       ├── navigation_server.h       # Header for NavigationServer class (action server logic)
│       └── wormhole_manager.h        # Header for WormholeManager class (database queries)
├── launch/
│   └── navigation_server.launch      # ROS launch file to start the navigation server node
├── maps/
│   ├── map1.yaml                     # Map 1 yaml metadata (resolution, origin, etc.)
│   ├── map1.pgm                      # Map 1 occupancy grid image
│   ├── map2.yaml                     # Map 2 yaml metadata
│   ├── map2.pgm                      # Map 2 occupancy grid image 
├── src/
│   ├── map_switcher.cpp              # Implementation of MapSwitcher (map switching logic)
│   ├── navigation_server.cpp         # Implementation of NavigationServer (main action server)
│   └── wormhole_manager.cpp          # Implementation of WormholeManager (DB access)
├── CMakeLists.txt                    # Build configuration for the ROS package
├── package.xml                       # ROS package manifest (dependencies, metadata)
└── readme.md                         # Project documentation
```

---

## 🛠️ Dependencies

- move_base
- actionlib
- dwa_planner
- amcl
- tf2_ros
- std_msgs
- actionlib_msgs
- move_base_msgs
- message_generation
- rospack
- roslib
- sqlite3

---

## 🏗️ Build Instructions

```sh
cd ~/catkin_ws/src
git clone https://github.com/AkshatSrivastava88/wormhole_navigation.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## ▶️ Run Instructions

1. **Launch the navigation server:**
   ```sh
   roslaunch wormhole_navigation navigation_server.launch
   ```

2. **Send a navigation goal (example using rostopic):**
   ```sh
   rostopic pub /navigate_to_goal/goal wormhole_navigation/NavigateToGoalActionGoal "goal:
     target_x: -7.0
     target_y: -3.5
     target_map: 'map2'"
   ```

3. 
    ```bash
    rostopic pub /navigate_to_goal/goal multi_map_nav/NavigateToGoalActionGoal "header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    goal_id:
      stamp:
        secs: 0
        nsecs: 0
      id: ''
    goal:
      target_x: -7.0
      target_y: -6.0
      target_map: 'map1'"
    ```

4. **Send a navigation goal (example using action client file):**
    ```py
    #!/usr/bin/env python3
    import rospy
    import actionlib
    from multi_map_nav.msg import NavigateToGoalAction, NavigateToGoalGoal

    def send_goal():
        client = actionlib.SimpleActionClient('navigate_to_goal', NavigateToGoalAction)
        client.wait_for_server()

        goal = NavigateToGoalGoal()
        goal.target_map = "map2"
        goal.target_x = 7.0
        goal.target_y = -3.0

        client.send_goal(goal)
        client.wait_for_result()

        return client.get_result()

    if __name__ == '__main__':
        rospy.init_node('navigation_client')
        result = send_goal()
        print("Result:", result.success, result.message)
    ``` 




---

## 📋 Action Definition

**action/NavigateToGoal.action**
```
float64 target_x
float64 target_y
string target_map
---
bool success
string message
---
string feedback_msg
```

---

## 🗃️ Example: Wormhole Database

**Schema:**
```sql
CREATE TABLE wormholes (
    from_map TEXT,  -- Source map name
    to_map TEXT,    -- Destination map name
    from_x REAL,    -- X-coordinate in source map
    from_y REAL     -- Y-coordinate in source map
);
```

**Sample Entries:**
```sql
INSERT INTO wormholes VALUES ('map1', 'map2', -7.0, -7.5);
INSERT INTO wormholes VALUES ('map2', 'map1', -7.0, -3.5);
```

---

## 🧩 Example Use Case

Suppose the robot is in `map1` and receives a goal in `map2`. The system will:

1. Query the database for a wormhole from `map1` to `map2`.
2. Navigate to the wormhole location in `map1`.
3. Switch to `map2` and teleport the robot to the corresponding wormhole location.
4. Continue navigation to the final goal in `map2`.

If no direct wormhole exists, the robot will route via an intermediate map (e.g., `map1` as a central hub).

---

## 📚 Appendices

### Appendix A: Database Queries

- **View all wormhole connections:**  
  `SELECT * FROM wormholes;`
- **Add a new wormhole connection:**  
  `INSERT INTO wormholes (from_map, to_map, from_x, from_y) VALUES ('map1', 'map2', -7.0, -7.5);`
- **Delete a wormhole connection:**  
  `DELETE FROM wormholes WHERE from_map='map1' AND to_map='map2';`
- **Update wormhole coordinates:**  
  `UPDATE wormholes SET from_x=-8.0, from_y=-8.0 WHERE from_map='map1' AND to_map='map2';`

### Appendix B: Configuration Files

**navigation_server.launch**
```xml
<launch>
  <node pkg="wormhole_navigation" type="navigation_server" name="navigation_server" output="screen">
    <param name="wormhole_db_path" value="$(find wormhole_navigation)/database/wormholes.db" />
    <param name="map_folder" value="$(find wormhole_navigation)/maps" />
  </node>
</launch>
```

**maps/map1.yaml**
```yaml
image: map1.pgm
resolution: 0.050000
origin: [-12.200000, -20.200000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## 🏁 Conclusion

The `wormhole_navigation` package provides a practical, scalable, and modular solution for multi-map navigation in autonomous robots. By leveraging wormhole-based transitions and a robust ROS architecture, it enables robots to navigate complex, multi-room environments with ease. The system is fully tested on the ANSCER AR100 platform and is adaptable to other AMRs and environments.

---

**For questions or contributions, please contact:**
Akshat Srivastava — akshatsrivastava8840@gmail.com
