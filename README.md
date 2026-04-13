# Autonomous Navigation System (WATonomous ASD Assignment)

A simple autonomous navigation system built for the WATonomous ASD admission assignment.  
The project enables a simulated robot to navigate from point A to point B while avoiding obstacles using ROS-based architecture.

## Features

- Autonomous point-to-point navigation
- Obstacle avoidance using LiDAR data
- Costmap generation for environment representation
- Global path planning using A* algorithm
- Path following using Pure Pursuit control
- Modular ROS node architecture

## Components Used

- ROS 2 (Robot Operating System)
- C++ / Python
- Gazebo simulation
- Foxglove visualization
- LiDAR (LaserScan data)
- Odometry data

## Demo

[Video](https://youtu.be/fQkYwO3_WmU)

## How It Works

1. The robot receives **LiDAR data** from the `/lidar` topic  
2. A **Costmap Node** converts sensor data into an occupancy grid  
3. A **Map Memory Node** builds a global map over time  
4. A **Planner Node** computes a path using the A* algorithm  
5. A **Control Node** follows the path using Pure Pursuit  
6. Velocity commands are sent to `/cmd_vel` to move the robot  

## Default Behavior

- Navigates toward a user-defined goal point  
- Avoids obstacles based on costmap data  
- Replans path when environment changes  
- Stops when the goal is reached  

## ROS Topics Used

- `/lidar` → LaserScan data  
- `/costmap` → Local occupancy grid  
- `/map` → Global map  
- `/goal_point` → Target destination  
- `/path` → Planned trajectory  
- `/cmd_vel` → Robot velocity commands  
- `/odom/filtered` → Robot position  

## Libraries / Tools Required

- ROS 2
- Docker
- Foxglove Studio

Install dependencies using Docker as described in the assignment.

## Setup & Running

1. Clone the repository:
   ```bash
   git clone git@github.com:WATonomous/wato_asd_training.git
   cd wato_asd_training
   ```

2. Build the environment:
   ```bash
   ./watod build
   ```

3. Run the simulation:
   ```bash
   ./watod up
   ```

4. Open Foxglove to visualize robot data

## Key Concepts

- Costmaps for obstacle representation  
- A* path planning on occupancy grids  
- Pure Pursuit control for path tracking  
- ROS publishers and subscribers  
- Modular robotics system design  

## Future Improvements

- Dynamic obstacle avoidance  
- Sensor fusion (camera + LiDAR)  
- Improved path smoothing  
- Real-world robot deployment  
- Machine learning-based perception  

## License

This project is licensed under the MIT License.

## Completed by: Rudra Shukla, Aabjosh Singh, Bhavya Kurseja



