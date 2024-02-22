# Lab1-Local-Planner
## 1. Getting Started
## 1.1 ROS2 setup environment
Follow the [Official ROS 2 tutorial](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html) to create your Colcon workspace.

```bash
mkdir ~/your_workspace
cd ~/your_workspace
colcon build
source install/setup.bash
```

To automate workspace setup every time you open a new terminal, add the following line to your `~/.bashrc` or `~/.bash_profile` file:

```bash
echo "source ~/your_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 1.2 Intallation package
To installation this project

```bash
cd ~/your_workspace
git clone https://github.com/tuchapong1234/Lab1-Local-Planner.git
```

## 1.3 Running robot
To start control robot you can run launch file

```bash

```
```bash
ros2 launch carver_bringup carver_bringup.launch.py
```
# 2. System overview
## 2.1 system_interface_diagram

![Screenshot from 2024-02-22 20-27-46](https://github.com/tuchapong1234/Lab1-Local-Planner/assets/113016544/d2a31ae8-4852-473a-b302-04e571de2f20)

### Block Description

white box: package / exacutable file

green box: subscript topic

blue box: publish topic

blue box: tf

purple box: tunning parameter
## 2.2 rqt_graph 
```bash
rqt_graph
```
![Screenshot from 2024-02-22 19-28-23](https://github.com/tuchapong1234/Lab1-Local-Planner/assets/113016544/bd7dbe52-339f-4410-8ac3-f837e0a8310f)

# 3. Odometry
## 3.1 Wheel Odometry 
wheel odometry is one of methods to estimate pose of the robot. It involves using sensors, usually encoders, mounted on a vehicle's wheels to measure their rotational movement. By tracking the number of rotations with robot description (wheel diameter, distance between wheel) can use to calculate how far the vehicle has traveled. This information is then used to estimate the vehicle's position and orientation based on the assumption that the wheels have not slipped or skidded significantly. While wheel odometry is a useful method for estimating short-term movement in robotics and autonomous vehicles, it can be affected by wheel slippage, uneven terrain, and other factors, leading to inaccuracies over time.

&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; ![Screenshot from 2024-02-22 17-13-44](https://github.com/tuchapong1234/Lab1-Local-Planner/assets/113016544/3b44d227-e3e6-45d6-b7bc-1e179054e595)

### Forward Kinematics 

&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; $w_{robot}\ =\frac{r\cdot\dot{\phi_{L}}\ -\ r\cdot\dot{\phi_{R}}\ }{B}$


&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; $v_{robot}\ =\frac{r\cdot\dot{\phi_{L}}\ +\ r\cdot\dot{\phi_{R}}\ }{2}$

### Inverse Kinematics 

&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; $\dot{\phi_{L}}\ =\ \ \frac{2\cdot v_{robot}\ +\ B\cdot w_{robot}}{2\cdot r}$

&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; $\dot{\phi_{R}}\ =\ \ \frac{2\cdot v_{robot}\ -\ B\cdot w_{robot}}{2\cdot r}$

Where:

$w_{robot}$ is the angular velocity of robot.

$v_{robot}$ is the linear velocity of robot.

$r$ is the radius of the wheel.

$B$ is the distance between the wheel.

$\dot{\phi_{L}}\$ is the angular velocity of left wheel.

$\dot{\phi_{R}}\$ is the angular velocity of right wheel.

In this work we use 
- Forward kinematic to transform Wheel velocity or `/velocity_controllers/commands` to robot velocity or `/cmd_vel`
- Inverse kinematic to transform robot velocity or `/cmd_vel` to Wheel velocity or `/velocity_controllers/commands`

# 4. Local Planner

A local planner is use to navigate by constrant of immediate obstacles and make short-term decisions. It's like the brain's reflexes, quickly adjusting movement based on what's directly in front of it. Instead of planning a whole journey, it focuses on the next few steps, considering things like nearby objects, terrain, and the robot's capabilities to choose the best path in real-time. So, while the global planner maps out the overall route, the local planner handles the nitty-gritty details to keep things moving smoothly.

## 4.1 Pure Pursuit Algorithm
Pure Pursuit Algorithm is a method used in robotic or autonomous vehicle control for path tracking. It calculates the steering commands required to follow a specified path by continuously aiming to intercept a reference point on that path. The algorithm works by selecting a point ahead of the vehicle along the desired path. This point serves as the target for the vehicle to reach.The algorithm generates steering commands that guide the vehicle towards the path while maintaining a smooth trajectory.

&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; ![image](https://github.com/tuchapong1234/Lab1-Local-Planner/assets/113016544/81d92bd2-316b-437f-a603-689f61e9f2db)

### Pseudo Code

    path [array of point generate from the robot to goal point by nav2]

    lookahead distance [distance between robot position and the goal point from Pure Pursuit Algorithm]

    p_x, p_y [Init empty list]

    current_pose_index [Init select point in path]

    For i from current_pose_index to end of path do

        Append position from the path at i index to p_x and p_y 

    Convert list of p_x and p_y to numpy arrays as (p_x_np and p_y_np)

    distance_array = Calculate the Euclidean distance between p_x_np, p_y_np and robot position 

    Find indices where distance_array is greater than lookahead distance

    Find the index of the minimum distance within indices and set it as min_goal_point_idx

    Update current_pose_index based on min_goal_point_idx and previous current_pose_index

    goal_point = Find the different between path[current_pose_index] (current goal point) and robot position 

    Return goal_point [Return goal_point to use as a local set point for robot to follow]

## 5. Obstacle Avoidance

obstacle avoidance refers to the process of designing and implementing systems that enable robots or autonomous vehicles to detect and navigate around obstacles in their environment. This typically involves using sensors such as cameras, lidar, radar, or ultrasonic sensors to perceive the surroundings, and then employing algorithms to analyze this sensory data and make decisions about how to maneuver to avoid collisions. The algorithms may involve techniques such as path planning, trajectory generation, and control theory to ensure safe and efficient navigation. Conclusion, obstacle avoidance is a critical aspect of autonomous systems design, enabling them to operate safely and effectively in complex and dynamic environments.

## 5.1 Virtual Force Field (VFF) Algorithm

Virtual Force Field (VFF) algorithm is a computational method used for simulating the behavior of interconnected objects within a system. Attraction and Repulsion: Each object in the virtual environment exerts a force on other nearby objects. Contain of two force can be either attract them (like gravity) or repel them (like a magnetic force). VFF algorithm calculates these virtual forces between objects based on their positions and certain properties like mass or charge. For example, if two objects are close together, they might exert an attractive force on each other, trying to pull them closer. If they get too close, they might exert a repulsive force to prevent them from colliding.
VFF algorithm provides a way to simulate the behavior of objects in a virtual environment by modeling the forces acting on them and how they respond to those forces.

&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; ![image](https://github.com/tuchapong1234/Lab1-Local-Planner/assets/113016544/48505d37-ea1b-4a79-b81b-fae3bb801062)

### Pseudo Code

    OBSTACLE_DISTANCE [Threshold distance for obstacle influence]
    
    vff_vector [Initialize the VFF vectors containing 3 vector 1. attractive (Goal-directed vector), 2. repulsive (Obstacle avoidance vector), 3. result (Combined vector)]

    vff_vector[attractive] = Find attractive vector from goal_point (position x,y of goal point)

    min_idx = Find the nearest obstacle distance index 
    distance_min = Find the nearest obstacle distance 

    if distance_min less than OBSTACLE_DISTANCE do
        obstacle_angle = Find angle that has the nearest obstacle distance 
        opposite_angle = Make the opposite angle from the obstacle_angle by plus pi to the obstacle_angle 
        complementary_dist = Find the different value of distance_min and OBSTACLE_DISTANCE
        gain = [Weight of the repulsive vector] recommend to use square of complementary_dist

        vff_vector[repulsive_vector] = [Calculate repulsive vector components]
    
    vff_vector[result_vector] = Calculate the resultant vector by combining attractive and repulsive vectors
    
    return vff_vector

## 5. Implementing local planner (Pure Pursuit Algorithm) with Obstacle Avoidance (VFF Algorithm)

You can implement these two algorithms by using a goal point selected from the Pure Pursuit Algorithm as an attractive vector of the VFF Algorithm. These can generate a vector toward the goal but still avoid obstacles by repulsive vectors that generate away from obstacles. The combination of these two vectors can generate a new vector pulling the robot toward the goal and pushing away from obstacles.

### 5.1 Standard VFF Algorithm & Modified VFF Algorithm Testing

### Condition 1: Evaluation in Condition of No Additional Obstacles

1. Evaluating Robot Movement Narrow / Wide Doorway 

2. Evaluating Robot Movement Narrow / Wide Pathways 

3. Evaluating Robot Movement in Narrow Angles / Compact Spaces

### Condition 2: Evaluation in Condition of Additional Obstacles

1. Evaluating Robot Movement in the middle of Wide Pathways with (Cylindrical / Cube)

2. Evaluating Robot Movement offset from the middle of Wide Pathways with (Cylindrical / Cube)

3. Evaluating Robot Movement at the corner with (Cylindrical / Cube)

### 5.2 Comparing Result between Standard VFF Algorithm & Modified VFF Algorithm

### Condition 1: Evaluation in Condition of No Additional Obstacles

| Testing       |  Testing       |Standard VFF  | Modified VFF
| :---: | :---:| :---: | :---:
| Narrow    | Doorway  | Content Cel | Content Cel |
| Wide  | Doorway  | Content Cell | Content Cel |
| Narrow  | Pathways   | Content Cell | Content Cel |
| Wide  | Pathways   | Content Cell | Content Cel |
| Narrow  | Angles  | Content Cell | Content Cel |
| Compact  | Spaces | Content Cell | Content Cel |

### Condition 2: Evaluation in Condition of Additional Obstacles

| Testing       |  Testing       |Standard VFF  | Modified VFF
| :---: | :---:| :---: | :---:
| Cylindrical | middle of Pathways  | Content Cell | Content Cel |
| Cube  | middle of Pathways  | Content Cell | Content Cel |
| Cylindrical  | offset middle of Pathways  | Content Cell | Content Cel |
| Cube  | offset middle of Pathways  | Content Cell | Content Cel |
| Cylindrical  | corner  | Content Cell | Content Cel |
| Cube | corner  | Content Cell | Content Cel |






