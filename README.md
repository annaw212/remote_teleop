# Remote Teleop

This project allows the user to load and run a tool to remotely teleop a Fetch Robotics Freight robot.

Package structure:
```
remote_teleop
├── remote_teleop_robot_backend
    ├── CMakeLists.txt
    ├── package.xml
    ├── action
    ├── launch
    ├── msg
    ├── src
├── remote_teleop_rviz_plugin
    ├── CMakeLists.txt
    ├── package.xml
    ├── plugin_description.xml
    ├── rviz_config.rviz
    ├── rosdoc.yaml
    ├── src
└── README.md
```

# Building

Open up the terminal on your computer. I am using Ubuntu 20.04 LTS and ROS Noetic, and this has not been tested on any other OS.

Note: You must have [ROS downloaded](http://wiki.ros.org/noetic/Installation/Ubuntu) to run the project.

Begin by [creating a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Once you have your catkin workspace created, make sure you are in the `src` folder of your catkin workspace.
   - `cd ~/catkin_ws/src`
   
Next, clone the `remote_teleop` repository into your `src` folder. Your directory should now look like the following:
```
catkin_ws (or whatever name you choose)
├── build
├── devel
├── src
    ├── remote_teleop
└── README.md
```
Then, run:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
If there are any errors related to uninstalled packages, please install them and try again.

Tip: add `source /my/path/to/catkin_ws/devel/setup.bash` to your `.bashrc` so you don't have to resource it for every new terminal.

# Running
**Checks**
1. For every terminal open on your local machine, make sure you have set the `ROS_MASTER_URI` to point at the robot.
   ```
   export ROS_MASTER_URI=http://robot-name:11311
   ```
2. Make sure all necessary packages are installed (check by compiling the package using `catkin_make`.

## Launching the Backend Nodes

The backend nodes are launched on the robot. SSH in and then make sure you have a copy of the `remote_teleop` repository on the robot. Follow the instructions from the [Building](https://github.com/annaw212/remote_teleop/new/master?readme=1#building) section to set up the repository.

Next, make sure your local machine's IP address (which can be found by running the command `ifconfig` on a terminal separate from the robot) is inside the robot's `/etc/hosts` file so your robot and local machine can communicate:
    ```
    cd /etc/
    sudo vim hosts
    # add your local machine's IP address and hostname to the file
    ```
    Note: Do the same for the robot's IP address and hostname on your local machine's `/etc/hosts` file.

To launch the ROS nodes, make sure you are in the `/root/` folder of your robot, and run `roslaunch remote_teleop rt_nodes.launch`.
 
## Launching the Rviz Plugin

To load the custom Rviz plugin, open a new terminal and run:
```
export ROS_MASTER_URI=http://robot-name:11311
cd /path/to/remote_teleop/remote_teleop_rviz_plugin/rviz_config.rviz
rviz -d rviz_config.rviz
```
# Using Remote Teleop

Once the Rviz plugin and backend nodes are running, you should be able to see the Remote Teleop Rviz panel, upward and downward RGB camera feeds, the 2D laser scan, Point Cloud, costmap/occupancy grid, interactive marker, and robot mesh.

![image](https://user-images.githubusercontent.com/107591234/189235810-850bfb28-0d41-4312-b21e-1e74651c0139.png)

If you are unable to see the robot mesh, make sure the machine running Rviz has the [ra-fetch-commercial_robots repository](https://github.com/zebratechnologies/ra-fetch-commercial_robots) installed.

**Navigation**

To navigate the robot, drag and rotate the interactive marker to the goal pose and press the `Confirm Coordinates` button on the Remote Teleop Rviz panel. If you have chosen a valid location (meaning free of obstacles), the interactive marker will disappear and be replaced with a goal location marker pin. If you have chosen an invalid location, the interactive marker will turn red to indicate to the user that the robot is unable to navigate to that location, and then will reset itself.

In the event that you move the interactive marker but aren't satisfied with the location, you may press `Reset Marker` to snap the interactive marker back to the robot. This is only valid while the robot is _not_ moving.

_Navigation Goal Marker_

![image](https://user-images.githubusercontent.com/107591234/189235910-447ba340-9801-4f74-abf9-12bed187c888.png)

_Valid Navigation Goal_ & _Invalid Navigation Goal_

![image](https://user-images.githubusercontent.com/107591234/189236151-977b0e5b-e19a-46d4-bfec-8c6cca3a4c95.png)
![image](https://user-images.githubusercontent.com/107591234/189236432-1944b65f-f26d-46db-acbd-4671e917a6b3.png)





As of right now (9/8/2022), the robot does not perform obstacle avoidance. Instead, before and during navigation, it checks the straight-line path between its current location and the goal location for obstacles, and if any are detected, the robot stops and cancels that navigation goal.

**Turn-in-Place**

The robot can be turned in place with the `Turn Left` and `Turn Right` buttons available on the Remote Teleop Rviz panel. These buttons will turn the robot 30&deg; in the respective direction.

If you are interested in turning the robot greater or less than 30 degrees, you may use the interactive marker rotation feature to do so.

**Nudge**

In the event that the user is interested in moving forward or backward a very small distance, they may use the `Nudge Forward` or `Nudge Backward` buttons. These buttons move the robot 15cm in the respective direction.

Please note that obstacle checking does not occur during nudge. Use at your own risk.

**Velocity Toggles**

To change the linear or angular velocity of the robot, use the input stepper toggles to change the velocities. By default the robot is initialized at 0.5 m/s for both linear and angular velocity. The maximum velocity is 1.5 m/s and the minimum velocity is set to 0.1 m/s.

**Virtual E-Stop**

The `STOP` button stops all motion immediately.

Once the button has been pressed and all motion has ceased, the robot is able to accept another navigation or button command.

# Demo Videos

**Navigation Demo**

https://user-images.githubusercontent.com/107591234/189237156-e93c4343-9255-4829-b4d0-fa2e212d0d7e.mp4

**Buttons Demo**

https://user-images.githubusercontent.com/107591234/189237179-5ba1a29f-91fb-4041-878c-c51d1583b548.mp4


