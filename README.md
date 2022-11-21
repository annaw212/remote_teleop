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

    `cd ~/catkin_ws/src`
   
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

  `export ROS_MASTER_URI=http://robot-name:11311`
    
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

To launch the ROS nodes, run `roslaunch remote_teleop rt_nodes.launch`.
 
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

# Conversion from ROS to Bazel

**File Structure of ROS project backend**
```
remote_teleop
├── remote_teleop_robot_backend       # The backend code to be run on the robot
    ├── CMakeLists.txt
    ├── package.xml
    ├── action
        ├── Nudge.action
        ├── PointClickNav.action
        ├── ResetMarker.action
        ├── SpeedToggle.action
        ├── StopNav.action
        ├── TurnInPlace.action
    ├── include
        ├── remote_teleop_server.h
    ├── launch
        ├── costmap_freight.yaml
        ├── rt_nodes.launch
    ├── msg
        ├── Velocity.msg
    ├── src
        ├── remote_teleop_server.cpp
```
**File structure of the Bazel project**
```
carl
├── carrack_ros
    ├── bottle                            # This is where messages are kept
        ├── catkin_ws/src
            ├── fetch_remote_teleop_msgs
                ├── CMakeLists.txt
                ├── action
                    ├── Nudge.action
                    ├── PointClickNav.action
                    ├── ResetMarker.action
                    ├── SpeedToggle.action
                    ├── StopNav.action
                    ├── TurnInPlace.action
                ├── msg
                    ├── Velocity.msg
                ├── package.xml
    ├── remote_teleop                     # This is where the actual project is
        ├── BUILD
        ├── include
            ├── remote_teleop_server.h
        ├── launch
            ├── costmap_freight.yaml
            ├── remote_teleop_launch.py
            ├── rt_nodes.launch
        ├── src
            ├── remote_teleop_server.cpp
```
**Message Conversion**

The most straight-forward step of conversion is moving the messages from your project to the `bottle` folder inside `carl`. Because it uses a `catkin_ws` to contain the messages, nothing in the declaration of the messages needs to be changed, and the files can just be copied and pasted into the folder. The same goes for `CMakeLists.txt` and `package.xml`. 

Run `bazel-pack -f` to build the `bazel_ws` to reflect the new changes. This will also help you 

Note:

* You DO need to create a new folder to contain your messages inside of `catkin_ws/src`. In this example, I created `fetch_remote_teleop_msgs`. 

* Check to make sure your `CMakeLists.txt` and `package.xml` files are updated to contain the correct names and paths to reflect the move.

* You will need to update any `#include` in your source files to reflect the new path.

**Project Conversion**

Create a folder inside of `carrack_ros` that will contain the project. In this case, the folder is named `remote_teleop`.

Inside that folder, you will need a `BUILD` file, your source and header files, and your launch files.

The main difference for the project in Bazel (aside from the `BUILD` file) is that the launch file will be different. Instead of having a `.launch` file, you will have a Python launch file. _In this case, I was able to include my `.launch` files in this project and just call them from the Python launch file, but normally you would not have a `.launch` file._

_**Build File**_

To create the `BUILD` file, you need a few elements:

* If you have header files in your project, you will need a `fetch_cc_library` declaration. 

    * Note: you will need to load this at the top of your file.

* If you have source files in your project, you will need a `fetch_cc_binary` declaration.

    * Note: you will need to load this at the top of your file.

* If you have a Python launch file, you will need a `py_binary` declaration.

* You will need to know which packages your project depends on to run and declare them here.

The following file is what my `BUILD` file looks like.
```
# This declares which rules your project will follow and that you will have a
# fetch_cc_binary and fetch_cc_library in your BUILD file.
load("@carl//:custom_rules.bzl", "fetch_cc_binary", "fetch_cc_library")

fetch_cc_library (
   name = "remote_teleop",
   hdrs = glob([
        "include/remote_teleop_server.h"
   ]),
   include_prefix = "carrack_ros/remote_teleop",
   strip_include_prefix = "//carrack_ros/remote_teleop/include",
   visibility = ["//visibility:public"],
   deps = [
        "//bottle:cpp_msgs",
        "@ros//:roscpp",
        "@ros//:geometry_msgs",
        "@ros//:std_msgs",
        "@ros//:visualization_msgs",
        "@ros//:actionlib",
        "@ros//:actionlib_msgs",
		"@ros//:tf",
		"@ros//:tf2_geometry_msgs",
		"@interactive_markers",
   ],
)

fetch_cc_binary (
    name = "remote_teleop_backend_node",
    srcs = [
        "src/remote_teleop_server.cpp", 
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":remote_teleop",
    ],
)

py_binary(
    name = "remote_teleop_launch",
    srcs = ["launch/remote_teleop_launch.py"],
    data = [
		"launch/rt_nodes.launch",
		"launch/costmap_freight.yaml",
    ],
    visibility = ["//visibility:public"],
    deps = [
		"//boab/launcher:launchboi",
	],
)
```
Build the project using `bazel-pack -f` to check for errors.

_**Launch File**_

The Python launch files are placed inside a `launch` folder inside of the project folder. They depend on a launcher called `LaunchBoi` which launches the node when its command is run.

* Even if you have the `.launch` file, you need to launch the main project node from the Python launch file. You can launch supporting nodes from the `.launch` file (for example, an image rotate node), but the actual project node (`remote_teleop_robot_backend`) cannot be launched from there or an error will occur.

* You can launch multiple nodes from the Python launch file, so you do not need to create multiple of them.

* Most projects (but not this one) include a params file. A param file defines how we want to configure our node. In the example below, there is a `costmap_param_file` but that is not the same thing as an actual params file.

    * It is a Python script containing all the params which gets converted to a json document that’s generated into a special folder.

    * A param file’s path gets passed into each node.

    * Inside the `/etc/fetchcore/profiles` folder, there are default parameter files that are overridden by the specified param file.

    * You need an `init` function for your params.

    * There is no particular way to create a params file, but a good example can be found in the `~/bazel_ws/src/carl/carrack_ros/realsense_params` folder.

The following file is what my Python launch file looks like.
```
#!/usr/bin/env python

import os
from boab.param.writer.base import Exe
from boab.launcher.launchboi import LaunchBoi

if __name__ == "__main__":
    # Launch path info
    pkg_path = "carrack_ros/remote_teleop/launch"       # This is where to find the launch file
    launch_file = "rt_nodes.launch"                     # This is the .launch file from the ROS project
    costmap_param_file = "costmap_freight.yaml"         # This is a .yaml file I use for costmap params in the ROS project

    # Execution path info
    exec_name = "remote_teleop"
    exec_dir = os.path.join(Exe().dir, pkg_path)

    # Create execution command
    # Note how 'roslaunch' is the command. This is because I am launching the .launch file through the LaunchBoi
    exec_cmd = "roslaunch --wait %s/%s parampath:=%s/%s" % (exec_dir, launch_file, exec_dir, costmap_param_file)

    print(exec_cmd)                                     # This is optional
    lb = LaunchBoi(exec_name)                           # Create the LaunchBoi object

    # You can launch multiple nodes using one LaunchBoi
    # The format is as follows: lb.launch(<name_of_node>, <command_used_to_launch_node>, postfix, add_sys_args)
    lb.launch("remote_teleop_backend", "carl-carrack_ros-remote_teleop-remote_teleop_backend_node")
    lb.launch("remote_teleop_robot_backend_node", exec_cmd, "launchboi_kill:=dongs", False)
    
    # Run the LaunchBoi
    lb.run()
```
Build the project.

As a sanity check, make sure your commands have been created by looking in the `~/bazel_ws/install/bin` folder. If they have not, something has gone wrong in your `launch` file.

If all is well, you should be able to run your project using the `carl-carrack_ros-remote_teleop-remote_teleop_launch` command (or whatever your command is) from inside your project’s `launch` folder.
