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
catkin_ws
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
  You can make a shortcut for setting this in each terminal by editing your `.bashrc` file and creating an `alias`.
2. Make sure you are running `roscore` in a terminal.
3. Make sure all necessary packages are installed (check by compiling the package using `catkin_make`.

## Launching the Backend Nodes

To launch the ROS nodes, you can work on either your local machine or on the robot itself. 
- **Local machine:** `roslaunch remote_teleop rt_nodes.launch`
- **Robot:** Running on the robot is a little move involved, but is the better option when latency is concern.
  - SSH into the robot: `ssh fetch@freightXXXX-XXXX`
  - Make sure your local machine's IP address (which can be found by running the command `ifconfig` on a terminal separate from the robot) is inside the robot's `/etc/hosts` file so your robot and local machine can communicate:
    ```
    cd /etc/
    sudo vim hosts
    # add your local machine's IP address and hostname to the file
    ```
    Note: Do the same for the robot's IP address and hostname on your local machine's `/etc/hosts` file.
    
  - Enter your dev docker if you have one (`docker exec -it docker-name`), otherwise create a new dev docker to work in:
    - Outside of the robot, make a directory to store the updater in: `mkdir ~/temp_dev`.
    - SSH into the robot: `ssh fetch@freightXXXX-XXXX`
    - Backup the cronjob for later and stop the updater: `sudo mv /etc/cron.d/updater_status_checker ~/temp_dev; sudo service updater stop`
    - Bring down the robot docker (delete, unless it is a dev docker that belongs to someone else, in which case stop the docker instead): `cd /etc/fetchcore/docker && docker-compose down` or `docker stop docker-name`
    - Pull yourself a new master_dev docker image: `docker pull quay.io/fetch/dev:robot__master_dev`
    - Make a new folder to store the config for your new dev_docker in: `mkdir ~/dev_docker_config`
    - Copy that configuration from the prod docker: `cp /etc/fetchcore/docker/robot.env /home/fetch/dev_docker_config/robot.env`
    - Edit the `docker-compose` file: `vi /home/fetch/dev_docker_config/docker-compose.yml`
    - Start the dev docker: `cd /home/fetch/dev_docker_config && docker-compose up -d`
    - To check if the docker is running: `docker ps | grep -v k8s`
    - Enter the dev docker: `docker exec -it docker-name`
 - Once you are inside your dev docker, run: `cd /root/` to make sure you are in the root of the robot.
 - Follow the instructions from the [Building](https://github.com/annaw212/remote_teleop/new/master?readme=1#building) section to create a catkin workspace and clone the `remote_teleop` repository.
 - Launch the nodes: `roslaunch remote_teleop rt_nodes.launch`
 
 
## Launching the Rviz Plugin

To load the custom Rviz plugin, open a new terminal and run:
```
cd ~/catkin_ws/src/remote_teleop/remote_teleop_rviz_plugin
rviz -d rviz_config.rviz
```
