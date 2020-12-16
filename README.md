# Project: Home-Service-Bot 

This project uses a simplified Spherebot in conjuction with the [ROS Navigation](http://wiki.ros.org/navigation) stack to "pickup" a virtual object from one location and "deliver" it to another. Technically, Spherebot doesn't actually pick-up or drop-off anything, but rather communicates with two custom nodes: [Pick Objects](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/pick_objects/pick_objects.cpp) and [Add Markers](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/add_markers/src/add_markers.cpp), to imitate this behavior.

The project employs several ROS Navigation packages: [Teleop Twist Keyboard](http://wiki.ros.org/teleop_twist_keyboard) to drive Spherebot via keyboard during mapping, [GMapping](http://wiki.ros.org/gmapping) to generate a world map, [AMCL](http://wiki.ros.org/amcl) to localize Spherebot relative to the world map, [Move Base](http://wiki.ros.org/move_base) to perform trajectory-path planning and command Spherebot relative to that path, and finally [RViz](http://wiki.ros.org/rviz) to place navigation goals and visualize aspects of the whole process. Please see more detailed descriptions of this functionality and the components involved in the [Execution Section](#Execution:) below, including instruction for testing various components.

Note: I'm not using Turtlebot.
Note: I put some of my maps and config directories in spherebot node

![Navigation](https://github.com/ajdonich/home-service-bot/blob/main/navigation.jpg)


## Installation and Build:
___

Note: the project requires both [ROS](http://wiki.ros.org/ROS/Installation) and [Gazebo](http://gazebosim.org/) to be appropriately installed (including PATH access to respective binaries) on the client machine. The project was developed on a Ubuntu system with Gazebo v7.16.0 and a ROS Kinetic Kame distro. To install and build:

``` bash
$ git clone https://github.com/ajdonich/home-service-bot.git
$ cd home-service-bot/catkin_ws/src
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard
$ cd ..
$ catkin_make
```

This build will generate two directories in *catkin_ws*: *build* and *devel*. Though not explicitly necessary if you are running exclusively from the shell scripts in this repo, you will need to source the ros/catkin setup file if you plan to execute and ROS commands. To do so execute: 
``` bash
$ source devel/setup.bash
```
**Note:** you can easily automate this step by including the following in your *.bash_profile* (or similar in other shells):

``` bash
ROS_SETUP=/home/workspace/home-service-bot/catkin_ws/devel/setup.bash
if test -f "$ROS_SETUP"; then
    echo "Catkin workspace found, sourcing setup file:"
    echo "${ROS_SETUP}"
    cd /home/workspace/home-service-bot/catkin_ws
    source devel/setup.bash
    echo ""
fi
```


## Execution:
___

### Mapping:

![Mapping](https://github.com/ajdonich/home-service-bot/blob/main/mapping.jpg)

The initial step is to generate a static 2D-map of the simulated Gazebo world. Spherebot is equipped with a Hokuyo Lidar (mounted to his head) and a Skid Steer Drive Controller. The GMapping Node subscribes to laser scan and odometry messages published by these components, respectively, and implements a [Rao-Blackwellized Particle Filer](https://openslam-org.github.io/gmapping.html) to learn grid maps from the data. This repo already contains a version of this map (viewable JPG): [world_gmap.pgm](https://github.com/ajdonich/home-service-bot/blob/main/world_gmap.jpg) for the world [home_service.world](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/spherebot/worlds/home_service.world), but the process of generating such a map is as follows:  
``` bash
$ cd /home/workspace/home-service-bot/catkin_ws/src/scripts
$ ./test_slam_spherebot.sh
```
This command will **(1)** launch a Gazebo GUI with Spherebot spawned in its world, **(2)** launch a GMapping Node and RViz GUI, and **(3)** launch a Teleop Node, each from a unique XTerm window. To generate your map, you must use keyboard commands (as described in the Teleop XTerm window) to manually drive Spherebot thoroughly throughout the world environment. RViz will continously display your map as it generates, enabling you to see what areas remain uncharted as you go. Upon completion of a satisfactory map displayed in RViz, execute the following to export it into a PGM image file and corresponding YAML metadata file:
``` bash
$ rosrun map_server map_saver -f <map_file_name>
```
___
### Navigation and Localization:
![Localization](https://github.com/ajdonich/home-service-bot/blob/main/localization.jpg)

Next, localization and navigation of Spherebot can be accomplished with respect to the 2D-map just generated. RViz provides a 2D Nav Goal button in its menu bar enabling the placement of a navigation goal marker anywhere within the world (and underlying trigger of a Move Base action command). The Move Base node maintains two [costmaps](https://wiki.ros.org/costmap_2d) used to accomplish navigation tasks, one for a global planner (based on the static 2D-map) and one for a local planner (based on continuous incoming laser-scan data). At the same time an AMCL Node performs (Adaptive Monte Carlo) localization using a particle filter to track Spherebot's pose against the world map.

A significant number of important configuration parameters are available in [YAML Config Files](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/spherebot/config) to tune Move Base path-planning and navigation, and in [amcl_demo.launch](https://github.com/ajdonich/amc-localization/blob/main/catkin_ws/src/spherebot/launch/amcl_demo.launch) to tune localization. Of note are the **inflation_radius** parameter controlling the degree of Minkowski-sum obstacle inflation within costmaps, robot **vel\*/acc\*** parameters to tune the Move Base drive commands to the Skid Steer Controller, and **odom_alpha\*** noise values to improve localization accuracy (simulated Gazebo odom is noise free). To run the test execute:
``` bash
$ cd /home/workspace/home-service-bot/catkin_ws/src/scripts
$ ./test_navigation.sh
```
This command will **(1)** launch a Gazebo GUI with Spherebot spawned in its world, **(2)** launch a MapServer Node, AMCL, Move Base and RViz Node, each from a unique XTerm window. By default, RViz will be set up to display laser scans, global and local/dynamic costmaps, 2D navigation goal markers, Spherebot's base footprint, planned global trajectories and planned local trajectories. The AMCL point cloud can be turned on from the left explorer menu to visualize localization accuracy. To begin actual navigation, it is necessary to set a 2D navigation goal marker somewhere within the world, which will trigger path-planning, navigation, and localization to commense. 

**Trouble Shooting:** Though this test performs reasonably well, even with relatively challenging marker placement (for example, placing navigation markers behind/around obstacles and walls), it is not perfect. Certain marker placements, in conjunction with turn limitations of the Skid Steer Controller can result in wall collisions that, for some reason, the local planner is not dexterious enough to prevent in time. Additionally, the global planner tends to tightly hug inflation boundaries around corners, leaving little room for error, and on occation, the local plan suddenly and erroneously diverges from an otherwise safe global plan and steers Spherebot into an obstacle, yet over-inflating walls or robot footprint buffers tends to confuse the planner into false **stuck** states when there is actually still plenty of space to move. Though planner recovery behavior is enabled, it is mediocre at best. If Spherebot gets stuck, it is often possible to just (wisely) move the nav marker, but sometimes a full restart is needed. 
___

### Add Marker:
![Add Markers](https://github.com/ajdonich/home-service-bot/blob/main/add-markers.jpg)

Adding a marker to the RViz can be accomplised by simply publishing a [Marker Msg](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html) and configuring RViz to subscribe to the chosen topic. My [Add Markers](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/add_markers/src/add_markers.cpp) Node performs the task by providing a [Place Marker Service](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/add_markers/srv/PlaceMarker.srv). You can use the following script, which simply employs the [rosservice](http://wiki.ros.org/rosservice) CLI, to  test of this functionality. A cube-shaped marker will appear in RViz at one location, remain there for 5 seconds, then disappear for 5 seconds, and finally reappear at a different location.
``` bash
$ cd /home/workspace/home-service-bot/catkin_ws/src/scripts
$ ./add_marker.sh
```
___

### Home Service:
![Home Service](https://github.com/ajdonich/home-service-bot/blob/main/home-service.jpg)

Finally, my [Pick Objects](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/pick_objects/src/pick_objects.cpp) node employs a [SimpleActionClient](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient) to communicate with the Move Base node to facilitate path planning in very similar fashion to the RViz 2D Nav Goal button. In conjuction it subscribes to Marker and Odometry messages and uses the Place Marker Service to imitate a virtual object delivery service (see node's XTerm for play-by-play logging). After bringing up the system, the [Home Service Script](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/scripts/home_service.sh) uses the Place Marker Service to insert a cube-shaped virtual object, which kicks off this virtual-object-delivery-service. After delivering the virtual object, Sperebot will go to a pre-defined "waiting" zone until it receives further object delivery prompts (i.e. Marker messages). To test execute:

``` bash
$ cd /home/workspace/home-service-bot/catkin_ws/src/scripts
$ ./home_service.sh
```
___  
  
### Implementation Notes:

Spherebot's stripped down design specification can be found in [spherebot.xacro](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/spherebot/urdf/spherebot.xacro) and [spherebot.gazebo](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/spherebot/urdf/spherebot.gazebo). For his more complete implementation, see the [Spherebot](https://github.com/ajdonich/spherebot) repo. Configuration of both RViz settings and Move Base parameters are in [Config Dir](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/spherebot/config). Node and simulation launching is done through a combination of [Shell Scripts](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/scripts) and [Launch Files](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/spherebot/launch). 

The most complicated software logic is contained in [pick_objects.cpp](https://github.com/ajdonich/home-service-bot/blob/main/catkin_ws/src/pick_objects/src/pick_objects.cpp). This process runs two threads: one to receive incoming Odometry and Marker information, and a second thread to manage a simple state machine to coordinate the virtual object delivery behavior.

