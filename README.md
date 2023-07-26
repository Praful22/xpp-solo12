# xpp-solo12

# xpp_solo12
A ROS package for visualizing gaits generated using towr in xpp.

## How to Add Solo12 to Towr
These instructions are written for [towr built with catkin](https://github.com/ethz-adrl/towr#-building-with-catkin).

### Set up the Solo12 visualization in towr
#### Get the xpp_solo12 package
1. Create a catkin workspace (tutorial here http://wiki.ros.org/catkin/Tutorials)
2. Get the source files from the GitHub, put them in the `src` folder of the workspace, and rename the cloned repo to xpp_solo12
3. Changed the hard-coded joint state CSV save path in urdf_visualizer_solo12.cc to wherever you would like to save the file
4. Build the workspace with `catkin_make`
5. Source the workspace by running the commands 
`echo “source devel/setup.bash” >> ~/.bashrc` (writes the source command to the .bashrc file, which runs whenever a new terminal is opened) and `source ~/.bashrc` in the workspace directory

#### Link the xpp_solo12 package to towr
1. In your towr catkin workspace, add `<include file="$(find xpp_solo12)/launch/xpp_solo12.launch"></include>` to `src/towr/towr_ros/launch/towr_ros.launch` under `"Launches the URDF Visualizers"`. When towr is launched, it will now also launch the xpp_solo12 package.
2. Build the towr workspace
3. Launch towr* and create a new visualization within rviz as shown [in this tutorial](http://wiki.ros.org/xpp#Visualize_.2BAC8_Contribute_your_own_robot), but name it Solo12. Put `solo12_rviz_urdf_robot_description` in the Robot Description field and `solo12` in the TF Prefix field. Save when you exit towr. You should now be able to select Solo12 and see it in the visualization pane of towr

### Add the Solo12 kinematics and dynamics models to towr
1. Copy solo12_model.h from the models folder of the xpp_solo12 package and paste it in `src/towr/towr/include/towr/models/examples` of your towr catkin workspace
2. Add Solo12 to `src/towr/towr/include/towr/models/robot_model.h` and `src/towr/towr/src/robot_model.cc` in your towr catkin workspace
    * See the models/robot_model.h and models/robot_model.cc files in the xpp_solo12 package for how the modified files should look
3. Add xpp_solo12 as an executable dependency in `src/towr/towr_ros/package.xml`
4. Build towr
5. Solo12 should show as a robot model option in the xterm user interface

**If you encounter an error that says the towr launch file isn’t found or that the xpp_solo12 package isn’t found, then that means the respective workspace needs to be sourced. If sourcing one of the workspaces seems to undo the sourcing of the other workspace, it means that in the ROS_PACKAGE_PATH environment variable, the path of the other workspace is getting overwritten. See [this link](https://www.theconstructsim.com/overlaying-ros-workspaces/) on how to properly overlay workspaces.*

## Kinematics and Dynamics Model Parameter Derivation
These parameters are used in Solo12's towr robot model, and are located in the solo12_model.h file.

* Weight: 2.5 kg [(source)](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/quadruped_robot_12dof_v1/README.md#quadruped-robot-12dof-v1)
* End effectors: 4
* Origin: Stated to be the center of mass in the Towr paper. Assumed to be the center of the base of the robot
* Nominal position: standing position
* End effector location in nominal position: x = (45 cm / 2 - 3.2 cm) = 19.3 cm, y = (30 cm / 2) = 15 cm, z = height of the robot when standing ([source](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/quadruped_robot_12dof_v1/README.md#dimensions), scroll down to Dimensions)
* Max deviation of end effectors from nominal position: changed through trial and error until trajectory in towr looked realistic
* Inertia tensor: added the inertia tensors of all the links in the Solo12 URDF file

## Inverse Kinematics
<insert info on inverse kinematics>

## How This Package Was Created
### Resources Used
* https://github.com/ethz-adrl/towr 
* https://github.com/leggedrobotics/xpp 
* http://wiki.ros.org/xpp 
* https://medium.com/swlh/7-simple-steps-to-create-and-build-our-first-ros-package-7e3080d36faa 
* http://wiki.ros.org/catkin/Tutorials

### The Process
* Most of the package was written by referencing the Add Your Own Robot section of the Towr github https://github.com/ethz-adrl/towr#add-your-own-robot 
* Created the kinematic and dynamic models based off the quadruped HyQ kinematic and dynamic models (more explanation of the parameters was in the Gait and Trajectory Optimization of Legged Robots paper)
* Made the Solo12 model accessible from towr by adding Solo12 to the RobotModel class 
* Followed this tutorial https://medium.com/swlh/7-simple-steps-to-create-and-build-our-first-ros-package-7e3080d36faa to build a ROS package named xpp_solo12. Based the package off of xpp_hyq
    * Populated the package with the urdf files and meshes from the robot_properties_solo GitHub
    * Created launch file
    * Created executable node for urdf visualization inside the package. Contents for the executable node were based off of http://wiki.ros.org/xpp#Visualize_.2BAC8_Contribute_your_own_robot
    * In the packages CMakeList.txt file, `add_executable(urdf_visualizer_solo12 src/urdf_visualizer_solo12.cc)`, added the following code (the guide missed this step)
    ```
    target_link_libraries(urdf_visualizer_solo12
    ${catkin_LIBRARIES}
    )
    ```
* Added xpp_solo12 as an executable dependency in the towr_ros package.xml
* Added `<include file="$(find xpp_solo12)/launch/xpp_solo12.launch"></include>` to towr_ros.launch so that when towr is launched, it will also launch the xpp_solo12 package
* Launched towr and created new visualization within rviz as shown here http://wiki.ros.org/xpp#Visualize_.2BAC8_Contribute_your_own_robot, but named it Solo12. Then put solo12_rviz_urdf_robot_description in the Robot Description field of Solo12
* In solo12.urdf.xacro and leg.xacro, changed package:// paths to file:// paths because they were throwing "package not found" errors
* Added inverse kinematics
* Added code to urdf_visualizer_solo12.cc to write generated joint positions to a CSV file
