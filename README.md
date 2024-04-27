/////////////////////(setup)//////////////////////
// install operating system that support ros2 
// install git
// install VS "code"
// setup network structure 
// setup ssh  
// install ROS2 humble "https://docs.ros.org/en/humble/index.html"
// install colcon
// source ros2 each time starting a terminal or bash it*  "source /opt/ros/humble/setup.bash" 


///////////////////////(GitHub)//////////////////////
project name "Sanad_dev"
git clone git@github.com:sanad39/project.git
colcon build --symlink-install


///////////////////(launch Gazebo)/////////////////////////
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui
source install/setup.bash
ros2 launch sanad rsp.launch.py 
ros2 launch sanad rsp.launch.py use_sim_time:=true

after the new launch file:
ros2 launch sanad launch_sim.launch.py 
ros2 run joint_state_publisher_gui joint_state_publisher_gui 
rviz2


//////// (before add gazebo file)** no need for this now I added a launch file  )///////
ros2 launch gazebo_ros gazebo.launch.py 
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity sanad
ros2 run joint_state_publisher_gui joint_state_publisher_gui 
include the world:
ros2 launch sanad launch_sim.launch.py world:=./src/sanad/worlds/obstacles.world


///////////////(to test the model)/////////////////
source install/setup.bash 
ros2 launch sanad launch_sim.launch.py world:=./src/sanad/worlds/obstacles.world
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2
rqt_graph


///////////////////( push update to git )////////////////////////
cd ~/sanad_ws/src/sanad/
git status
git add .
git status
git commit -m "created core URDF"
git push
//pull
git pull origin main


///////////////////( camera driver )////////////////////////
cd ~/sanad_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
cd ~/sanad_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
source ~/sanad_ws/install/setup.bash
camera test 
ros2 launch realsense2_camera rs_launch.py
ros2 topic list


///////////////////( laser RPLIDAR setup )////////////////////////
sudo apt install ros-humble-rplidar-ros


///////////////////( run the driver )////////////////////////
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=lidar_link -p angle_compensate:=true -p scan_mode:=Standard


///////////////////(  laser rplidar driver setup )////////////////////////
cd ~/robot_ws/src
git clone git@github.com:babakhani/rplidar_ros2.git             //Clone the RPLIDAR ROS2 Package
cd ~/robot_ws
colcon build --symlink-install                                  //Build the Workspace
source ~/robot_ws/install/setup.bash                            //Source the Workspace
ros2 launch rplidar_ros rplidar.launch.py                       //Launch the RPLIDAR Node
source ~/robot_ws/install/setup.bash
rviz2                                                           //Visualizing RPLIDAR Data with 
sudo usermod -aG dialout $USER                                  // Additional Notes Permissions


///////////////////( to test the node with rviz2 )////////////////////////
ros2 run rplidar_ros rplidar_composition


///////////////// to stop the motor to save battery ////////////////
ros2 service call /stop_motor std_srvs/srv/Empty 
ros2 service call /start_motor std_srvs/srv/Empty 


//////////////////////////////////////////////////////////////////
install software on Raspberry pi -git -openssh -vs code -network to sync -install ros -colcon 


///////////////////(  Intel® RealSense™ Depth Camera D435i setup  )////////////////////////
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev -y
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev -y
//Install the RealSense™ SDK
git clone https://github.com/IntelRealSense/librealsense.git                        
//Build and install the SDK
cd librealsense
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) # Compiles using all available cores
sudo make install    
// Install the ROS2 Wrapper for RealSense™                                                               
cd ~/robot_ws/src
git clone -b humble https://github.com/IntelRealSense/realsense-ros.git
rosdep install -i --from-path ~/robot_ws/src --rosdistro humble -y
cd ~/robot_ws
colcon build --symlink-install
source ~/sanad_ws/install/setup.bash
// Launch the RealSense™ Node
ros2 launch realsense2_camera rs_launch.py
// Visualize the Camera Data
rviz2


/////////////////////// motor ////////////////
ros2 run vesc_py_interface vesc_controller




