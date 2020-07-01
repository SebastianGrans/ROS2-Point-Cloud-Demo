# ROS2 Point Cloud 

![](demo.gif)


This is an example package which demonstrates how to publish a PointCloud2 message using Python in ROS2. The point cloud is visualized using RViz2.

A full write-up can be found on my blog: [sebastiangrans.github.io](http://sebastiangrans.github.io/Visualizing-PCD-with-RViz/)

## Installation

Move to your ROS workspace source folder, e.g:
```
cd ~/dev_ws/src
```
Clone this repo:
```
git clone https://github.com/SebastianGrans/Point-Cloud-Publisher.git
```
Compile:
```
colcon build --symlink-install --packages-select pcd_publisher
```


## Running
Source the installation:
```
cd ~/dev_ws/
source install/local_setup.sh # If you use bash
# _or_
source install/local_setup.zsh # If you use zsh
```
### Run the demo using a launch file (automatically starts RViz):
```
ros2 launch pcd_publisher pcd_publisher_demo.launch.py
```
RViz should now show a spinning Utah teapot! 

### Run the demo manually: 

```
ros2 run pcd_publisher pcd_publisher_node ~/dev_ws/src/pcd_publisher/resource/teapot.ply
```
In a new terminal:
```
ros2 run rviz2 rviz2
```
Make sure the `Displays` panel is visible `Panels > [✔] Displays`. Then, in the lower corner, press `Add` and select `PointCloud2`. In the list, expand `PointCloud2` and specify `pcd` as the topic. 

RViz should now show a spinning Utah teapot! 

