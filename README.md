# ROS2 Point Cloud

![spinning pointcloud of Utah Teapot](demo.gif)

This is an example ROS2 (python) package which demonstrates how to utilize the `sensor_msg.msg.PointCloud2`. The various scripts show how to publish a point cloud represented by a `numpy` array as a `PointCloud2` message, and vice versa. I also demonstrate how to visualize a point cloud in RViz2.

**2025-04-19: This is now updated to work on ROS2 Rolling**

I install ROS2 rolling on Ubuntu 24.10 (oracular).

## Installation

This assumes that you have ROS2 Rolling installed and have some basic knowledge of ROS2.
Check out the [ROS2 Rolling Documentation](https://docs.ros.org/en/rolling/index.html).

Move to your ROS workspace source folder:

```bash
cd ~/dev_ws
```

Clone this repo:

```bash
git clone https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo.git
```

Set up a virtual environment:

```bash
# Do this in the root of your workspace
cd ~/dev_ws/
# Source the underlay
source /opt/ros/rolling/source.bash
# Create the virtual environment
python3 -m venv venv
# Add a `COLCON_IGNORE` file
touch venv/COLCON_IGNORE
# Activate the environment
source venv/bin/activate
# Install the python package
pip install ./ROS2-Point-Cloud-Demo
```

Add the environment to the `$PYTHONPATH` environment variable:

**Note:** Replace `<your-username-here>`

```bash
export PYTHONPATH="${PYTHONPATH}:/home/<your-username-here>/dev_ws/venv/lib/python3.12/site-packages)"
```

Compile with `colcon`:

```bash
colcon build --symlink-install --packages-select pcd_demo
```

## Running

Source the installation:

```bash
cd ~/dev_ws/
source install/local_setup.bash
```

### Publisher demo

#### Using a launch file (automatically starts RViz)

```bash
ros2 launch pcd_demo pcd_publisher_demo.launch.py
```

RViz should now show a spinning Utah teapot!

#### Manually

```bash
ros2 run pcd_demo pcd_publisher_node ~/dev_ws/src/ROS2-Point-Cloud-Demo/resource/teapot.ply
```

In a new terminal:

```bash
ros2 run rviz2 rviz2
```

Make sure the `Displays` panel is visible `Panels > [✔] Displays`. Then, in the lower corner, press `Add` and select `PointCloud2`. In the list, expand `PointCloud2` and specify `pcd` as the topic.

RViz should now show a spinning Utah teapot!

### Subscriber demo

**This seems to be broken!**

This demo is similar to the one above, but rather than relying on RViz, we instead use [Open3D](http://www.open3d.org/) for visualization.

**Note:** This is not yet fully optimized. It needs to be threaded such that the Open3D interface is responsive.

#### Running from launch file

**This seems to be broken!**

```bash
ros2 launch pcd_demo pcd_pubsub_demo.launch.py
```

#### Manually

**This seems to be broken!**

In one terminal, run the publisher:

```bash
ros2 run pcd_demo pcd_publisher_node ~/dev_ws/src/pcd_publisher/resource/teapot.ply
```

and in another terminal, run the subscriber node:

```bash
ros2 run pcd_demo pcd_subscriber_node
```
