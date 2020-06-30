

# Visualizing a point cloud in RViz2 using Python 

While trying to learn how to use ROS2 and in extension RViz2 I discovered that there is a severe lack of documentation on how to use it with Python. Therefore I thought that I could write some ROS2 Python tutorials while I learn. In that way, you don't have to suffer the way I did ;) 


## Prerequisites 
I highly suggest that you first do the **Beginner: CLI Tools** tutorials on the [ROS2 website](https://index.ros.org/doc/ros2/Tutorials/). It will give you a basic understanding of the fundamentals of how ROS2 works.

My PhD supervisor Lars Tingelstad also has a [GitHub repo](https://github.com/tingelst/ros2_seminar_spring_2020_demos) of a simple ROS2 example/tutorial on how to use simulate a robot arm using RViz.


## Let's begin!

Assuming that you already have a [workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) with a `src` folder. In that folder, create a package by running:

```
ros2 pkg create --build-type ament_python --node-name pcd_publisher_node pcd_publisher
```

This will create the file `pcd_publisher/pcd_publisher_node.py` which is what we will be editing.

```python
import sys
import os

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
import open3d as o3d

class PCDPublisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher_node')

        assert len(sys.argv) > 1, "No ply file given."
        assert os.path.exists(sys.argv[1]), "File doesn't exist."
        pcd_path = sys.argv[1]
        pcd = o3d.io.read_point_cloud(pcd_path)
        # The up vector of bunny.ply is in the y-direction, while the up 
        # vector in RViz is the z-direction. 
        # For visualization purposes I rotate the whole point cloud.
        pcd.rotate(o3d.geometry.get_rotation_matrix_from_xyz([np.pi/2, 0, 0]), [0, 0, 0])
        points = np.asarray(pcd.points)


        self.points = np.asarray(pcd.points).astype(np.float32)
        self.pcd = None

        self.pcd_publisher_ = self.create_publisher(sensor_msgs.PointCloud2, 'pcd1', 10)
        timer_period = 1/30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi/48])

              
                
    def timer_callback(self):
        self.points = self.points @ self.R
        self.pcd1 = point_cloud(self.points, 'map')
        self.pcd_publisher_.publish(self.pcd1)

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions Note: Must be of dtype=np.float32
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

def main(args=None):
    rclpy.init(args=args)


    pcd_publisher = PCDPublisher()

    rclpy.spin(pcd_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rviz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


## Creating an RViz configuration





## Creating a launch file
[https://index.ros.org/doc/ros2/Tutorials/Launch-system/#python-packages]



## Building

Move to the root of the workspace, e.g. `cd ~/dev_ws`. Then compile the package using `colcon`.

```
colcon build --symlink-install --packages-select pcd_publisher
```

The purpose of `--symlink-install` is that it allows us to do changes to our Python code without having to recompile every time. 





https://answers.ros.org/question/197309/rviz-does-not-display-pointcloud2-if-encoding-not-float32/


