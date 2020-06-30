
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
    print("what the fuck")
    print("args:" + str(args))

    pcd_publisher = PCDPublisher()
    

    rclpy.spin(pcd_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
