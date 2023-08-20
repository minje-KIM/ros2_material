#!/usr/bin/env python3
import imageio.v3 as iio
import rclpy
import open3d as o3d
import numpy as np
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from sensor_msgs.msg import PointCloud2
from rclpy.node import Node


class PointCloudPublisher(Node): 
    def __init__(self):
        super().__init__("pcd_publisher")
        #pcd_path = "/home/rvilab/nyu/dep.ply"
        
        pcd_path = "/home/rvilab/ros2_material/src/pa2/data/result_depth2pcd.ply"
        pcd = o3d.io.read_point_cloud(pcd_path)
        pcd = pcd.random_down_sample(sampling_ratio=0.005)
        #pcd = depth2pcd()
        #o3d.visualization.draw_geometries([pcd])
        #print(type(pcd))
        self.np_pcd = np.asarray(pcd.points)
        #self.np_pcd = depth2pcd()
        print(self.np_pcd.shape)

        self.R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi/48])
        self.publisher_ = self.create_publisher(PointCloud2, "sanghyeon_pcd", 10)
        self.timer_ = self.create_timer(1.0, self.publish_callback)

    def publish_callback(self):
        self.np_pcd = self.np_pcd @ self.R
        self.msg_pcd = convert_msgpcd(self.np_pcd, 'map')
        self.publisher_.publish(self.msg_pcd)


def depth2pcd():

    # Camera parameters:
    FX_DEPTH = 5.8262448167737955e+02
    FY_DEPTH = 5.8269103270988637e+02
    CX_DEPTH = 3.1304475870804731e+02
    CY_DEPTH = 2.3844389626620386e+02

    depth_image = iio.imread('/home/rvilab/nyu/depth/depth11.png')
    # get depth image resolution:
    height, width = depth_image.shape
    # compute indices:
    jj = np.tile(range(width), height)
    ii = np.repeat(range(height), width)
    # Compute constants:
    xx = (jj - CX_DEPTH) / FX_DEPTH
    yy = (ii - CY_DEPTH) / FY_DEPTH
    # transform depth image to vector of z:
    length = height * width
    z = depth_image.reshape(length)
    # compute point cloud
    pcd = np.dstack((xx * z, yy * z, z)).reshape((length, 3))

    # Convert to Open3D.PointCLoud:
    pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
    pcd_o3d.points = o3d.utility.Vector3dVector(pcd)  # set pcd_np as the point cloud points
    o3d.io.write_point_cloud("/home/rvilab/ros2_material/src/pa2/data/result_depth2pcd.ply", pcd_o3d)
    # Visualize:
    #o3d.visualization.draw_geometries([pcd_o3d])
    return pcd_o3d


def convert_msgpcd(np_pcd, origin_frame):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = np_pcd.astype(dtype).tobytes() 

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]
    
    header = std_msgs.Header(frame_id=origin_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=np_pcd.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * np_pcd.shape[0]),
        data=data
    )

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()