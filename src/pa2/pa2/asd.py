import imageio.v3 as iio
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

pcd_o3d1 = o3d.io.read_point_cloud("/home/rvilab/ros2_material/src/pa2/data/result_depth2pcd.ply")
pcd_o3d2 = o3d.io.read_point_cloud("/home/rvilab/ros2_material/src/pa2/data/depth_2_pcd.ply")

#pcd_o3d1 = o3d.io.read_point_cloud("/home/rvilab/ros2_material/src/pa2/data/bunny_pcd_data.ply")
#pcd_o3d2 = o3d.io.read_point_cloud("/home/rvilab/ros2_material/src/pa2/data/bunny_pcd_output.ply")

#print(type(pcd_o3d1))
#print(type(pcd_o3d2))

#rint()
print(pcd_o3d1.has_colors()) #False 
print(pcd_o3d2.has_colors()) #True

print(pcd_o3d1.has_normals()) #False 
print(pcd_o3d2.has_normals()) #False

print(pcd_o3d1.get_geometry_type()) #Type.PointCloud
print(pcd_o3d2.get_geometry_type()) #Type.PointCloud







#o3d.visualization.draw_geometries([pcd_o3d1])
#o3d.visualization.draw_geometries([pcd_o3d2])



