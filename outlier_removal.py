import open3d as o3d
import numpy as np
import os
import sys

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    # inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])#,
    o3d.visualization.draw_geometries([inlier_cloud])#,
                                    #   zoom=0.3412,
                                    #   front=[0.4257, -0.2125, -0.8795],
                                    #   lookat=[2.6172, 2.0475, 1.532],
                                    #   up=[-0.0694, -0.9768, 0.2024])
    return inlier_cloud



def remove_pointcloud(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    
    # inlier_cloud = inlier_cloud - outlier_cloud
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


pcd = o3d.io.read_point_cloud("dataset/scene/integrated.ply")
o3d.visualization.draw_geometries([pcd])
# pcd = pcd.voxel_down_sample(voxel_size=0.01)
# pcd = pcd.uniform_down_sample(every_k_points=5)

# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# print("Downsample the point cloud with a voxel of 0.02")
# pcd = pcd.voxel_down_sample(voxel_size=0.02)
# o3d.visualization.draw_geometries([pcd])

# o3d.visualization.draw_geometries([voxel_down_pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# mesh = o3d.io.read_triangle_mesh("dataset/scene/integrated.ply")
# o3d.visualization.draw_geometries([mesh])
print("Radius oulier removal")
cl, ind = pcd.remove_radius_outlier(nb_points=50, radius=0.02)

print("Statistical oulier removal")
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=1000, std_ratio=0.00000000000000000000000000000000000000000000000000001)

# inlier = display_inlier_outlier(pcd, ind)

inlier = display_inlier_outlier(pcd, ind)

# print("Statistical oulier removal")
# cl, ind = inlier.remove_statistical_outlier(nb_neighbors=100, std_ratio=0.00000000000000000000000000000000000000000000000000001)
# display_inlier_outlier(inlier, ind)



# mu, sigma = 0, 0.1  # mean and standard deviation
# threshold = 1.0
# print("Robust point-to-plane ICP, threshold={}:".format(threshold))
# loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
# print("Using robust loss:", loss)
# p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
# reg_p2l = o3d.pipelines.registration.registration_icp(source_noisy, target,
#                                                       threshold, trans_init,
#                                                       p2l)
# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
# draw_registration_result(source, target, reg_p2l.transformation)