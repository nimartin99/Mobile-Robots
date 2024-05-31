# Script to display the points
import open3d as o3d
import numpy as np


if __name__ == "__main__":

    points = None # [x, y, z] coordinates in m

    pcd= o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd])
