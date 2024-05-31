# Script to display the points
import open3d as o3d
import numpy as np
import csv
import math


if __name__ == "__main__":
    points = [] # [x, y, z] coordinates in m

    with open('LIDAR_points.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in spamreader:
            coords = row[0].split(",")
            distance = float(coords[0])
            tilt = float(coords[1])
            azimuth = float(coords[2])
            
            x = distance * math.cos(azimuth) * math.cos(tilt)
            y = distance * math.sin(azimuth) * math.cos(tilt)
            z = distance * math.cos(tilt)
            points.append([x, y, z])

    pcd= o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd])
