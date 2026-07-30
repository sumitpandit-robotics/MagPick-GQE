import numpy as np

from magpick.utils.geometry import *

pcd = load_point_cloud("datasets/scene.ply")

print()

print("Loaded")

print("Points :", len(pcd.points))

points = np.asarray(pcd.points)

print("\nPoint Cloud Statistics")
print("----------------------")
print("Min    :", points.min(axis=0))
print("Max    :", points.max(axis=0))
print("Center :", points.mean(axis=0))
print("Extent :", points.max(axis=0) - points.min(axis=0))
estimate_normals(pcd)

print("Normals estimated")

center = np.asarray(pcd.points)[0]

crop = crop_sphere(

    pcd,

    center,

    radius=0.04,

)

print("Crop size :", len(crop.points))

idx = nearest_point(

    pcd,

    center,

)

print("Nearest index :", idx)

normal = compute_normal(

    pcd,

    idx,

)

print("Normal :", normal)