import numpy as np

from magpick.utils.geometry import quaternion_to_approach

print("=" * 50)
print("Quaternion Test")
print("=" * 50)

# Identity quaternion (x, y, z, w)
q = np.array([0.0, 0.0, 0.0, 1.0])

approach = quaternion_to_approach(q)

print("Quaternion :", q)
print("Approach   :", approach)
print("Magnitude  :", np.linalg.norm(approach))