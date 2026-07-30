import numpy as np

from magpick.utils.geometry import angle_between

print()

print("0 degrees")
print(angle_between([0,0,1],[0,0,1]))

print()

print("90 degrees")
print(angle_between([1,0,0],[0,1,0]))

print()

print("180 degrees")
print(angle_between([0,0,1],[0,0,-1]))