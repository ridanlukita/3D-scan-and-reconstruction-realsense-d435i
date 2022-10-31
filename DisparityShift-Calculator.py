from math import tan, radians

distance = int(input("Center distance from camera: "))
radius = int(input("Radius from center: "))
max_range = distance + radius

Xres = 848
HFOV = 87
focal_length = 0.5 *Xres/tan(radians(HFOV/2))
baseline = 50

disparity_shift = (focal_length* baseline) / max_range
print("Disparity shift = " , int(disparity_shift/10))