import numpy as np

with open('resource/camera_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

print("Camera Matrix : ", camera_matrix)
print("Distortion Matrix : ", camera_distortion)