import numpy as np

for i in range(7,8):
    data = np.load(f'sensor_data/lidar_{i}.npy')
    print(f"lidar_{i}.npy")
    print(data)
    print("--------------------------------")