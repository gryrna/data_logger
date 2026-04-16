import numpy as np

for i in range(2128,2129):
    data = np.load(f'lidar_{i}.npy')
    print(f"lidar_{i}.npy")
    print(data)
    print("--------------------------------")