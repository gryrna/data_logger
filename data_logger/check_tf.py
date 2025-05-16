from tf_transformations import euler_from_quaternion
import numpy as np

q = [0.000000, 0.000017, 0.263967, 0.964532]
roll, pitch, yaw = euler_from_quaternion(q)
print(np.degrees(yaw))  # Should be around 15°, 30°, etc., not 180°
