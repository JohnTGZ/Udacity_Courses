import numpy as np

def roll(theta):

    return np.matrix([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def pitch(theta):

    return np.matrix([
        [np.cos(theta),     0,  np.sin(theta)],
        [0,                 1,  0],
        [-np.sin(theta),    0,  np.cos(theta)]
    ])

def yaw(theta):

    return np.matrix([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta),  0],
        [0,             0,              1]
    ])

ans = roll(0) * pitch(0.2) * yaw(0.3)

print(ans)