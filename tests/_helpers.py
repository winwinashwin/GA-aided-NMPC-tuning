import math

from lib import nmpc
import numpy as np


def polyfit(ptsx, ptsy, order):
    assert len(ptsx) == len(ptsy)
    return np.polyfit(ptsx, ptsy, order)[::-1]


def generate_points():
    px = -8.0
    py = 1.5
    theta = -0.6

    ptsx, ptsy = [0.0] * 10, [0.0] * 10

    for i in range(10):
        ptsx[i] = px + i * 0.1

    for i in range(10):
        shift_x = ptsx[i] - px
        shift_y = ptsy[i] - py
        # transform
        ptsx[i] = shift_x * math.cos(-theta) - shift_y * math.sin(-theta)
        ptsy[i] = shift_x * math.sin(-theta) + shift_y * math.cos(-theta)

    ptsx_transform = ptsx[:6]
    ptsy_transform = ptsy[:6]

    return ptsx_transform, ptsy_transform


def get_params_and_coeffs():
    ps = nmpc.Params()
    ps.forward.timesteps = 12
    ps.forward.dt = 0.1

    ps.desired.vel = 0.5
    ps.desired.cte = 0.0
    ps.desired.etheta = 0.0

    ps.limits.omega.min = -2.0
    ps.limits.omega.max = 2.0
    ps.limits.throttle.min = -1.0
    ps.limits.throttle.max = 1.0

    ps.weights.cte = 87.859183
    ps.weights.etheta = 99.532785
    ps.weights.vel = 54.116644
    ps.weights.omega = 47.430096
    ps.weights.acc = 2.185306
    ps.weights.omega_d = 4.611500
    ps.weights.acc_d = 66.870729

    ptsx, ptsy = generate_points()
    coeffs = polyfit(ptsx, ptsy, 3)

    return ps, coeffs


def get_state():
    return nmpc.State(0, 0, 0, 0, 0, 0, -1.81744, -0.6)
