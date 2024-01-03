import numpy as np
import matplotlib.pyplot as plt
import math
import numpy as np
from scipy.interpolate import interp1d
import pytransform3d.rotations as pr
import pytransform3d.batch_rotations as pbr
import pytransform3d.transformations as pt
from sympy import symbols, Eq, solve

EPSILON = 1e-10

def find_intersection(circle_center, circle_radius, line_point1):
    # Define symbolic variables
    x, y = symbols('x y')

    # Circle equation: (x - x_c)^2 + (y - y_c)^2 = r^2
    circle_eq = Eq((x - circle_center[0])**2 + (y - circle_center[1])**2, circle_radius**2)

    # Line equation: y = mx + b
    m = (circle_center[1] - line_point1[1]) / (circle_center[0] - line_point1[0])
    b = line_point1[1] - m * line_point1[0]
    line_eq = Eq(y, m*x + b)

    # Solve the system of equations to find intersection points
    intersection_points = solve([circle_eq, line_eq], (x, y))
    
    # Filter out points outside the line segment
    valid_intersection_points = []
    for point in intersection_points:
        x_val, y_val = point
        if (min(line_point1[0], circle_center[0]) <= x_val <= max(line_point1[0], circle_center[0]) and
                min(line_point1[1], circle_center[1]) <= y_val <= max(line_point1[1], circle_center[1])):
            valid_intersection_points.append(point)
    return np.array(list(valid_intersection_points[0]),dtype=np.float32)

def obstacle_avoidance_acceleration_2d_v0(
        y, yd, obstacle_position, gamma=1000.0, beta=20.0 / math.pi):
    obstacle_diff = obstacle_position - y
    pad_width = ([[0, 0]] * (y.ndim - 1)) + [[0, 1]]
    obstacle_diff_0 = np.pad(obstacle_diff, pad_width, mode="constant",
                             constant_values=0.0)
    yd_0 = np.pad(yd, pad_width, mode="constant", constant_values=0.0)
    r = 0.5 * np.pi * pbr.norm_vectors(np.cross(obstacle_diff_0, yd_0))
    R = pbr.matrices_from_compact_axis_angles(r)[..., :2, :2]
    theta_nom = np.einsum(
        "ni,ni->n", obstacle_diff.reshape(-1, 2), yd.reshape(-1, 2))
    shape = y.shape[:-1]
    if shape:
        theta_nom = theta_nom.reshape(*shape)
    theta_denom = (np.linalg.norm(obstacle_diff, axis=-1)
                   * np.linalg.norm(yd, axis=-1) + EPSILON)
    theta = np.arccos(theta_nom / theta_denom)
    rotated_velocity = np.einsum(
        "nij,nj->ni", R.reshape(-1, 2, 2), yd.reshape(-1, 2))
    if shape:
        rotated_velocity = rotated_velocity.reshape(*(shape + (2,)))
    cdd = (gamma * rotated_velocity
           * (theta * np.exp(-beta * theta))[..., np.newaxis])
    return np.squeeze(cdd), theta[0]

def obstacle_avoidance_acceleration_2d_v1(
        y, yd, obstacle_position, gamma=1000.0, beta=20.0 / math.pi, k=1000.0):
    
    obstacle_diff = obstacle_position - y
    pad_width = ([[0, 0]] * (y.ndim - 1)) + [[0, 1]]
    obstacle_diff_0 = np.pad(obstacle_diff, pad_width, mode="constant",
                             constant_values=0.0)
    yd_0 = np.pad(yd, pad_width, mode="constant", constant_values=0.0)
    r = 0.5 * np.pi * pbr.norm_vectors(np.cross(obstacle_diff_0, yd_0))
    R = pbr.matrices_from_compact_axis_angles(r)[..., :2, :2]
    theta_nom = np.einsum(
        "ni,ni->n", obstacle_diff.reshape(-1, 2), yd.reshape(-1, 2))
    shape = y.shape[:-1]
    if shape:
        theta_nom = theta_nom.reshape(*shape)
    theta_denom = (np.linalg.norm(obstacle_diff, axis=-1)
                   * np.linalg.norm(yd, axis=-1) + EPSILON)
    theta = np.arccos(theta_nom / theta_denom)
    rotated_velocity = np.einsum(
        "nij,nj->ni", R.reshape(-1, 2, 2), yd.reshape(-1, 2))
    if shape:
        rotated_velocity = rotated_velocity.reshape(*(shape + (2,)))
    cdd = (gamma * rotated_velocity
           * (theta * np.exp(-beta * theta)*np.exp(-k * np.linalg.norm(obstacle_diff, axis=-1)))[..., np.newaxis])
    return np.squeeze(cdd), theta[0]

def obstacle_avoidance_acceleration_2d_v2(
        y, yd, obstacle_position, intersec_point, gamma=[1000.0,1000.0], beta=[20.0 / math.pi,20.0 / math.pi], k=[1000.0,1000.0]):
    
    obstacle_diff = obstacle_position - y

    pad_width = ([[0, 0]] * (y.ndim - 1)) + [[0, 1]]
    obstacle_diff_0 = np.pad(obstacle_diff, pad_width, mode="constant",
                             constant_values=0.0)
    yd_0 = np.pad(yd, pad_width, mode="constant", constant_values=0.0)
    r = 0.5 * np.pi * pbr.norm_vectors(np.cross(obstacle_diff_0, yd_0))
    R = pbr.matrices_from_compact_axis_angles(r)[..., :2, :2]
    theta_nom = np.einsum(
        "ni,ni->n", obstacle_diff.reshape(-1, 2), yd.reshape(-1, 2))
    shape = y.shape[:-1]
    if shape:
        theta_nom = theta_nom.reshape(*shape)
    theta_denom = (np.linalg.norm(obstacle_diff, axis=-1)
                   * np.linalg.norm(yd, axis=-1) + EPSILON)
    theta = np.arccos(theta_nom / theta_denom)
    rotated_velocity = np.einsum(
        "nij,nj->ni", R.reshape(-1, 2, 2), yd.reshape(-1, 2))
    if shape:
        rotated_velocity = rotated_velocity.reshape(*(shape + (2,)))
    cdd = (gamma[0] * rotated_velocity
           * (theta * np.exp(-beta[0] * theta)*np.exp(-k[0] * np.linalg.norm(obstacle_diff, axis=-1)))[..., np.newaxis])
    
    intersec_point_diff = intersec_point - y
    cdd += (gamma[1] * rotated_velocity
           * (theta * np.exp(-beta[1] * theta)*np.exp(-k[1] * np.linalg.norm(intersec_point_diff, axis=-1)))[..., np.newaxis])
    
    return np.squeeze(cdd), theta[0]

def obstacle_avoidance_acceleration_2d_v3(
        y, yd, obstacle_position, intersec_point, gamma=[1000.0,1000.0], beta=[20.0 / math.pi,20.0 / math.pi], k=[1000.0,1000.0]):
    
    obstacle_diff = obstacle_position - y

    pad_width = ([[0, 0]] * (y.ndim - 1)) + [[0, 1]]
    obstacle_diff_0 = np.pad(obstacle_diff, pad_width, mode="constant",
                             constant_values=0.0)
    yd_0 = np.pad(yd, pad_width, mode="constant", constant_values=0.0)
    r = 0.5 * np.pi * pbr.norm_vectors(np.cross(obstacle_diff_0, yd_0))
    R = pbr.matrices_from_compact_axis_angles(r)[..., :2, :2]
    theta_nom = np.einsum(
        "ni,ni->n", obstacle_diff.reshape(-1, 2), yd.reshape(-1, 2))
    shape = y.shape[:-1]
    if shape:
        theta_nom = theta_nom.reshape(*shape)
    theta_denom = (np.linalg.norm(obstacle_diff, axis=-1)
                   * np.linalg.norm(yd, axis=-1) + EPSILON)
    theta = np.arccos(theta_nom / theta_denom)
    rotated_velocity = np.einsum(
        "nij,nj->ni", R.reshape(-1, 2, 2), yd.reshape(-1, 2))
    if shape:
        rotated_velocity = rotated_velocity.reshape(*(shape + (2,)))
    cdd = (gamma[0] * rotated_velocity
           * (theta * np.exp(-beta[0] * theta)*np.exp(-k[0] * np.linalg.norm(obstacle_diff, axis=-1)))[..., np.newaxis])
    
    intersec_point_diff = intersec_point - y
    cdd += (gamma[1] * rotated_velocity
           * (theta * np.exp(-beta[1] * theta)*np.exp(-k[1] * np.linalg.norm(intersec_point_diff, axis=-1)))[..., np.newaxis])
    
    cdd += (gamma[2] * rotated_velocity
           * (theta *np.exp(-k[2] * np.linalg.norm(intersec_point_diff, axis=-1)))[..., np.newaxis])
    
    return np.squeeze(cdd), theta[0]


if __name__ == "__main__":
    eef_y = np.array([0.3,0.2])
    eef_yd = np.array([0.077,0.289])
    obstacle = np.array([0.7, 0.7])
    obstacle_radius = 0.2

    version = 1 #0,1,2,3

    x_range = 0, 1.0
    y_range = 0, 1.0

    fig = plt.figure(figsize=(21, 8))
    markersize=15
    fontsize=20

    ax = plt.subplot(1,1, 1, aspect="equal")

    intersec_point = find_intersection(obstacle, obstacle_radius, eef_y)
    circle = plt.Circle(obstacle, obstacle_radius, color='tab:red', fill=True, alpha=0.5, label="Obstacle volume")
    ax.add_patch(circle)
    ax.plot(intersec_point[0], intersec_point[1], "o", color="tab:purple", markersize=markersize, label="Instersection point (m)")

    if version==0:
        ct, theta = obstacle_avoidance_acceleration_2d_v0(eef_y, eef_yd, obstacle, gamma=10.0, beta=20.0 / math.pi) 
    elif version==1:
        ct, theta = obstacle_avoidance_acceleration_2d_v1(eef_y, eef_yd, obstacle, gamma=10.0, beta=10.0 / math.pi, k=5.0 / math.pi)
    elif version==2: 
        ct, theta = obstacle_avoidance_acceleration_2d_v2(eef_y, eef_yd, obstacle, intersec_point, gamma=[5.0,15.0], beta=[10.0/math.pi,10.0/math.pi], k=[5.0/math.pi,10.0/math.pi])
    elif version==3:
        ct, theta = obstacle_avoidance_acceleration_2d_v3(eef_y, eef_yd, obstacle, intersec_point, gamma=[5.0,15.0,2.0], beta=[10.0/math.pi,10.0/math.pi], k=[5.0/math.pi,10.0/math.pi,20.0/math.pi])

    ax.plot([obstacle[0],eef_y[0]], [obstacle[1],eef_y[1]], color="black", linewidth=3)
    ax.plot(obstacle[0], obstacle[1], "o", color="tab:red", markersize=markersize, label="Obstacle position (m)")
    ax.plot(eef_y[0], eef_y[1], "o", color="tab:blue", markersize=markersize, label="End-effector position (m)")
    ax.quiver(eef_y[0], eef_y[1], eef_yd[0], eef_yd[1], angles='xy', scale_units='xy', scale=1, width=0.01, color='tab:green', label="End-effector velocity (m/s)")
    ax.quiver(eef_y[0], eef_y[1], ct[0], ct[1], angles='xy', scale_units='xy', scale=1, width=0.01, color='tab:orange', label="Coupling acceleration (m/s²)")

    ax.set_title('DMP + APF v'+str(version), fontsize=fontsize)

    ax.tick_params(labelsize=fontsize*0.75)
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.set_xlabel("X",fontsize=fontsize)
    ax.set_ylabel("Y",fontsize=fontsize)
    ax.legend(fontsize=fontsize*0.75, loc='center left', bbox_to_anchor=(1, 0.5))
    fig.tight_layout()

    plt.show()
