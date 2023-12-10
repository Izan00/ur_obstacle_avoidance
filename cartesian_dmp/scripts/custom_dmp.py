#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import DisplayTrajectory
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Path
from std_msgs.msg import Header
import copy

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pytransform3d import rotations as pr
import math
import abc
#from ur_sim import UR5Simulation

def canonical_system_alpha(goal_z, goal_t, start_t, int_dt=0.001):
    if goal_z <= 0.0:
        raise ValueError("Final phase must be > 0!")
    if start_t >= goal_t:
        raise ValueError("Goal must be chronologically after start!")

    execution_time = goal_t - start_t
    n_phases = int(execution_time / int_dt) + 1
    # assert that the execution_time is approximately divisible by int_dt
    assert abs(((n_phases - 1) * int_dt) - execution_time) < 0.05
    return (1.0 - goal_z ** (1.0 / (n_phases - 1))) * (n_phases - 1)


def phase(t, alpha, goal_t, start_t, int_dt=0.001, eps=1e-10):
    execution_time = goal_t - start_t
    b = max(1.0 - alpha * int_dt / execution_time, eps)
    return b ** ((t - start_t) / int_dt)


class ForcingTerm:
    def __init__(self, n_dims, n_weights_per_dim, goal_t, start_t, overlap,
                 alpha_z):
        if n_weights_per_dim <= 1:
            raise ValueError("The number of weights per dimension must be > 1!")
        self.n_weights_per_dim = n_weights_per_dim
        if start_t >= goal_t:
            raise ValueError("Goal must be chronologically after start!")
        self.goal_t = goal_t
        self.start_t = start_t
        self.overlap = overlap
        self.alpha_z = alpha_z

        self._init_rbfs(n_dims, n_weights_per_dim, start_t)

    def _init_rbfs(self, n_dims, n_weights_per_dim, start_t):
        self.log_overlap = float(-math.log(self.overlap))
        self.execution_time = self.goal_t - self.start_t
        self.weights_ = np.zeros((n_dims, n_weights_per_dim))
        self.centers = np.empty(n_weights_per_dim)
        self.widths = np.empty(n_weights_per_dim)
        # -1 because we want the last entry to be execution_time
        step = self.execution_time / (self.n_weights_per_dim - 1)
        # do first iteration outside loop because we need access to i and i - 1 in loop
        t = start_t
        self.centers[0] = phase(t, self.alpha_z, self.goal_t, self.start_t)
        for i in range(1, self.n_weights_per_dim):
            # normally lower_border + i * step but lower_border is 0
            t = i * step
            self.centers[i] = phase(t, self.alpha_z, self.goal_t, self.start_t)
            # Choose width of RBF basis functions automatically so that the
            # RBF centered at one center has value overlap at the next center
            diff = self.centers[i] - self.centers[i - 1]
            self.widths[i - 1] = self.log_overlap / diff ** 2
        # Width of last Gaussian cannot be calculated, just use the same width
        # as the one before
        self.widths[self.n_weights_per_dim - 1] = self.widths[
            self.n_weights_per_dim - 2]

    def _activations(self, z):
        z = np.atleast_2d(z)  # 1 x n_steps
        squared_dist = (z - self.centers[:, np.newaxis]) ** 2
        activations = np.exp(-self.widths[:, np.newaxis] * squared_dist)
        activations /= activations.sum(axis=0)  # normalize
        return activations

    def design_matrix(self, T, int_dt=0.001):  # returns: n_weights_per_dim x n_steps
        Z = phase(T, alpha=self.alpha_z, goal_t=T[-1], start_t=T[0],
                  int_dt=int_dt)
        return Z[np.newaxis, :] * self._activations(Z)

    def phase(self, t, int_dt=0.001):
        return phase(t, alpha=self.alpha_z, goal_t=self.goal_t,
                     start_t=self.start_t, int_dt=int_dt)

    def forcing_term(self, z):
        
        z = np.atleast_1d(z)
        activations = self._activations(z)
        return z[np.newaxis, :] * self.weights_.dot(activations)

    def __call__(self, t, int_dt=0.001):
        return self.forcing_term(self.phase(t, int_dt))

    @property
    def shape(self):
        """Shape (n_dims, n_weights_per_dim) of weights configuring the forcing term."""
        return self.weights_.shape


class PointToPointMovement(abc.ABC):
    def __init__(self, n_pos_dims, n_vel_dims):
        self.n_dims = n_pos_dims
        self.n_vel_dims = n_vel_dims

        self.t = 0.0
        self.last_t = None

        self.start_y = np.zeros(n_pos_dims)
        self.start_yd = np.zeros(n_vel_dims)
        self.start_ydd = np.zeros(n_vel_dims)

        self.goal_y = np.zeros(n_pos_dims)
        self.goal_yd = np.zeros(n_vel_dims)
        self.goal_ydd = np.zeros(n_vel_dims)

        self.current_y = np.zeros(n_pos_dims)
        self.current_yd = np.zeros(n_vel_dims)

    def configure(
            self, t=None, start_y=None, start_yd=None, start_ydd=None,
            goal_y=None, goal_yd=None, goal_ydd=None):
   
        if t is not None:
            self.t = t
        if start_y is not None:
            assert len(start_y) == self.n_dims, f"{len(start_y)}"
            self.start_y = start_y
        if start_yd is not None:
            assert len(start_yd) == self.n_vel_dims, f"{len(start_yd)}"
            self.start_yd = start_yd
        if start_ydd is not None:
            assert len(start_ydd) == self.n_vel_dims, f"{len(start_ydd)}"
            self.start_ydd = start_ydd
        if goal_y is not None:
            assert len(goal_y) == self.n_dims, f"{len(goal_y)}"
            self.goal_y = goal_y
        if goal_yd is not None:
            assert len(goal_yd) == self.n_vel_dims, f"{len(goal_yd)}"
            self.goal_yd = goal_yd
        if goal_ydd is not None:
            assert len(goal_ydd) == self.n_vel_dims, f"{len(goal_ydd)}"
            self.goal_ydd = goal_ydd

    @abc.abstractmethod
    def step(self, last_y, last_yd):
        """"""
    def n_steps_open_loop(self, last_y, last_yd, n_steps):
        for _ in range(n_steps):
            last_y, last_yd = self.step(last_y, last_yd)
        return last_y, last_yd


class DMPBase(PointToPointMovement):
   
    def __init__(self, n_pos_dims, n_vel_dims):
        super(DMPBase, self).__init__(n_pos_dims, n_vel_dims)
        self.initialized = False

    def reset(self):
        self.t = 0.0
        self.last_t = None
        self.current_y = np.copy(self.start_y)
        self.current_yd = np.copy(self.start_yd)


class WeightParametersMixin:
    def get_weights(self):
        return self.forcing_term.weights_.ravel()

    def set_weights(self, weights):
        self.forcing_term.weights_[:, :] = weights.reshape(*self.forcing_term.shape)

    @property
    def n_weights(self):
        return np.product(self.forcing_term.shape)


class CartesianDMP(DMPBase):
    
    def __init__(
            self, execution_time=1.0, dt=0.01, n_weights_per_dim=10,
            int_dt=0.001, smooth_scaling=False):
        super(CartesianDMP, self).__init__(7, 6)
        self._execution_time = execution_time
        self.dt_ = dt
        self.n_weights_per_dim = n_weights_per_dim
        self.int_dt = int_dt
        self.smooth_scaling = smooth_scaling

        self._init_forcing_term()

        self.alpha_y = 25.0
        self.beta_y = self.alpha_y / 4.0

    def _init_forcing_term(self):
        alpha_z = canonical_system_alpha(
            0.01, self.execution_time_, 0.0, self.int_dt)
        self.forcing_term_pos = ForcingTerm(
            3, self.n_weights_per_dim, self.execution_time_, 0.0, 0.8,
            alpha_z)
        self.forcing_term_rot = ForcingTerm(
            3, self.n_weights_per_dim, self.execution_time_, 0.0, 0.8,
            alpha_z)

    def get_execution_time_(self):
        return self._execution_time

    def set_execution_time_(self, execution_time):
        self._execution_time = execution_time
        weights_pos = self.forcing_term_pos.weights_
        weights_rot = self.forcing_term_rot.weights_
        self._init_forcing_term()
        self.forcing_term_pos.weights_ = weights_pos
        self.forcing_term_rot.weights_ = weights_rot

    execution_time_ = property(get_execution_time_, set_execution_time_)

    def dmp_imitate(self,
            T, Y, n_weights_per_dim, regularization_coefficient, alpha_y, beta_y,
            overlap, alpha_z, allow_final_velocity,
            determine_forces=None, smooth_scaling=False):
        
        if determine_forces == None:
            determine_forces = self.determine_forces

        if regularization_coefficient < 0.0:
            raise ValueError("Regularization coefficient must be >= 0!")

        forcing_term = ForcingTerm(
            Y.shape[1], n_weights_per_dim, T[-1], T[0], overlap, alpha_z)
        F, start_y, start_yd, start_ydd, goal_y, goal_yd, goal_ydd = determine_forces(
            T, Y, alpha_y, beta_y, alpha_z, allow_final_velocity, smooth_scaling)
        # F shape (n_steps, n_dims)

        X = forcing_term.design_matrix(T)  # shape (n_weights_per_dim, n_steps)

        return (self.ridge_regression(X, F, regularization_coefficient),
                start_y, start_yd, start_ydd, goal_y, goal_yd, goal_ydd)


    def _dmp_acc(self, Y, V, cdd, alpha_y, beta_y, goal_y, goal_yd, goal_ydd,
                start_y, z, execution_time, f, coupling_term, tdd,
                smooth_scaling):
        if coupling_term is not None:
            _, cdd = coupling_term.coupling(Y, V)
        if smooth_scaling:
            smoothing = beta_y * (goal_y - start_y) * z
        else:
            smoothing = 0.0
        return (
            alpha_y * (
                beta_y * (goal_y - Y)
                - execution_time * V
                - smoothing
            )
            + f
            + cdd
            + tdd
        ) / execution_time ** 2

    def ridge_regression(self, X, Y, regularization_coefficient):
        return np.linalg.pinv(
            X.dot(X.T) + regularization_coefficient * np.eye(X.shape[0])
        ).dot(X).dot(Y).T

    def dmp_step_quaternion_python(self,
            last_t, t,
            current_y, current_yd,
            goal_y, goal_yd, goal_ydd,
            start_y, start_yd, start_ydd,
            goal_t, start_t, alpha_y, beta_y,
            forcing_term,
            coupling_term=None,
            coupling_term_precomputed=None,
            int_dt=0.001,
            smooth_scaling=False):
    
        if start_t >= goal_t:
            raise ValueError("Goal must be chronologically after start!")

        if t <= start_t:
            return np.copy(start_y), np.copy(start_yd), np.copy(start_ydd)

        execution_time = goal_t - start_t

        current_ydd = np.empty_like(current_yd)

        current_t = last_t
        while current_t < t:
            dt = int_dt
            if t - current_t < int_dt:
                dt = t - current_t
            current_t += dt

            if coupling_term is not None:
                cd, cdd = coupling_term.coupling(current_y, current_yd)
            else:
                cd, cdd = np.zeros(3), np.zeros(3)
            if coupling_term_precomputed is not None:
                cd += coupling_term_precomputed[0]
                cdd += coupling_term_precomputed[1]

            z = forcing_term.phase(current_t, int_dt)
            f = forcing_term.forcing_term(z).squeeze()

            if smooth_scaling:
                goal_y_minus_start_y = pr.compact_axis_angle_from_quaternion(
                    pr.concatenate_quaternions(goal_y, pr.q_conj(start_y)))
                smoothing = beta_y * z * goal_y_minus_start_y
            else:
                smoothing = 0.0

            current_ydd[:] = (
                alpha_y * (
                    beta_y * pr.compact_axis_angle_from_quaternion(pr.concatenate_quaternions(goal_y, pr.q_conj(current_y)))
                    - execution_time * current_yd
                    - smoothing
                )
                + f
                + cdd
            ) / execution_time ** 2
            current_yd += dt * current_ydd + cd / execution_time
            current_y[:] = pr.concatenate_quaternions(
                pr.quaternion_from_compact_axis_angle(dt * current_yd), current_y)

    def dmp_step_rk4(self,
            last_t, t, current_y, current_yd, goal_y, goal_yd, goal_ydd, start_y,
            start_yd, start_ydd, goal_t, start_t, alpha_y, beta_y, forcing_term,
            coupling_term=None, coupling_term_precomputed=None, int_dt=0.001,
            p_gain=0.0, tracking_error=0.0, smooth_scaling=False):

        if coupling_term is None:
            cd, cdd = np.zeros_like(current_y), np.zeros_like(current_y)
            if coupling_term_precomputed is not None:
                cd += coupling_term_precomputed[0]
                cdd += coupling_term_precomputed[1]
        else:
            cd, cdd = None, None  # will be computed in _dmp_acc()

        # RK4 (Runge-Kutta) for 2nd order differential integration
        # (faster and more accurate than Euler integration),
        # implemented following https://math.stackexchange.com/a/2023862/64116

        # precompute constants for following queries
        execution_time = goal_t - start_t
        dt = t - last_t
        dt_2 = 0.5 * dt
        T = np.array([t, t + dt_2, t + dt])
        Z = forcing_term.phase(T, int_dt=int_dt)
        F = forcing_term.forcing_term(Z)
        tdd = p_gain * tracking_error / dt

        C0 = current_yd
        K0 = self._dmp_acc(
            current_y, C0, cdd, alpha_y, beta_y, goal_y, goal_yd, goal_ydd,
            start_y, Z[0], execution_time, F[:, 0], coupling_term, tdd,
            smooth_scaling)
        C1 = current_yd + dt_2 * K0
        K1 = self._dmp_acc(
            current_y + dt_2 * C0, C1, cdd, alpha_y, beta_y, goal_y, goal_yd,
            goal_ydd, start_y, Z[1], execution_time, F[:, 1], coupling_term, tdd,
            smooth_scaling)
        C2 = current_yd + dt_2 * K1
        K2 = self._dmp_acc(
            current_y + dt_2 * C1, C2, cdd, alpha_y, beta_y, goal_y, goal_yd,
            goal_ydd, start_y, Z[1], execution_time, F[:, 1], coupling_term, tdd,
            smooth_scaling)
        C3 = current_yd + dt * K2
        K3 = self._dmp_acc(
            current_y + dt * C2, C3, cdd, alpha_y, beta_y, goal_y, goal_yd,
            goal_ydd, start_y, Z[2], execution_time, F[:, 2], coupling_term, tdd,
            smooth_scaling)

        current_y += dt * (current_yd + dt / 6.0 * (K0 + K1 + K2))
        current_yd += dt / 6.0 * (K0 + 2 * K1 + 2 * K2 + K3)

        if coupling_term is not None:
            cd, _ = coupling_term.coupling(current_y, current_yd)
            current_yd += cd / execution_time

    def step(self, last_y, last_yd, coupling_term=None,
             step_function=dmp_step_rk4,
             quaternion_step_function=dmp_step_quaternion_python):

        assert len(last_y) == 7
        assert len(last_yd) == 6

        self.last_t = self.t
        self.t += self.dt_

        # TODO tracking error
        
        self.current_y[:], self.current_yd[:] = last_y, last_yd
        step_function(self,
            self.last_t, self.t,
            self.current_y[:3], self.current_yd[:3],
            self.goal_y[:3], self.goal_yd[:3], self.goal_ydd[:3],
            self.start_y[:3], self.start_yd[:3], self.start_ydd[:3],
            self.execution_time_, 0.0,
            self.alpha_y, self.beta_y,
            self.forcing_term_pos,
            coupling_term=coupling_term, #TODO rev
            int_dt=self.int_dt,
            smooth_scaling=self.smooth_scaling)
        
        quaternion_step_function(self,
            self.last_t, self.t,
            self.current_y[3:], self.current_yd[3:],
            self.goal_y[3:], self.goal_yd[3:], self.goal_ydd[3:],
            self.start_y[3:], self.start_yd[3:], self.start_ydd[3:],
            self.execution_time_, 0.0,
            self.alpha_y, self.beta_y,
            self.forcing_term_rot,
            coupling_term=None, #TODO rev
            int_dt=self.int_dt,
            smooth_scaling=self.smooth_scaling)
        return np.copy(self.current_y), np.copy(self.current_yd)

    def imitate(self, T, Y, regularization_coefficient=0.0,
                allow_final_velocity=False):

        self.forcing_term_pos.weights_[:, :] = self.dmp_imitate(
            T, Y[:, :3],
            n_weights_per_dim=self.n_weights_per_dim,
            regularization_coefficient=regularization_coefficient,
            alpha_y=self.alpha_y, beta_y=self.beta_y,
            overlap=self.forcing_term_pos.overlap,
            alpha_z=self.forcing_term_pos.alpha_z,
            allow_final_velocity=allow_final_velocity,
            smooth_scaling=self.smooth_scaling)[0]
        self.forcing_term_rot.weights_[:, :] = self.dmp_quaternion_imitation(
            T, Y[:, 3:],
            n_weights_per_dim=self.n_weights_per_dim,
            regularization_coefficient=regularization_coefficient,
            alpha_y=self.alpha_y, beta_y=self.beta_y,
            overlap=self.forcing_term_rot.overlap,
            alpha_z=self.forcing_term_rot.alpha_z,
            allow_final_velocity=allow_final_velocity,
            smooth_scaling=self.smooth_scaling)[0]

        self.configure(start_y=Y[0], goal_y=Y[-1])

    def get_weights(self):

        return np.concatenate((self.forcing_term_pos.weights_.ravel(),
                               self.forcing_term_rot.weights_.ravel()))

    def set_weights(self, weights):

        n_pos_weights = self.forcing_term_pos.weights_.size
        self.forcing_term_pos.weights_[:, :] = weights[:n_pos_weights].reshape(
            -1, self.n_weights_per_dim)
        self.forcing_term_rot.weights_[:, :] = weights[n_pos_weights:].reshape(
            -1, self.n_weights_per_dim)

    def dmp_quaternion_imitation(self,
            T, Y, n_weights_per_dim, regularization_coefficient, alpha_y, beta_y,
            overlap, alpha_z, allow_final_velocity, smooth_scaling=False):
        
        if regularization_coefficient < 0.0:
            raise ValueError("Regularization coefficient must be >= 0!")

        forcing_term = ForcingTerm(
            3, n_weights_per_dim, T[-1], T[0], overlap, alpha_z)
        F, start_y, start_yd, start_ydd, goal_y, goal_yd, goal_ydd = \
            self.determine_forces_quaternion(
                T, Y, alpha_y, beta_y, alpha_z, allow_final_velocity,
                smooth_scaling)  # n_steps x n_dims

        X = forcing_term.design_matrix(T)  # n_weights_per_dim x n_steps

        return (self.ridge_regression(X, F, regularization_coefficient),
                start_y, start_yd, start_ydd, goal_y, goal_yd, goal_ydd)

    def determine_forces_quaternion(self,
            T, Y, alpha_y, beta_y, alpha_z, allow_final_velocity,
            smooth_scaling=False):
        
        n_dims = 3

        DT = np.gradient(T)

        Yd = pr.quaternion_gradient(Y) / DT[:, np.newaxis]
        if not allow_final_velocity:
            Yd[-1, :] = 0.0

        Ydd = np.empty_like(Yd)
        for d in range(n_dims):
            Ydd[:, d] = np.gradient(Yd[:, d]) / DT
        Ydd[-1, :] = 0.0

        execution_time = T[-1] - T[0]
        goal_y = Y[-1]
        start_y = Y[0]
        goal_y_minus_start_y = pr.compact_axis_angle_from_quaternion(
            pr.concatenate_quaternions(goal_y, pr.q_conj(start_y)))
        S = phase(T, alpha_z, T[-1], T[0])
        F = np.empty((len(T), n_dims))
        for t in range(len(T)):
            if smooth_scaling:
                smoothing = beta_y * S[t] * goal_y_minus_start_y
            else:
                smoothing = 0.0
            F[t, :] = execution_time ** 2 * Ydd[t] - alpha_y * (
                beta_y * pr.compact_axis_angle_from_quaternion(
                    pr.concatenate_quaternions(goal_y, pr.q_conj(Y[t])))
                - execution_time * Yd[t]
                - smoothing
            )
        return F, Y[0], Yd[0], Ydd[0], Y[-1], Yd[-1], Ydd[-1]

    def determine_forces(self, T, Y, alpha_y, beta_y, alpha_z, allow_final_velocity,
                        smooth_scaling=False):
        n_dims = Y.shape[1]
        DT = np.gradient(T)
        Yd = np.empty_like(Y)
        for d in range(n_dims):
            Yd[:, d] = np.gradient(Y[:, d]) / DT
        if not allow_final_velocity:
            Yd[-1, :] = 0.0
        Ydd = np.empty_like(Y)
        for d in range(n_dims):
            Ydd[:, d] = np.gradient(Yd[:, d]) / DT
        Ydd[-1, :] = 0.0

        execution_time = T[-1] - T[0]
        goal_y = Y[-1]
        start_y = Y[0]
        Z = phase(T, alpha_z, T[-1], T[0])
        F = np.empty((len(T), n_dims))
        for t in range(len(T)):
            if smooth_scaling:
                smoothing = beta_y * (goal_y - start_y) * Z[t]
            else:
                smoothing = 0.0
            F[t, :] = execution_time ** 2 * Ydd[t] - alpha_y * (
                beta_y * (goal_y - Y[t])
                - Yd[t] * execution_time
                - smoothing
            )
        return F, Y[0], Yd[0], Ydd[0], Y[-1], Yd[-1], Ydd[-1]


class CouplingTermObstacleAvoidance3D:  # for DMP
    def __init__(self, obstacle_position, gamma=1000.0, beta=20.0 / math.pi):
        self.obstacle_position = obstacle_position
        self.gamma = gamma
        self.beta = beta
        self.epsilon = 1e-10

    def coupling(self, y, yd):
        if len(self.obstacle_position.shape)>1:
            cdd = np.zeros_like(y)
            for pose in self.obstacle_position:
                cdd += self.obstacle_avoidance_acceleration_3d(y, yd, pose, self.gamma, self.beta)
            return np.zeros_like(cdd), cdd
        else:
            cdd = self.obstacle_avoidance_acceleration_3d(
                y, yd, self.obstacle_position, self.gamma, self.beta)
            return np.zeros_like(cdd), cdd

    def obstacle_avoidance_acceleration_3d(self,
            y, yd, obstacle_position, gamma=1000.0, beta=20.0 / math.pi):

        obstacle_diff = obstacle_position - y
        r = 0.5 * np.pi * pr.norm_vector(np.cross(obstacle_diff, yd))
        R = pr.matrix_from_compact_axis_angle(r)
        theta = np.arccos(
            np.dot(obstacle_diff, yd)
            / (np.linalg.norm(obstacle_diff) * np.linalg.norm(yd) + self.epsilon))
        cdd = gamma * np.dot(R, yd) * theta * np.exp(-beta * theta)
        return cdd


if __name__ == '__main__':

    rospy.init_node('custom_dmp', anonymous=True)

    # Initialize MoveIt!
    robot = RobotCommander()
    group_name = "manipulator"
    move_group = MoveGroupCommander(group_name)
    planning_scene_interface = PlanningSceneInterface()

    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)


    #display_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=1, latch=True)
    imitated_path_pub = rospy.Publisher("/imitated_path_avoidance", Path, queue_size=1)

    dt = 0.01 # do not change
    execution_time = 1.0
    samples = 100

    #obstacle = np.array([0.3, 0.3, 0.2])
    obstacle = np.array([0.0, 0.4, 0.1])

    dmp = CartesianDMP(
        execution_time=execution_time, dt=dt,
        n_weights_per_dim=100, int_dt=0.0001)
    Y = np.zeros((samples, 7))
    T = np.linspace(0, execution_time, len(Y))

    Y[:, 0] = np.linspace(0.35, -0.35, samples)
    #Y[:, 1] = 0.4
    Y[:, 1] = np.concatenate((np.linspace(0.3, 0.4, samples//2),np.linspace(0.4, 0.3, samples//2)))
    #Y[:, 2] = np.concatenate((np.linspace(0.05, 0.25, samples//2),np.linspace(0.25, 0.05, samples//2)))
    Y[:, 2] = 0.15
    
    Y[:, 3] = 0.14574490410891283
    Y[:, 4] = 0.6706940470850778
    Y[:, 5] = 0.16227650367257673
    Y[:, 6] = 0.7089388228096465

    start_p = Y[0,:]
    start_v = np.zeros(6)
    end_p = Y[-1,:]
    end_v = np.zeros(6)

    move_group.set_end_effector_link('rg2_eef_link')

    print(move_group.get_current_pose())
    
    print('Moving to start...')
    #move_group.set_end_effector_link('wrist_3_joint')
    point = start_p.tolist()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = point[0]
    pose_goal.position.y = point[1]
    pose_goal.position.z = point[2]
    pose_goal.orientation.x = point[3]
    pose_goal.orientation.y = point[4]
    pose_goal.orientation.z = point[5]
    pose_goal.orientation.w = point[6]
    move_group.set_pose_target(pose_goal)

    (success, my_plan, planning_time, error_code) = move_group.plan()
    print(success, error_code)
    move_group.execute(my_plan,wait=True)

    move_group.clear_pose_targets()
    move_group.stop()
    print('Moved')
    
   
    print('Computing DMP...')
    #coupling_term = CouplingTermObstacleAvoidance3D(obstacle, gamma=1000.0, beta=9.0 / math.pi)
    coupling_term = CouplingTermObstacleAvoidance3D(obstacle, gamma=1000.0, beta=10.0 / math.pi)

    dmp.imitate(T, Y, allow_final_velocity=True)
    dmp.configure(start_y=Y[0], goal_y=Y[-1])

    last_p=start_p
    last_v=start_v
    desired_positions=[]
    desired_velocities=[]

    waypoints=[]
    poses = []

    wpose = move_group.get_current_pose().pose
    
    for i in range(int(execution_time / dt)):
        p, v = dmp.step(last_p, last_v, coupling_term=coupling_term)

        desired_positions.append(p.tolist())
        desired_velocities.append(v.tolist())

        last_v = v
        last_p = p

        point=p.tolist()
        wpose.position.x,wpose.position.y ,wpose.position.z  = point[:3]
        wpose.orientation.x,wpose.orientation.y ,wpose.orientation.z ,wpose.orientation.w = point[3:]
        waypoints.append(copy.deepcopy(wpose))

        poses.append(PoseStamped(pose=Pose(position=Point(x=wpose.position.x, y=wpose.position.y, z=wpose.position.z))))#, header=Header(frame_id = "base_link")))
        
    print('Computed')
    
    imitated_path = Path()
    imitated_path.header.frame_id = "base_link"
    #imitated_path.header.stamp = rospy.Time.now()
    imitated_path.poses = poses

    for _ in range(10):
        imitated_path_pub.publish(imitated_path)
    
    
    # Plan the trajectory
    print('Converting castesina path to joint plan...')
    #move_group.set_pose_targets(desired_positions)
    #(success, my_plan, planning_time, error_code) = move_group.plan()   

    (my_plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
    if fraction ==1:
        print('Plan correct')

        print('Moving...')
        # Execute the trajectory
        move_group.execute(my_plan, wait=True)
        move_group.stop()
        print('Moved')
    else:
        print('Plan incorrect, fraction: ',str(fraction))


    dP = np.asarray(desired_positions)
    dV = np.asarray(desired_velocities)


    ax = plt.figure().add_subplot(projection='3d')
    ax.scatter(obstacle[0], obstacle[1],obstacle[2], s=50, color='tab:orange')
    ax.plot(Y[:, 0], Y[:, 1], Y[:, 2], 'tab:red', label="Demo")
    ax.scatter([[Y[0, 0], Y[-1, 0]]], [[Y[0, 1], Y[-1, 1]]], [[Y[0, 2], Y[-1, 2]]], s=20, color='tab:red')
    #ax.plot(P[:, 0], P[:, 1], P[:, 2], 'tab:blue',label="Actual")
    #ax.scatter([[P[0, 0], P[-1, 0]]], [[P[0, 1], P[-1, 1]]], [[P[0, 2], P[-1, 2]]], s=20, color='tab:blue')
    ax.plot(dP[:, 0],dP[:, 1],dP[:, 2], 'tab:green', label="Desired")
    ax.scatter([[dP[0, 0], dP[-1, 0]]],[[dP[0, 1], dP[-1, 1]]],[[dP[0, 2], dP[-1, 2]]], s=20, color='tab:green')
    ax.set_xlim([-0.6,0.6])
    ax.set_ylim([-0.6,0.6])
    ax.set_zlim([-0.6,0.6])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()
    plt.show()
