# coding: utf-8

"""
Planar robot environment helpers used within the introduction to robot path planning notebooks.

This code is part of a series of notebooks regarding  "Introduction to robot path planning".

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import math
import numpy as np

from shapely import plotting
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotlib.animation
from IPython.display import HTML, display
from shapely.affinity import rotate, translate

from IPEnvironment import CollisionChecker
from IPPerfMonitor import IPPerfMonitor


class ShapeRobot:
    """Simple planar robot wrapper that stores a Shapely geometry."""

    def __init__(self, geometry, limits=[[0.0, 22.0], [0.0, 22.0]]):
        self.geometry = geometry
        self._pose = [0.0, 0.0]
        # Store workspace limits so they can be reused by the collision checker.
        self.limits = [list(bound) for bound in limits]

    def setTo(self, pos): 
        """Move the robot geometry to a new absolute position."""
        x_pos, y_pos = pos
        dx = x_pos - self._pose[0]
        dy = y_pos - self._pose[1]
        self.geometry = translate(self.geometry, xoff=dx, yoff=dy)
        self._pose = (x_pos, y_pos)
        
    def getLimits(self):
        """Return the configuration limits of the robot."""
        return self.limits

    def drawRobot(self, ax, color="blue", **kwargs):
        """Draw the current robot geometry on the provided axes."""
        plotting.plot_polygon(self.geometry, ax=ax, add_points=False, color=color, **kwargs)
        ax.scatter(self._pose[0], self._pose[1], marker="x", color="k")

    def normalizePoses(self, poses: np.ndarray) -> np.ndarray:
        # Create a copy as float to prevent modifying the original and to support decimals
        norm_poses = np.array(poses, dtype=np.float64)

        # Create vectors for Min and Max values
        mins = np.min(self.limits, axis=1)
        maxs = np.max(self.limits, axis=1)

        # Calculate the range (denominator)
        ranges = maxs - mins

        # Handle possible divisions by 0
        ranges[ranges == 0] = 1.0

        # Apply Min-Max Normalization: (Value - Min) / (Max - Min)
        np.divide(norm_poses - mins, ranges, out=norm_poses)

        return norm_poses
    
    def denormalizePoses(self, poses: np.ndarray) -> np.ndarray:
        # Create a copy as float to ensure calculations are done with decimals
        denorm_poses = np.array(poses, dtype=np.float64)

        # Create vectors for Min and Max values
        mins = np.min(self.limits, axis=1)
        maxs = np.max(self.limits, axis=1)

        # Calculate the range (Max - Min)
        ranges = maxs - mins

        # Handle zero range for symmetry
        ranges[ranges == 0] = 1.0

        # Apply Denormalization Formula: (Value * Range) + Min
        denorm_poses = (denorm_poses * ranges) + mins

        return denorm_poses
        

class ShapeRobotWithOrientation(ShapeRobot):
    """Planar robot wrapper that also tracks orientation (theta)."""

    def __init__(
        self,
        geometry,
        limits=[[0.0, 22.0], [0.0, 22.0], [-math.pi, math.pi]],
        anchor=(0.0, 0.0),
        use_radians=True,
    ):
        super().__init__(geometry, limits=limits)
        self._template = geometry
        self._anchor = anchor
        self._orientation = 0.0
        self._use_radians = use_radians

    def setTo(self, pos):
        x_pos, y_pos, orientation = pos
        """Move the robot to an absolute pose (x, y, theta)."""
        if orientation == self._orientation:
            dx = x_pos - self._pose[0]
            dy = y_pos - self._pose[1]
            if dx == 0.0 and dy == 0.0:
                return
            self.geometry = translate(self.geometry, xoff=dx, yoff=dy)
        else:
            rotated = rotate(
                self._template,
                orientation,
                origin=self._anchor,
                use_radians=self._use_radians,
            )
            dx = x_pos - self._anchor[0]
            dy = y_pos - self._anchor[1]
            if dx != 0.0 or dy != 0.0:
                rotated = translate(rotated, xoff=dx, yoff=dy)
            self.geometry = rotated
        self._pose = (x_pos, y_pos)
        self._orientation = orientation
        


class CollisionCheckerShapeRobot(CollisionChecker):
    """Collision checker that evaluates intersections against a movable robot geometry."""

    def __init__(self, scene, robot, limits=None, statistic=None):
        if limits is None:
            limits = getattr(robot, "limits", None)
        if limits is None:
            limits = [[0.0, 22.0], [0.0, 22.0]]
        super().__init__(scene=scene, limits=limits, statistic=statistic)
        self.robot = robot
        self.dim = len(self.limits)
        
    def getDim(self):
        return self.dim
        
    def drawObstacles(self, ax: plt.Axes):
        """Draw all obstacles plus the robot at its current configuration."""
        super().drawObstacles(ax)
        #self.robot.drawRobot(ax)
        ax.set_xlim(self.limits[0])
        ax.set_ylim(self.limits[1])
        ax.grid(True)
        ax.set_xticks(range(int(self.limits[0][0]), int(self.limits[0][1]) + 1))
        ax.set_yticks(range(int(self.limits[1][0]), int(self.limits[1][1]) + 1))
        
    def getEnvironmentLimits(self): 
        """Return the configuration limits of the robot.
           Currently this seems not optimal, as there are two sources of limits (robot and environment),
           but for now this is kept for compatibility reasons. Dominating limits are the ones of the robot in any case."""
        return self.robot.getLimits()

    @IPPerfMonitor
    def pointInCollision(self, pos):
        """Check whether the robot placed at the given position collides with the scene."""
        assert len(pos) == self.getDim()
        self.robot.setTo(pos)
        for obstacle in self.scene.values():
            if obstacle.intersects(self.robot.geometry):
                return True
        return False


def get_interpolated_poses(poses: np.ndarray, step_size: float = 0.05) -> np.ndarray:
    """ This function interpolates a list of poses.
    
    The number of extra poses created depends on the euclidean distance between the two parent poses.T

    Parameters
    ---
    poses : The numpy array containing the poses [ [ x0,  y0,  a0], ...]
    step_size : The minimum euclidean distance between two created poses.
    """
    if len(poses) < 2:
        return poses
    
    interpolated_poses = [poses[0].tolist()]


    for i in range(len(poses) - 1):
        current_pose = poses[i]
        next_pose = poses[i+1]

        # Calculate euclidean distance between the current [i] and the next [i+1] pose.
        distance = np.linalg.norm(next_pose - current_pose)

        # Don't interpolate if the distance is to short to fit a new point
        if distance <= step_size:
            interpolated_poses.append(next_pose.tolist())
            continue

        # Calculate the number of segments
        num_poses = int(np.ceil(distance / step_size)) + 1

        # Linearly interpolate betwen the two poses, the result includes start and endpoint
        new_poses = np.linspace(current_pose, next_pose, num_poses)

        # Remove the start point
        new_poses = new_poses[1:]

        # Append to the already interpolated poses
        interpolated_poses.extend(new_poses.tolist())

    return np.array(interpolated_poses)


class Animator:
    def __init__(self, poses: np.ndarray, cc: CollisionCheckerShapeRobot, figsize: tuple = (9, 9)):
        self.poses = poses
        self.cc = cc
        self.robot: ShapeRobot = self.cc.robot

        self.fig, self.ax = plt.subplots(figsize=figsize)
        rcParams['animation.embed_limit'] = 124

    def animate(self, frame_interval_ms = 50):
        # Normalizes poses
        print(f"   Raw poses: {len(self.poses)}")
        poses = self.robot.normalizePoses(self.poses)
        # Interpolate poses
        poses = get_interpolated_poses(poses, 0.01)
        # Denormalize poses
        self.poses = self.robot.denormalizePoses(poses)
        print(f"   Interpolated poses: {len(self.poses)}")

        # Create the animation object
        animation = matplotlib.animation.FuncAnimation(
            self.fig, 
            self.update, 
            frames=len(poses), 
            interval=frame_interval_ms
        )

        # Convert to JS/HTML
        animation_html = HTML(animation.to_jshtml())
        print("   Done.")

        # Close the plot
        plt.close(self.fig)

        # Display the animation
        display(animation_html)

    def update(self, frame_idx):
        """ This is what happens every frame of the animation.
            """

        # Get the pose for this frame
        current_pose = self.poses[frame_idx]

        # Clear the plot
        self.ax.clear()

        # Re-add plot configuration 
        self.ax.set_xlim(self.cc.robot.limits[0][0], self.cc.robot.limits[0][1])
        self.ax.set_xticks(range(self.cc.robot.limits[0][0], self.cc.robot.limits[0][1] + 1))
        self.ax.set_ylim(self.cc.robot.limits[1][0], self.cc.robot.limits[1][1])
        self.ax.set_yticks(range(self.cc.robot.limits[1][0], self.cc.robot.limits[1][1] + 1))
        self.ax.grid(True)
        self.ax.set_title(f'Robot Animation (Frame {frame_idx+1}/{len(self.poses)})')

        # Plot the full trajectory
        self.ax.plot(self.poses[:, 0], self.poses[:, 1], 'g--', label='Full Trajectory')

        # Plot the already completed trajectory
        self.ax.plot(self.poses[:frame_idx+1, 0], self.poses[:frame_idx+1, 1], 'b-', label='Completed Path')

        # Draw the robot
        self.cc.robot.setTo(current_pose)
        self.cc.robot.drawRobot(self.ax)

        # Draw the obstacles
        self.cc.drawObstacles(self.ax)

        # Plot the legend
        self.ax.legend(loc='upper right')

