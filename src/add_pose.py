
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    
    
    x_rel = math.sqrt(2.0)
    y_rel = math.sqrt(2.0)
    theta_rel = math.pi / 2.0
    
    odometry_motion = gtsam.Pose2(x_rel, y_rel, theta_rel)
    
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry_motion, ODOMETRY_NOISE))

    target_x = 4.0 + math.sqrt(2.0)
    target_y = math.sqrt(2.0)
    target_theta = math.pi / 2.0
    
    new_pose = gtsam.Pose2(target_x, target_y, target_theta)

    initial_estimate.insert(X(4), new_pose)

    return graph, initial_estimate