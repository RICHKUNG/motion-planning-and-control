import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.3.1: Tune Stanley Gain
                 kp=1):
        self.path = None
        self.kp = kp
        self.l = model.l
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0

        # Search Front Wheel Target Locally
        front_x = x + self.l*np.cos(np.deg2rad(yaw))
        front_y = y + self.l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta)) if np.cos(np.deg2rad(delta)) != 0 else v
        
        min_idx, min_dist = utils.search_nearest_local(self.path, (front_x,front_y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        target = self.path[min_idx]

        # TODO 4.3.1: Stanley Control for Bicycle Kinematic Model
        theta_e = utils.angle_norm(target[2] - yaw)
        dist_e = np.sqrt((front_x - target[0])**2 + (front_y - target[1])**2)
        if (target[0] - front_x)*np.sin(np.deg2rad(target[2])) - (target[1] - front_y)*np.cos(np.deg2rad(target[2])) < 0:
            dist_e = -dist_e
        next_delta = theta_e + np.rad2deg(np.arctan2(-self.kp*dist_e, vf))
        # [end] TODO 4.3.1
    
        return next_delta
