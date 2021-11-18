"""
 The transform.py is responsible for doing the calculations to transform the visualzied objects to the link0 frame.

PARAMETERS:
    name - name of the object (pancake, spatula, bottle)
    obj_pos - position of the object in the camera frame
    robot_atag - position of april tag of known distance from robot

"""


from math import inf, pi, sin, cos, tan, atan2, sqrt
import numpy as np

# Function to compute position of the object in robot frame
class TransformToRobot():
    def __init__(self, name, obj_pos):
        """Manages setup for the class.

		Args:
            name - name of the object (pancake, spatula, bottle)
            obj_pos - position of the object in the camera frame

		Returns:
			None
		"""
        self.name = name
        self.obj_pos = np.array(obj_pos)
        self.link0_ratag_tf = np.array([0.1143, 0.1111, 0]) # THESE ARE ARBITRALILY SET RIGHT NOW

    def compute(self):
        """Computes position of object with respect to link0.

		Args:
            None
		Returns:
			pose_list - list of 7 values corresponding to a pose
		"""
        pose = np.zeros(7)
        pose[0:2] = self.obj_pos
        # apr_obj_tf = np.array([0,0,0])
        # apr_obj_tf = self.obj_pos - self.robot_atag
        # apr_obj_tf[2] = -1 *apr_obj_tf[2]
        # pose[0:3] = self.link0_ratag_tf + apr_obj_tf
        if self.name == "pancake":
            pose[3] = 1
            pose[4] = 0
            pose[5] = 0
            pose[6] = 0
        elif self.name == "spatula":
            pose[3] = 1
            pose[4] = 0
            pose[5] = 0
            pose[6] = 0
        elif self.name == "bottle":
            pose[3] = 0.629614
            pose[4] = 0.28868
            pose[5] = -0.348472
            pose[6] = 0.631521
        else:
            KeyError("Object does not exist") 
        return pose.tolist()