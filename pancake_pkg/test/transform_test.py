"""
Tests the python package pancake_pkg 

"""

import unittest
from numpy.testing._private.utils import assert_equal
from pancake_pkg.transform import TransformToRobot
from geometry_msgs.msg import Pose
import numpy as np

class Test_compute(unittest.TestCase):
    """ Runs series of tests on python package computations
    """
    def __init__(self, *args):
        super(Test_compute, self).__init__(*args)

    def test_spatula(self):
        """ Function to check the computation for the spatula pose
        """
        test_pose = np.zeros(7)
        test_pose[0] = 1
        test_pose[1] = 1
        test_pose[2] = 1
        test_pose[3] = -0.65337
        test_pose[4] = -0.232
        test_pose[5] = -0.28944
        test_pose[6] = 0.65993

        # pose = Pose()
        # pose.position.x = 1
        # pose.position.y = 1
        # pose.position.z = 1
        # pose.orientation.x = 0
        # pose.orientation.y = 0
        # pose.orientation.z = 0
        # pose.orientation.w = 1

        pose = np.zeros(7)
        pose[0] = 1
        pose[1] = 1
        pose[2] = 1
        pose[3] = 0
        pose[4] = 0
        pose[5] = 0
        pose[6] = 1

        name = "spatula"
        obj_pos = pose
        transform = TransformToRobot(name, obj_pos)
        
        computed = transform.compute()
        assert_equal(computed, test_pose)
        

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('pancake_pkg', 'test_compute', Test_compute)