import unittest
from arm import Arm
from endeffector import EndEffector

class TestEndEffector(unittest.TestCase):
    def test_default_gripper_type(self):
        ee = EndEffector()
        self.assertEqual(ee.ee_type, "TWOFINGERS")

class TestArm(unittest.TestCase):
    pass
        
if __name__ == '__main__':
    unittest.main()