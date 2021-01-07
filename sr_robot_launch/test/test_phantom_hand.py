#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import rostest
from sr_robot_commander.sr_hand_commander import SrHandCommander
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs import JointState
from unittest import TestCase

PKG = "sr_robot_launch"


class TestHandJointMovement(TestCase):
    """
    Tests the Hand Commander
    """
    @classmethod
    def setUpClass(cls):
        cls.hand_type = rospy.get_param('~test_phantom/hand_type', 'hand_e_plus')
        if cls.hand_type == 'hand_e':
            cls.hand_type = 'hand_e_plus'
            cls.number_of_joints = 24
        if cls.hand_type == 'hand_lite':
            cls.number_of_joints = 16
        if cls.hand_type == 'hand_extra_lite':
            cls.number_of_joints = 12
        elif cls.hand_type not in ('hand_e_plus', 'hand_lite', 'hand_extra_lite'):
            raise TypeError("The specified hand_type is incorrect.")
        cls.hand_id = rospy.get_param('~test_phantom/hand_id', 'rh')

        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        if cls.hand_id == 'rh':
            cls.hand_commander = SrHandCommander(name='right_hand')
        elif cls.hand_id == 'lh':
            cls.hand_commander = SrHandCommander(name='left_hand')
        else:
            raise TypeError("The specified hand_id is incorrect.")

    @classmethod
    def tearDownClass(cls):
        pass

    def test_number_of_joints(self):
        joint_state = rospy.wait_for_message('/joint_states', JointState)
        number_of_joints = len(joint_state.name)
        self.assertEqual(number_of_joints, self.number_of_joints)

    def test_plan_execution(self):
        self.hand_commander.move_to_named_target('pack', wait=True)
        rospy.sleep(2)
        goal_status = rospy.wait_for_message('/move_group/status', GoalStatusArray)
        goal_status = goal_status.status_list[0].status
        self.assertEqual(goal_status, 4)


if __name__ == "__main__":
    rospy.init_node('test_phantom', anonymous=True)
    rostest.rosrun(PKG, "test_phantom", TestHandJointMovement)