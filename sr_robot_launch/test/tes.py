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
from sensor_msgs.msg import JointState


if __name__ == "__main__":
    rospy.init_node('test_phantom', anonymous=True)
    hand_type = 'hand_extra_lite'
    if hand_type == 'hand_e':
       hand_type = 'hand_e_plus'
       number_of_joints = 24
    if hand_type == 'hand_lite':
       number_of_joints = 16
    if hand_type == 'hand_extra_lite':
       number_of_joints = 12

    hand_id = 'lh'
    rospy.wait_for_message('/move_group/status', GoalStatusArray)
    if hand_id == 'rh':
       hand_commander = SrHandCommander(name='right_hand')
    elif hand_id == 'lh':
        hand_commander = SrHandCommander(name='left_hand')

    joint_state = rospy.wait_for_message('/joint_states', JointState)
    joints = len(joint_state.name)
    print "Number of joints expected:" + str(number_of_joints) + " and found: " + str(joints)

    print "Moving to pack position..."
    hand_commander.move_to_named_target('pack', wait=True)
    goal_status = rospy.wait_for_message('/move_group/status', GoalStatusArray)
    goal_status = goal_status.status_list[0].status
    if goal_status == 3:
        print "Planning was succesful. Status: " + str(goal_status)
    else:
        print "Planning failed. Status: " + str(goal_status)
