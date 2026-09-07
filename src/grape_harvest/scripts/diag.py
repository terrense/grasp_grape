#!/usr/bin/env python3
"""Diagnostic: what does the tool frame actually look like, and which of the
key poses have an IK solution (as opposed to no collision-free path)?"""
import math
import os
import sys

import rospy
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import PoseStamped

import moveit_commander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_models import VINE_X, BUNCH_Y, CRATE_XY, CRATE_H, GRASP_Z, BUNCH_DROP_BELOW_TCP
from pick_grape import tool_quat, pose_at, APPROACH, RETREAT

rospy.init_node("diag", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander("arm")

cur = arm.get_current_pose().pose
q = [cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w]
R = tft.quaternion_matrix(q)[:3, :3]
print("=" * 68)
print("current TCP position : %.3f %.3f %.3f" % (cur.position.x, cur.position.y, cur.position.z))
print("current tool X axis  : %s" % np.round(R[:, 0], 3))
print("current tool Y axis  : %s" % np.round(R[:, 1], 3))
print("current tool Z axis  : %s  <- approach direction" % np.round(R[:, 2], 3))

Rt = tft.quaternion_matrix([tool_quat(0.0).x, tool_quat(0.0).y,
                            tool_quat(0.0).z, tool_quat(0.0).w])[:3, :3]
print("-" * 68)
print("tool_quat(0) X axis  : %s" % np.round(Rt[:, 0], 3))
print("tool_quat(0) Y axis  : %s  <- blades separate along this" % np.round(Rt[:, 1], 3))
print("tool_quat(0) Z axis  : %s  <- approach direction" % np.round(Rt[:, 2], 3))
print("=" * 68)

rospy.wait_for_service("/compute_ik", timeout=30)
ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)

crate_az = math.atan2(CRATE_XY[1], CRATE_XY[0])
drop_z = CRATE_H + BUNCH_DROP_BELOW_TCP + 0.06

poses = [
    ("grasp",      pose_at(VINE_X, BUNCH_Y[0], GRASP_Z, 0.0)),
    ("pre_grasp",  pose_at(VINE_X + APPROACH, BUNCH_Y[0], GRASP_Z, 0.0)),
    ("post_cut",   pose_at(VINE_X + RETREAT, BUNCH_Y[0], GRASP_Z + 0.04, 0.0)),
    ("over_crate", pose_at(CRATE_XY[0], CRATE_XY[1], drop_z + 0.12, crate_az)),
    ("release",    pose_at(CRATE_XY[0], CRATE_XY[1], drop_z, crate_az)),
    # same points but with the tool pointing the other way, to test the
    # sign convention
    ("grasp_flip",     pose_at(VINE_X, BUNCH_Y[0], GRASP_Z, math.pi)),
    ("pre_grasp_flip", pose_at(VINE_X + APPROACH, BUNCH_Y[0], GRASP_Z, math.pi)),
]

for avoid in (True, False):
    print("\n--- IK, avoid_collisions=%s ---" % avoid)
    for name, p in poses:
        req = GetPositionIKRequest()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "tcp_link"
        req.ik_request.robot_state = arm.get_current_state() \
            if hasattr(arm, "get_current_state") else moveit_commander.RobotCommander().get_current_state()
        req.ik_request.avoid_collisions = avoid
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose = p
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = rospy.Duration(2.0)
        try:
            res = ik(req)
            code = res.error_code.val
            if code == 1:
                j = [round(v, 3) for v in res.solution.joint_state.position[:6]]
                print("  %-16s OK    joints=%s" % (name, j))
            else:
                print("  %-16s FAIL  error_code=%d" % (name, code))
        except Exception as e:
            print("  %-16s EXC   %s" % (name, e))

moveit_commander.roscpp_shutdown()
