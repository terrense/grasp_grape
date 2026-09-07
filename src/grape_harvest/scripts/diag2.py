#!/usr/bin/env python3
"""Diagnostic: which links/objects are actually in contact at the start state
and at the IK solution for each key pose?"""
import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped

import moveit_commander
from moveit_msgs.srv import (GetPositionIK, GetPositionIKRequest,
                             GetStateValidity, GetStateValidityRequest)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pick_grape as pg

rospy.init_node("diag2", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")

rospy.wait_for_service("/compute_ik", timeout=30)
rospy.wait_for_service("/check_state_validity", timeout=30)
ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
validity = rospy.ServiceProxy("/check_state_validity", GetStateValidity)

h = pg.GrapeHarvester.__new__(pg.GrapeHarvester)
h.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
h.arm = arm
pg.GrapeHarvester.build_planning_scene(h)


def report(label, state):
    req = GetStateValidityRequest()
    req.robot_state = state
    req.group_name = "arm"
    res = validity(req)
    if res.valid:
        print("  %-12s VALID" % label)
    else:
        print("  %-12s INVALID  (%d contacts)" % (label, len(res.contacts)))
        seen = set()
        for c in res.contacts:
            k = (c.contact_body_1, c.contact_body_2)
            if k in seen:
                continue
            seen.add(k)
            print("        %s  <->  %s" % (c.contact_body_1, c.contact_body_2))
    return res.valid


print("=" * 68)
cur = robot.get_current_state()
print("START STATE")
report("scan/current", cur)

print("\nGOAL STATES (from collision-aware IK)")
poses = [("pre_grasp", h.pre_grasp if hasattr(h, "pre_grasp") else None)]

# rebuild the key poses without running __init__
crate_az = math.atan2(pg.CRATE_XY[1], pg.CRATE_XY[0])
drop_z = pg.CRATE_H + pg.BUNCH_DROP_BELOW_TCP + 0.06
keys = [
    ("grasp",      pg.pose_at(pg.VINE_X, pg.BUNCH_Y[0], pg.GRASP_Z, 0.0)),
    ("pre_grasp",  pg.backed_off(pg.VINE_X, pg.BUNCH_Y[0], pg.GRASP_Z, 0.0, pg.APPROACH)),
    ("post_cut",   pg.backed_off(pg.VINE_X, pg.BUNCH_Y[0], pg.GRASP_Z, 0.0, pg.RETREAT, lift=0.05)),
    ("over_crate", pg.pose_at(pg.CRATE_XY[0], pg.CRATE_XY[1], drop_z + 0.12, crate_az)),
    ("release",    pg.pose_at(pg.CRATE_XY[0], pg.CRATE_XY[1], drop_z, crate_az)),
]

for name, p in keys:
    req = GetPositionIKRequest()
    req.ik_request.group_name = "arm"
    req.ik_request.ik_link_name = "tcp_link"
    req.ik_request.robot_state = cur
    req.ik_request.avoid_collisions = True
    ps = PoseStamped()
    ps.header.frame_id = "world"
    ps.pose = p
    req.ik_request.pose_stamped = ps
    req.ik_request.timeout = rospy.Duration(3.0)
    res = ik(req)
    if res.error_code.val != 1:
        # retry without collision avoidance so we can see WHY it collides
        req.ik_request.avoid_collisions = False
        res = ik(req)
        if res.error_code.val != 1:
            print("  %-12s NO IK AT ALL (%d)" % (name, res.error_code.val))
            continue
        print("  %-12s (IK only found with collisions allowed)" % name)
    report(name, res.solution)

print("=" * 68)
moveit_commander.roscpp_shutdown()
