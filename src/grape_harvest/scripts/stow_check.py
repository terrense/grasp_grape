#!/usr/bin/env python3
"""Measure the stowed arm envelope instead of guessing it.

Sweeps candidate stow configurations, runs each through /compute_fk, and
reports the arm envelope in the base_link frame plus whether the state is
self-collision free. Ranks by lateral half-width first (that is what has to
fit between two trellis rows) then by height.

The reported extent is link-origin extent plus LINK_R, a flat allowance for
the link geometry around each origin -- it is an estimate, not a hull. The
authoritative clearance check is driving the aisle stowed and confirming
nothing in the vineyard moved (pick_grape.py --verify does that).
"""
import itertools
import math
import os
import sys

import rospy
from moveit_msgs.srv import (GetPositionFK, GetPositionFKRequest,
                             GetStateValidity, GetStateValidityRequest)
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

LINK_R = 0.09          # generous allowance for CR10 link cross-section
# the catch basket is the widest thing on the head and hangs 0.4 m below
# the tool, so it dominates the stowed envelope and has to be in here
ARM_LINKS = ["Link1", "Link2", "Link3", "Link4", "Link5", "Link6",
             "ee_base", "laser_link", "tcp_link", "ee_camera_link",
             "catch_basket"]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Half-width the stowed arm may occupy, measured from base_link.
#
# It is not the aisle half-width any more. The platform works a row from a lane
# LANE_STANDOFF = 0.80 m off it, so the binding side is the vine: the trunks sit
# 0.80 m away and the fruit hangs roughly in the row plane with a ~0.08 m
# cluster radius, leaving about 0.72 m before anything is touched. 0.62 m keeps
# ~0.10 m of margin on that side. The far side is the rest of the aisle,
# 1.4-2.2 m depending on the gap, and never binds.
#
# The CR5 fitted 0.45 m folded; the CR10 links are ~40% longer and its tightest
# collision-free fold is 0.565 m, so the old budget rejects every candidate.
LATERAL_BUDGET = 0.62


def candidates():
    out = []
    for j2 in (-0.9, -1.1, -1.3, -1.5):
        for j3 in (2.0, 2.4, 2.79):
            for j5 in (-1.5708, 0.0):
                # keep the wrist roughly vertical so the cutter points up
                j4 = -(j2 + j3) - math.pi / 2
                if j4 < -3.14 or j4 > 3.14:
                    continue
                out.append((0.0, j2, j3, round(j4, 4), j5, 0.0))
    # the pose the arm already uses while looking for fruit, for reference
    out.append((0.0, -0.7, 1.4, -0.7, -1.5708, 0.0))
    return out


def main():
    rospy.init_node("stow_check", anonymous=True)
    rospy.wait_for_service("/compute_fk", timeout=60)
    rospy.wait_for_service("/check_state_validity", timeout=60)
    fk = rospy.ServiceProxy("/compute_fk", GetPositionFK)
    validity = rospy.ServiceProxy("/check_state_validity", GetStateValidity)

    results = []
    for q in candidates():
        js = JointState()
        js.name = list(JOINTS)
        js.position = list(q)
        rs = RobotState()
        rs.joint_state = js
        rs.is_diff = True

        req = GetPositionFKRequest()
        req.header.frame_id = "base_link"
        req.fk_link_names = ARM_LINKS
        req.robot_state = rs
        res = fk(req)
        if res.error_code.val != 1 or not res.pose_stamped:
            continue

        zs = [p.pose.position.z for p in res.pose_stamped]
        xs = [p.pose.position.x for p in res.pose_stamped]
        ys = [p.pose.position.y for p in res.pose_stamped]
        top = max(zs) + LINK_R
        lat = max(max(abs(v) for v in xs), max(abs(v) for v in ys)) + LINK_R

        vreq = GetStateValidityRequest()
        vreq.robot_state = rs
        vreq.group_name = "arm"
        vres = validity(vreq)

        results.append((lat, top, q, vres.valid, len(vres.contacts)))

    results.sort(key=lambda r: (not r[3], r[0], r[1]))
    print("=" * 78)
    print("%-38s %7s %7s  %s" % ("joints (j1..j6)", "lat", "top", "valid"))
    print("-" * 78)
    for lat, top, q, valid, nc in results[:14]:
        print("%-38s %7.3f %7.3f  %s"
              % ("[" + ", ".join("%+.3f" % v for v in q) + "]",
                 lat, top, "yes" if valid else "NO (%d contacts)" % nc))
    print("=" * 78)

    ok = [r for r in results if r[3] and r[0] <= LATERAL_BUDGET]
    if not ok:
        print("no candidate fits the %.2f m lateral budget" % LATERAL_BUDGET)
        sys.exit(1)
    best = min(ok, key=lambda r: (r[1], r[0]))
    print("BEST  lat=%.3f m  top=%.3f m  joints=%s"
          % (best[0], best[1],
             " ".join("%.4f" % v for v in best[2])))
    print("SRDF  %s" % " ".join("%.4f" % v for v in best[2]))


if __name__ == "__main__":
    main()
