#!/usr/bin/env python3
"""Generate config/cr10_grape.srdf.

Written as a generator rather than by hand because the platform contributes a
block of rigid link pairs whose collision checks all have to be switched off,
and hand-maintaining ~50 <disable_collisions> lines is how typos get in.

The stow pose is an argument, not a placeholder to fill in afterwards:

    rosrun grape_harvest stow_check.py            # measures the envelope
    python3 gen_srdf.py ../config/cr10_grape.srdf --stow 0 -1.30 2.40 -2.67 -1.5708 0

An earlier version left __J1__ markers in the template for a human to
substitute, which is a good way to ship an SRDF full of literal __J1__.
"""
import argparse
import itertools
import os
import sys

# Folded over the deck for driving. The default is the starting point for
# stow_check.py, not a measured result -- run it and pass --stow.
DEFAULT_STOW = [0.0, -1.30, 2.40, -2.67, -1.5708, 0.0]

ap = argparse.ArgumentParser()
ap.add_argument("out", nargs="?", default="cr10_grape.srdf")
ap.add_argument("--stow", nargs=6, type=float, default=DEFAULT_STOW,
                metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                help="stow joint values, from stow_check.py")
args = ap.parse_args()

# Everything bolted rigidly to the chassis: no pair of these can ever collide,
# so every combination is disabled.
PLATFORM = ["base_link", "skid_plate", "crate_link", "camera_mast",
            "camera_link", "arm_column", "arm_base_link",
            "wheel_fl_link", "wheel_fr_link", "wheel_rl_link", "wheel_rr_link"]

# Arm links close enough to the mount that a collision check is meaningless.
# Deliberately NOT extended to Link3..Link6 or the blades: those really can
# swing into the crate or the camera mast, and the planner must see that.
NEAR_MOUNT = {
    "Link1": PLATFORM,
    "Link2": ["arm_column", "arm_base_link", "base_link", "camera_mast"],
}

ARM_ADJACENT = [
    ("Link1", "Link2", "Adjacent"),
    ("Link1", "Link4", "Never"),
    ("Link2", "Link3", "Adjacent"),
    ("Link3", "Link4", "Adjacent"),
    ("Link4", "Link5", "Adjacent"),
    ("Link4", "Link6", "Never"),
    ("Link4", "ee_base", "Never"),
    ("Link5", "Link6", "Adjacent"),
    ("Link5", "ee_base", "Never"),
    ("Link5", "left_blade", "Never"),
    ("Link5", "right_blade", "Never"),
    ("Link6", "ee_base", "Adjacent"),
    ("Link6", "left_blade", "Never"),
    ("Link6", "right_blade", "Never"),
    ("ee_base", "left_blade", "Adjacent"),
    ("ee_base", "right_blade", "Adjacent"),
    ("left_blade", "right_blade", "Never"),
]

rows = []
for a, b in itertools.combinations(PLATFORM, 2):
    rows.append((a, b, "Rigidly mounted"))
for link, others in NEAR_MOUNT.items():
    for o in others:
        rows.append((link, o, "Never"))
rows += ARM_ADJACENT

body = "\n".join(
    '    <disable_collisions link1="%s" link2="%s" reason="%s"/>' % r
    for r in rows)

stow = "\n".join(
    '        <joint name="joint%d" value="%.4f"/>' % (i + 1, v)
    for i, v in enumerate(args.stow))

srdf = """<?xml version="1.0" ?>
<robot name="cr10_robot">
    <!-- The arm chain starts at the CR10 mounting flange on top of the column,
         not at the chassis, and ends at the cutter TCP rather than the bare
         Link6 flange.

         The group is called `arm`, not `cr10_arm`: the name is baked into six
         config files and five scripts, and one that encodes the model has to
         be chased through all of them the next time the model changes. -->
    <group name="arm">
        <chain base_link="arm_base_link" tip_link="tcp_link"/>
    </group>
    <group name="gripper">
        <joint name="left_blade_joint"/>
        <joint name="right_blade_joint"/>
    </group>
    <end_effector name="cutter" parent_link="tcp_link" group="gripper"
                  parent_group="arm"/>

    <group_state name="home" group="arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="0"/>
        <joint name="joint3" value="0"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="0"/>
        <joint name="joint6" value="0"/>
    </group_state>
    <!-- elbow-up posture facing the vine, cutter roughly horizontal -->
    <group_state name="scan" group="arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="-0.7"/>
        <joint name="joint3" value="1.4"/>
        <joint name="joint4" value="-0.7"/>
        <joint name="joint5" value="-1.5708"/>
        <joint name="joint6" value="0"/>
    </group_state>
    <!-- STOW: folded over the deck for driving between rows. Values come from
         scripts/stow_check.py, which measures the actual link envelope rather
         than guessing, and has to be re-run whenever the arm changes -- the
         CR10's links are 40%% longer than the CR5's. -->
    <group_state name="stow" group="arm">
%s
    </group_state>
    <group_state name="open" group="gripper">
        <joint name="left_blade_joint" value="0.042"/>
        <joint name="right_blade_joint" value="0.042"/>
    </group_state>
    <group_state name="closed" group="gripper">
        <joint name="left_blade_joint" value="0.0"/>
        <joint name="right_blade_joint" value="0.0"/>
    </group_state>

    <!-- The robot drives, so the base is no longer pinned to the world. A
         floating joint (not planar) because the dirt track pitches and rolls
         the chassis, and MoveIt has to see that to plan against a fixed vine. -->
    <virtual_joint name="world_joint" type="floating"
                   parent_frame="world" child_link="base_footprint"/>

%s
</robot>
""" % (stow, body)

with open(args.out, "w") as f:
    f.write(srdf)
print("wrote %s (%d disable_collisions pairs, stow=%s)"
      % (args.out, len(rows), " ".join("%.4f" % v for v in args.stow)))
