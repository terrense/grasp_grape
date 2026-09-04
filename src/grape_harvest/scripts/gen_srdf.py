#!/usr/bin/env python3
"""Generate config/cr5_grape.srdf.

Written as a generator rather than by hand because the platform contributes a
block of rigid link pairs whose collision checks all have to be switched off,
and hand-maintaining ~50 <disable_collisions> lines is how typos get in.
"""
import itertools
import os
import sys

OUT = sys.argv[1] if len(sys.argv) > 1 else "cr5_grape.srdf"

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

srdf = """<?xml version="1.0" ?>
<robot name="cr5_robot">
    <!-- The arm chain starts at the CR5 mounting flange on top of the column,
         not at the chassis, and ends at the cutter TCP rather than the bare
         Link6 flange. -->
    <group name="cr5_arm">
        <chain base_link="arm_base_link" tip_link="tcp_link"/>
    </group>
    <group name="gripper">
        <joint name="left_blade_joint"/>
        <joint name="right_blade_joint"/>
    </group>
    <end_effector name="cutter" parent_link="tcp_link" group="gripper"
                  parent_group="cr5_arm"/>

    <group_state name="home" group="cr5_arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="0"/>
        <joint name="joint3" value="0"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="0"/>
        <joint name="joint6" value="0"/>
    </group_state>
    <!-- elbow-up posture facing the vine, cutter roughly horizontal -->
    <group_state name="scan" group="cr5_arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="-0.7"/>
        <joint name="joint3" value="1.4"/>
        <joint name="joint4" value="-0.7"/>
        <joint name="joint5" value="-1.5708"/>
        <joint name="joint6" value="0"/>
    </group_state>
    <!-- STOW: folded upright over the deck for driving between rows.
         Values come from scripts/stow_check.py, which measures the actual
         link envelope rather than guessing. -->
    <group_state name="stow" group="cr5_arm">
        <joint name="joint1" value="__J1__"/>
        <joint name="joint2" value="__J2__"/>
        <joint name="joint3" value="__J3__"/>
        <joint name="joint4" value="__J4__"/>
        <joint name="joint5" value="__J5__"/>
        <joint name="joint6" value="__J6__"/>
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
""" % body

with open(OUT, "w") as f:
    f.write(srdf)
print("wrote %s (%d disable_collisions pairs)" % (OUT, len(rows)))
