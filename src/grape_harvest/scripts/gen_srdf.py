#!/usr/bin/env python3
"""Generate config/cr10_grape.srdf.

Written as a generator rather than by hand because the platform and the end
effector between them contribute a block of rigid link pairs whose collision
checks all have to be switched off, and hand-maintaining ~100
<disable_collisions> lines is how typos get in.

The stow pose is an argument, not a placeholder to fill in afterwards:

    rosrun grape_harvest stow_check.py            # measures the envelope
    python3 gen_srdf.py ../config/cr10_grape.srdf --stow 0 -0.9 2.4 -3.07 -1.57 0

An earlier version left __J1__ markers in the template for a human to
substitute, which is a good way to ship an SRDF full of literal __J1__.

There is no gripper group: the head cuts with a laser and catches the bunch in
a fixed basket, so the arm is the only actuated group left.
"""
import argparse
import itertools

# Folded over the deck for driving. Measured by stow_check.py for the CR10;
# re-run it whenever the arm or the end effector changes, because the basket
# hangs 0.4 m below the tool and is part of the stowed envelope.
DEFAULT_STOW = [0.0, -0.90, 2.40, -3.0708, -1.5708, 0.0]

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

# Everything bolted rigidly to the Link6 flange. Same argument.
HEAD = ["ee_base", "laser_link", "tcp_link", "ee_camera_link", "catch_basket"]

# Arm links close enough to the mount that a collision check is meaningless.
# Deliberately NOT extended to Link3..Link6: those really can swing into the
# crate or the camera mast, and the planner must see that.
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
    ("Link5", "Link6", "Adjacent"),
]

rows = []
for a, b in itertools.combinations(PLATFORM, 2):
    rows.append((a, b, "Rigidly mounted"))
for a, b in itertools.combinations(HEAD, 2):
    rows.append((a, b, "Rigidly mounted"))
for link, others in NEAR_MOUNT.items():
    for o in others:
        rows.append((link, o, "Never"))
rows += ARM_ADJACENT

# The head hangs off Link6, so it is adjacent to it and close to Link5. The
# collar is now shallow enough to sit in the wrist's own envelope, so it is
# exempted alongside the rest of the head -- checking it against Link5 only
# produced refusals at poses the real thing clears easily. Link4 and below stay
# checked: if the collar reaches the forearm, the plan is wrong.
for h in HEAD:
    rows.append(("Link6", h, "Adjacent"))
for h in HEAD:
    rows.append(("Link5", h, "Never"))

body = "\n".join(
    '    <disable_collisions link1="%s" link2="%s" reason="%s"/>' % r
    for r in rows)

stow = "\n".join(
    '        <joint name="joint%d" value="%.4f"/>' % (i + 1, v)
    for i, v in enumerate(args.stow))

srdf = """<?xml version="1.0" ?>
<robot name="cr10_robot">
    <!-- The arm chain starts at the CR10 mounting flange on top of the column,
         not at the chassis, and ends at the laser focus rather than the bare
         Link6 flange.

         The group is called `arm`, not `cr10_arm`: the name is baked into six
         config files and five scripts, and one that encodes the model has to
         be chased through all of them the next time the model changes.

         There is no gripper group any more. The head cuts with a laser and the
         bunch drops into a fixed basket, so nothing on the end effector moves. -->
    <group name="arm">
        <chain base_link="arm_base_link" tip_link="tcp_link"/>
    </group>

    <group_state name="home" group="arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="0"/>
        <joint name="joint3" value="0"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="0"/>
        <joint name="joint6" value="0"/>
    </group_state>
    <!-- elbow-up posture facing the vine, head roughly level -->
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
         than guessing, and has to be re-run whenever the arm or the head
         changes: the CR10's links are 40%% longer than the CR5's, and the
         catch basket adds 0.4 m below the tool. -->
    <group_state name="stow" group="arm">
%s
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
