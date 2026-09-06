#!/usr/bin/env python3
"""Dobot CR5 grape harvesting from the mobile vineyard platform.

Per panel: drive the aisle until the panel is abreast of the arm, unstow, cut
every cluster on that panel into the deck crate, stow, move on.

Per cluster:

    approach standoff -> descend onto the peduncle -> close blades (clamp)
    -> cut (stem joint released, cluster handed to the cutter) -> retreat
    -> carry to its slot in the deck crate -> open (release)

"Cutting" is modelled with gazebo_ros_link_attacher: a cluster hangs off its
row's fruiting wire by a runtime fixed joint; the cut attaches it to the robot
flange and *then* removes the wire joint, so the fruit is never in free fall.

What changed from the fixed-pedestal version
--------------------------------------------
The arm base and the crate both ride on the platform now, so neither sits at a
constant place in the world. Vine-side poses are built in the world frame (the
fruit does not move) but their azimuth is measured from wherever the arm column
currently is; crate-side poses are built in `base_link` and pushed through TF
into the planning frame. Cluster geometry is no longer three hand-written y
coordinates -- it comes from make_models.bunch_layout(), which is also what
generated the world, so the scene and the motion plan cannot drift apart.

Collision safety: every motion is planned by MoveIt against a planning scene
carrying the posts, the canopy wires, the ground and every cluster still on the
vine. The crate and the vehicle are robot links, so MoveIt already avoids them.
The only object deliberately removed is the one cluster being cut, and only for
the final few centimetres of its own descent. `--verify` additionally watches
the Gazebo poses of everything the arm is not supposed to touch and reports any
that moved.
"""
import argparse
import copy
import math
import os
import sys
import time

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

import moveit_commander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_models import (WIRE_Z, CANOPY_Z, POST_H, PANEL_W, PANELS_PER_ROW,
                         N_ROWS, ROW_LEN, ROW_DX, CRATE_BASE_XYZ, CRATE_INNER,
                         CRATE_SLOTS, row_x, aisle_x, bunch_layout,
                         grape_bunch)

ROBOT_MODEL = "cr5_robot"
# ee_base/tcp_link are lumped into Link6 by the URDF->SDF fixed-joint merge,
# so Link6 is the link that actually exists in Gazebo.
ROBOT_EE_LINK = "Link6"
TRELLIS_MODEL = "trellis"
BUNCH_LINK = "bunch_link"

APPROACH = 0.15                    # standoff along the tool axis, metres
RETREAT = 0.18

BLADE_OPEN = 0.042
BLADE_CLOSED = 0.002               # slight squeeze on the 12 mm peduncle

CRATE_T = 0.014                    # crate floor thickness, matches the xacro
CRATE_FLOOR_Z = CRATE_BASE_XYZ[2] + CRATE_T
DROP_CLEARANCE = 0.05              # fruit hangs this far above the floor

# Prefilter only: a cluster is attempted if its grasp point is within this
# straight-line distance of the arm base. The CR5 reaches 900 mm to the flange
# and the cutter TCP sits 95 mm beyond it, so ~0.95 m is the outer envelope --
# but whether a given pose solves is an IK question, not a radius one. This
# just avoids burning planning time on obvious non-starters; --check-only
# reports what actually solves.
#
# The geometry is tight by construction: the aisle runs midway between rows
# 1.6 m apart, so the column always stands 0.80 m out from the fruit
# horizontally, and the fruiting wire is another 0.32-0.41 m above the arm
# base. Best case is therefore ~0.87 m of the ~0.95 m envelope, and a cluster
# much more than 0.2 m off the beam is already outside it -- which is why the
# platform parks once per cluster rather than once per panel.
REACH = 0.95
REACH_MIN = 0.25

# Cutter tilted down by this much. A purely horizontal approach puts the
# forearm at the same height as the top of the cluster and every plan collides;
# tilting down keeps the wrist above the fruit, which is also how a real
# shear-type harvesting head comes onto a peduncle.
PITCH = math.radians(20.0)


def tool_quat(azimuth, pitch=PITCH):
    """Cutter pointing outward from the arm column and tilted down by `pitch`.

    Tool +Z is the approach axis (out of the Link6 flange) and the blades
    separate along tool +/-Y, so the blades close across a vertical peduncle.

    The sign of the Y rotation matters: -90 deg puts the approach axis at
    (-1,0,0), which still solves for IK but makes the wrist reach *past* the
    trellis and point back through the fruiting zone.
    """
    q = tft.quaternion_multiply(
        tft.quaternion_about_axis(azimuth, (0, 0, 1)),
        tft.quaternion_about_axis(math.pi / 2 + pitch, (0, 1, 0)))
    return Quaternion(*q)


def approach_axis(azimuth, pitch=PITCH):
    """Unit vector the tool advances along. Standoffs are -dist along this."""
    return (math.cos(pitch) * math.cos(azimuth),
            math.cos(pitch) * math.sin(azimuth),
            -math.sin(pitch))


def pose_at(x, y, z, azimuth):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation = tool_quat(azimuth)
    return p


def backed_off(x, y, z, azimuth, dist, lift=0.0):
    """Pose `dist` short of (x,y,z) along the approach axis."""
    a = approach_axis(azimuth)
    return pose_at(x - dist * a[0], y - dist * a[1], z - dist * a[2] + lift,
                   azimuth)


class GrapeHarvester(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.arm = moveit_commander.MoveGroupCommander("cr5_arm")
        self.grip = moveit_commander.MoveGroupCommander("gripper")

        self.arm.set_planning_time(20.0)
        self.arm.set_num_planning_attempts(12)
        self.arm.set_goal_position_tolerance(0.003)
        self.arm.set_goal_orientation_tolerance(0.02)
        self.grip.set_max_velocity_scaling_factor(0.5)
        self.set_speed(0.35)

        self.planning_frame = self.arm.get_planning_frame()

        # The platform moves, so nothing on the robot has a fixed world pose.
        # Everything crate-side is resolved through TF at the moment it is used.
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        rospy.wait_for_service("/compute_ik", timeout=60.0)
        self.ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        self.slots_used = 0
        self.in_scene = set()

        rospy.loginfo("planning frame: %s", self.planning_frame)
        rospy.loginfo("end effector  : %s", self.arm.get_end_effector_link())

    def set_speed(self, f):
        self.arm.set_max_velocity_scaling_factor(f)
        self.arm.set_max_acceleration_scaling_factor(f)

    # -------------------------------------------------------------- frames
    def _mat(self, target, source, timeout=5.0):
        """4x4 transform that maps a point in `source` into `target`."""
        t = self.tf_buf.lookup_transform(target, source, rospy.Time(0),
                                         rospy.Duration(timeout))
        m = tft.quaternion_matrix([t.transform.rotation.x,
                                   t.transform.rotation.y,
                                   t.transform.rotation.z,
                                   t.transform.rotation.w])
        m[0, 3] = t.transform.translation.x
        m[1, 3] = t.transform.translation.y
        m[2, 3] = t.transform.translation.z
        return m

    def base_to_world(self, x, y, z):
        m = self._mat(self.planning_frame, "base_link")
        v = m.dot([x, y, z, 1.0])
        return v[0], v[1], v[2]

    def world_to_base(self, x, y, z):
        m = self._mat("base_link", self.planning_frame)
        v = m.dot([x, y, z, 1.0])
        return v[0], v[1], v[2]

    def base_yaw(self):
        m = self._mat(self.planning_frame, "base_link")
        return math.atan2(m[1, 0], m[0, 0])

    def arm_origin(self):
        """World xyz of the arm column, which is what azimuths radiate from."""
        m = self._mat(self.planning_frame, "arm_base_link")
        return m[0, 3], m[1, 3], m[2, 3]

    # ----------------------------------------------------------- key poses
    def vine_poses(self, spec):
        """(pre_grasp, grasp, post_cut) for one cluster.

        The fruit is fixed in the world, but the azimuth the tool comes in on
        is measured from wherever the arm column is standing right now.
        """
        ax, ay, _ = self.arm_origin()
        x, y, z = spec["x"], spec["y"], spec["grasp_z"]
        az = math.atan2(y - ay, x - ax)
        grasp = pose_at(x, y, z, az)
        pre = backed_off(x, y, z, az, APPROACH)
        post = backed_off(x, y, z, az, RETREAT, lift=0.05)
        return pre, grasp, post

    def crate_poses(self, slot, hang_below_tcp):
        """(over_crate, release) for one crate slot, in the planning frame.

        Built in base_link because the crate rides on the deck: the slot is at
        a constant place on the robot, not in the vineyard.
        """
        dx, dy = CRATE_SLOTS[slot % len(CRATE_SLOTS)]
        bx = CRATE_BASE_XYZ[0] + dx
        by = CRATE_BASE_XYZ[1] + dy
        bz = CRATE_FLOOR_Z + hang_below_tcp + DROP_CLEARANCE

        # Azimuth in the base frame, then rotated into the world: the crate is
        # behind the column, so the tool turns round to face it.
        az = math.atan2(by, bx) + self.base_yaw()

        rx, ry, rz = self.base_to_world(bx, by, bz)
        ox, oy, oz = self.base_to_world(bx, by, bz + 0.12)
        return pose_at(ox, oy, oz, az), pose_at(rx, ry, rz, az)

    def reachable(self, spec):
        """Straight-line distance from the arm base to the grasp point.

        Has to be 3D: the fruiting wire sits 0.3-0.4 m above the arm base, so a
        horizontal-only test would wave through clusters that are past the
        envelope once the height is counted.
        """
        ax, ay, az = self.arm_origin()
        d = math.sqrt((spec["x"] - ax) ** 2 + (spec["y"] - ay) ** 2
                      + (spec["grasp_z"] - az) ** 2)
        return REACH_MIN <= d <= REACH, d

    # ------------------------------------------------------------------ ROS
    def wait_for_services(self):
        rospy.loginfo("waiting for gazebo + link_attacher services ...")
        for s in ("/gazebo/spawn_sdf_model", "/gazebo/pause_physics",
                  "/gazebo/unpause_physics", "/link_attacher_node/attach",
                  "/link_attacher_node/detach"):
            rospy.wait_for_service(s, timeout=60.0)
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.attach = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        self.detach = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
        rospy.loginfo("services up")

    def _att(self, srv, m1, l1, m2, l2):
        req = AttachRequest()
        req.model_name_1, req.link_name_1 = m1, l1
        req.model_name_2, req.link_name_2 = m2, l2
        return srv.call(req)

    def model_pose(self, name, timeout=5.0):
        try:
            m = rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                       timeout=timeout)
        except rospy.ROSException:
            return None
        if name not in m.name:
            return None
        return m.pose[m.name.index(name)]

    # ---------------------------------------------------------------- scene
    def spawn_bunches(self, specs):
        """Spawn every cluster in the block and hang it on its row's wire.

        Physics is paused across spawn+attach: a free body would otherwise fall
        for the few hundred ms it takes the attach service to run. Each cluster
        gets its own SDF because bunch_layout() randomises peduncle length,
        body length, berry count and mass per cluster.
        """
        self.pause()
        try:
            for spec in specs:
                sdf = grape_bunch(**spec)
                p = Pose()
                p.position.x = spec["x"]
                p.position.y = spec["y"]
                p.position.z = WIRE_Z
                p.orientation.w = 1.0
                self.spawn(spec["name"], sdf, "", p, "world")
                # wall clock, not rospy.sleep: /clock is frozen while physics
                # is paused, so a sim-time sleep would never return
                time.sleep(0.15)
                r = self._att(self.attach, TRELLIS_MODEL,
                              "wire_r%d_fruit" % spec["row"],
                              spec["name"], BUNCH_LINK)
                if not r.ok:
                    rospy.logwarn("could not hang %s on wire_r%d_fruit",
                                  spec["name"], spec["row"])
                time.sleep(0.05)
        finally:
            self.unpause()
        rospy.loginfo("spawned and hung %d clusters", len(specs))
        rospy.sleep(0.5)

    def _box(self, name, x, y, z, sx, sy, sz):
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.w = 1.0
        self.scene.add_box(name, ps, size=(sx, sy, sz))
        self.in_scene.add(name)

    def _cyl(self, name, x, y, z, radius, height):
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.w = 1.0
        self.scene.add_cylinder(name, ps, height=height, radius=radius)
        self.in_scene.add(name)

    def build_planning_scene(self, specs):
        """Collision geometry for MoveIt: the whole block, built once.

        The vineyard is fixed in the world, so this does not need rebuilding as
        the platform drives. The crate and the vehicle are *robot links*, so
        MoveIt avoids them from the URDF and they must not be added here.

        Deliberately not included: the fruiting wires (4 mm radius, 75 mm above
        the grasp points -- adding them only produces spurious planning
        failures) and the peduncles, which the blades are supposed to close on.
        """
        margin = 1.5
        self._box("ground", row_x(N_ROWS - 1) / 2.0, 0.0, -0.03,
                  ROW_DX * (N_ROWS + 2) + margin, ROW_LEN + 2 * margin, 0.05)

        for r in range(N_ROWS):
            x = row_x(r)
            for i in range(PANELS_PER_ROW + 1):
                self._cyl("post_r%d_%d" % (r, i), x,
                          -ROW_LEN / 2.0 + i * PANEL_W, POST_H / 2,
                          0.05, POST_H)
            self._box("canopy_r%d" % r, x, 0.0, CANOPY_Z,
                      0.02, ROW_LEN, 0.02)

        # berry bodies only -- the top of each body sits pedu_len below the
        # wire, so the blades never have to plan through it
        for spec in specs:
            self._cyl(spec["name"], spec["x"], spec["y"],
                      WIRE_Z - spec["pedu_len"] - spec["body_len"] / 2.0,
                      spec["r_top"] * 0.9, spec["body_len"])

        rospy.sleep(1.0)
        rospy.loginfo("planning scene: %d objects", len(self.in_scene))

    def add_settled_cluster(self, spec):
        """Re-add a released cluster where it actually came to rest.

        The crate rides on the robot, so a cluster lying in it moves with the
        vehicle and its world-frame collision object would go stale the moment
        the platform drives off. It is therefore attached to `crate_link`
        rather than added to the world -- MoveIt then carries it along.
        """
        name = spec["name"]
        p = self.model_pose(name)
        if p is None:
            rospy.logwarn("  could not read %s pose; crate contents not added "
                          "to the planning scene", name)
            return
        bx, by, bz = self.world_to_base(
            p.position.x, p.position.y,
            p.position.z - spec["pedu_len"] - spec["body_len"] / 2.0)

        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = bx, by, bz
        ps.pose.orientation.w = 1.0
        self.scene.attach_box("crate_link", name + "_in_crate", pose=ps,
                              size=(2 * spec["r_top"] + 0.02,
                                    2 * spec["r_top"] + 0.02,
                                    spec["body_len"]),
                              touch_links=["crate_link", "base_link"])
        rospy.sleep(0.4)
        rospy.loginfo("  %s added to the crate contents", name)

    # ------------------------------------------------------------- motions
    def gripper(self, q, what):
        rospy.loginfo("gripper -> %s (%.3f m)", what, q)
        self.grip.set_joint_value_target({"left_blade_joint": q,
                                          "right_blade_joint": q})
        ok = self.grip.go(wait=True)
        self.grip.stop()
        return ok

    def go_named(self, name):
        rospy.loginfo("arm -> named pose %s", name)
        self.arm.set_start_state_to_current_state()
        self.arm.set_named_target(name)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return ok

    def solve_ik(self, pose, avoid=True, timeout=3.0):
        """Collision-aware IK seeded from the current state.

        Going through IK explicitly and then planning in joint space is much
        more repeatable than handing OMPL a pose goal: a pose goal lets the
        planner pick any of the many IK solutions, and the arm posture it lands
        in decides whether the following straight-line descent is possible.
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = "cr5_arm"
        req.ik_request.ik_link_name = self.arm.get_end_effector_link()
        req.ik_request.robot_state = self.robot.get_current_state()
        req.ik_request.avoid_collisions = avoid
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = rospy.Duration(timeout)
        try:
            res = self.ik(req)
        except rospy.ServiceException as e:
            rospy.logwarn("IK service failed: %s", e)
            return None
        if res.error_code.val != 1:
            return None
        js = res.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position)
                if n.startswith("joint")}

    def go_pose(self, pose, label, tries=3):
        rospy.loginfo("arm -> %s  (%.3f, %.3f, %.3f)", label,
                      pose.position.x, pose.position.y, pose.position.z)
        for attempt in range(1, tries + 1):
            self.arm.set_start_state_to_current_state()
            target = self.solve_ik(pose)
            if target:
                self.arm.set_joint_value_target(target)
            else:
                rospy.logwarn("  no collision-free IK, handing pose to OMPL")
                self.arm.set_pose_target(pose)
            ok = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            if ok:
                return True
            rospy.logwarn("  attempt %d/%d to reach %s failed",
                          attempt, tries, label)
            rospy.sleep(0.5)
        rospy.logerr("FAILED to reach %s", label)
        return False

    def go_cartesian(self, pose, label, step=0.005, tries=2):
        """Straight-line move -- what the approach and retreat must be."""
        rospy.loginfo("arm -> %s (cartesian)", label)
        for attempt in range(1, tries + 1):
            self.arm.set_start_state_to_current_state()
            wp = [copy.deepcopy(pose)]
            try:
                # MoveIt >= 1.1.14 dropped the jump_threshold argument
                plan, frac = self.arm.compute_cartesian_path(wp, step, True)
            except TypeError:
                plan, frac = self.arm.compute_cartesian_path(wp, step, 0.0)
            rospy.loginfo("  cartesian coverage %.0f%%", frac * 100)
            if frac < 0.9:
                break
            if self.arm.execute(plan, wait=True):
                self.arm.stop()
                return True
            self.arm.stop()
            rospy.logwarn("  cartesian execution attempt %d/%d failed",
                          attempt, tries)
            rospy.sleep(0.5)
        rospy.logwarn("  falling back to joint-space for %s", label)
        return self.go_pose(pose, label)

    # ------------------------------------------------------- harvest cycle
    def harvest(self, spec, slot):
        name = spec["name"]
        pre, grasp, post = self.vine_poses(spec)
        over, release = self.crate_poses(slot, spec["hang_below_tcp"])
        rospy.loginfo("########### harvesting %s  (%.2f, %.2f, %.2f) #######",
                      name, spec["x"], spec["y"], spec["grasp_z"])

        self.set_speed(0.35)
        if not self.gripper(BLADE_OPEN, "open"):
            return False
        if not self.go_pose(pre, "%s pre_grasp" % name):
            return False

        # The cluster body sits just under the grasp point, so keeping it as
        # world collision geometry makes the planner refuse the last few
        # centimetres of the descent -- the very motion whose whole purpose is
        # to close on this fruit. Drop it here; it comes back as an *attached*
        # object once it is cut.
        self.scene.remove_world_object(name)
        self.in_scene.discard(name)
        rospy.sleep(0.4)
        if not self.go_cartesian(grasp, "%s grasp" % name):
            return False

        rospy.loginfo("--- clamp + cut ---")
        self.gripper(BLADE_CLOSED, "closed")
        rospy.sleep(0.4)
        # hand the cluster to the cutter *before* releasing the stem
        r1 = self._att(self.attach, ROBOT_MODEL, ROBOT_EE_LINK, name,
                       BUNCH_LINK)
        rospy.loginfo("  clamped in cutter (ok=%s)", r1.ok)
        r2 = self._att(self.detach, TRELLIS_MODEL,
                       "wire_r%d_fruit" % spec["row"], name, BUNCH_LINK)
        rospy.loginfo("  stem cut (ok=%s)", r2.ok)

        # Bring the cluster back as geometry carried by the arm, so the planner
        # keeps avoiding it for the rest of the cycle. (attach_cylinder does not
        # exist in the MoveIt 1 python interface -- a box hull is what the API
        # supports.)
        held = PoseStamped()
        held.header.frame_id = "tcp_link"
        # Tool -X points roughly downward (exactly down only at PITCH=0), so
        # the cluster hangs along -X. The cross-section is oversized to absorb
        # the PITCH-induced misalignment -- collision padding, not a fruit model.
        held.pose.position.x = -(spec["hang_below_tcp"]
                                 - spec["body_len"] / 2.0)
        held.pose.orientation.w = 1.0
        self.scene.attach_box(
            ROBOT_EE_LINK, name, pose=held,
            size=(spec["body_len"] + 0.02,
                  2 * spec["r_top"] + 0.05, 2 * spec["r_top"] + 0.05),
            touch_links=["Link6", "Link5", "ee_base",
                         "left_blade", "right_blade", "tcp_link"])
        rospy.sleep(0.5)

        # Carrying the clamped cluster: slower, so the extra rigid constraint
        # link_attacher put between the fruit and the flange disturbs the
        # kinematically driven joints as little as possible.
        self.set_speed(0.2)

        rospy.loginfo("--- retreat ---")
        if not self.go_cartesian(post, "%s post_cut" % name):
            return False

        rospy.loginfo("--- carry to crate slot %d ---", slot)
        if not self.go_pose(over, "%s over_crate" % name):
            return False
        self.go_cartesian(release, "%s release" % name)

        rospy.loginfo("--- release ---")
        self.scene.remove_attached_object(ROBOT_EE_LINK, name=name)
        rospy.sleep(0.3)
        self.scene.remove_world_object(name)
        r3 = self._att(self.detach, ROBOT_MODEL, ROBOT_EE_LINK, name,
                       BUNCH_LINK)
        rospy.loginfo("  released into crate (ok=%s)", r3.ok)
        self.gripper(BLADE_OPEN, "open")
        rospy.sleep(1.5)
        self.add_settled_cluster(spec)

        self.set_speed(0.35)
        if not self.go_pose(over, "%s clear of crate" % name):
            rospy.logwarn("  could not lift clear of the crate")
        return True

    def harvest_here(self, specs, remaining):
        """Cut every cluster in `remaining` the arm can reach from right here.

        Returns (harvested, failed) name lists. `remaining` is mutated:
        anything attempted is removed from it, so the caller never retries it
        from the same standpoint.
        """
        harvested, failed = [], []
        here = []
        for spec in specs:
            if spec["name"] not in remaining:
                continue
            ok, d = self.reachable(spec)
            if ok:
                here.append((d, spec))
        here.sort(key=lambda t: t[0])

        if not here:
            rospy.loginfo("nothing within reach from this standpoint")
            return harvested, failed

        if not self.go_named("scan"):
            rospy.logerr("cannot reach scan pose")
            return harvested, [s["name"] for _, s in here]

        for d, spec in here:
            if self.slots_used >= len(CRATE_SLOTS):
                rospy.logwarn("crate full (%d slots): stopping",
                              len(CRATE_SLOTS))
                break
            remaining.discard(spec["name"])
            rospy.loginfo("cluster %s at %.2f m from the column",
                          spec["name"], d)
            if self.harvest(spec, self.slots_used):
                harvested.append(spec["name"])
                self.slots_used += 1
            else:
                failed.append(spec["name"])
                rospy.logerr("!!! %s NOT harvested, continuing", spec["name"])
                # leave the arm somewhere sane before the next attempt
                self.set_speed(0.35)
                self.go_named("scan")

        self.set_speed(0.35)
        self.go_named("scan")
        return harvested, failed

    # ----------------------------------------------------------------- run
    def check_only(self, specs):
        """IK reachability preflight from the current standpoint -- no motion."""
        checks = []
        n_reach = 0
        for spec in specs:
            ok, d = self.reachable(spec)
            if not ok:
                continue
            n_reach += 1
            pre, grasp, post = self.vine_poses(spec)
            checks += [("%s pre" % spec["name"], pre),
                       ("%s grasp" % spec["name"], grasp),
                       ("%s post" % spec["name"], post)]
        for s in range(len(CRATE_SLOTS)):
            over, rel = self.crate_poses(s, 0.35)
            checks += [("slot%d over" % s, over), ("slot%d release" % s, rel)]

        ax, ay, az = self.arm_origin()
        rospy.loginfo("arm column at (%.3f, %.3f, %.3f); %d of %d clusters "
                      "within reach", ax, ay, az, n_reach, len(specs))
        all_ok = True
        for label, pose in checks:
            target = self.solve_ik(pose)
            r = math.sqrt((pose.position.x - ax) ** 2
                          + (pose.position.y - ay) ** 2
                          + (pose.position.z - az) ** 2)
            rospy.loginfo("%-22s (%+.3f, %+.3f, %.3f)  r=%.3f  %s",
                          label, pose.position.x, pose.position.y,
                          pose.position.z, r,
                          "OK" if target else "UNREACHABLE")
            all_ok = all_ok and bool(target)
        rospy.loginfo("preflight: %s",
                      "ALL POSES REACHABLE" if all_ok else "SOME POSES FAILED")
        return all_ok

    def run_row(self, specs, row, driver=None, verify=False):
        """Work one trellis row panel by panel.

        With no driver this harvests whatever is reachable from where the robot
        already stands, which is what --no-drive is for: it keeps the arm work
        testable without depending on the base controller.
        """
        baseline = {}
        if verify:
            m = rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                       timeout=20)
            for n, p in zip(m.name, m.pose):
                baseline[n] = (p.position.x, p.position.y, p.position.z)

        row_specs = [s for s in specs if s["row"] == row]
        remaining = set(s["name"] for s in row_specs)
        harvested, failed = [], []

        if driver is None:
            h, f = self.harvest_here(specs, remaining)
            harvested += h
            failed += f
        else:
            # One stop per cluster, not per panel: see the REACH note above.
            # Parking once per 2 m panel would leave most of the fruit outside
            # the envelope and uncut.
            lane = aisle_x(row)          # aisle immediately in front of row
            for spec in sorted(row_specs, key=lambda s: s["y"]):
                if self.slots_used >= len(CRATE_SLOTS):
                    rospy.logwarn("crate full (%d slots): stopping",
                                  len(CRATE_SLOTS))
                    break
                if spec["name"] not in remaining:
                    continue        # already cut from an earlier standpoint
                rospy.loginfo("=" * 62)
                rospy.loginfo("driving abreast of %s (panel %d, y=%+.2f)",
                              spec["name"], spec["panel"], spec["y"])
                if not self.go_named("stow"):
                    rospy.logerr("cannot stow; refusing to drive")
                    break
                if not driver.drive_to_y(spec["y"], lane):
                    rospy.logwarn("could not park at %s, skipping",
                                  spec["name"])
                    continue
                rospy.sleep(0.5)
                # whatever else came within reach at this stop is fair game
                h, f = self.harvest_here(specs, remaining)
                harvested += h
                failed += f
            self.go_named("stow")

        rospy.loginfo("=" * 62)
        rospy.loginfo("harvested %d of %d clusters on row %d: %s",
                      len(harvested), len(row_specs), row,
                      ", ".join(harvested) if harvested else "none")
        if failed:
            rospy.logerr("failed: %s", ", ".join(failed))
        if remaining:
            rospy.loginfo("not attempted (out of reach or crate full): %d",
                          len(remaining))
        self.report(specs, harvested, baseline if verify else None)
        return not failed

    def report(self, specs, harvested, baseline):
        m = rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                   timeout=20)
        by_name = {s["name"]: s for s in specs}
        in_crate = 0
        for name in harvested:
            if name not in m.name:
                continue
            p = m.pose[m.name.index(name)].position
            # the crate rides on the deck, so "in the crate" is a base_link
            # question, not a world one
            bx, by, bz = self.world_to_base(p.x, p.y, p.z)
            ok = (abs(bx - CRATE_BASE_XYZ[0]) < CRATE_INNER[0] / 2 + 0.05
                  and abs(by - CRATE_BASE_XYZ[1]) < CRATE_INNER[1] / 2 + 0.05
                  and bz < CRATE_BASE_XYZ[2] + CRATE_INNER[2] + 0.10)
            in_crate += int(ok)
            rospy.loginfo("  %-16s base_link (%+.3f, %+.3f, %.3f)  %s",
                          name, bx, by, bz,
                          "IN CRATE" if ok else "NOT IN CRATE")
        rospy.loginfo("clusters in crate: %d/%d", in_crate, len(harvested))

        if baseline:
            rospy.loginfo("-- collision check: things the arm must not move --")
            moved = []
            for n, p in zip(m.name, m.pose):
                if n in by_name or n not in baseline or n == ROBOT_MODEL:
                    continue
                b = baseline[n]
                d = math.sqrt((p.position.x - b[0]) ** 2
                              + (p.position.y - b[1]) ** 2
                              + (p.position.z - b[2]) ** 2)
                if d > 0.005:
                    moved.append((n, d))
            if moved:
                for n, d in moved:
                    rospy.logerr("  %s MOVED by %.3f m -- something was hit",
                                 n, d)
            else:
                rospy.loginfo("  trellis and ground unmoved: "
                              "no unintended contact")
        rospy.loginfo("=" * 62)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check-only", action="store_true",
                    help="report IK reachability of every key pose and exit")
    ap.add_argument("--no-spawn", action="store_true",
                    help="assume the clusters are already in the world")
    ap.add_argument("--no-drive", action="store_true",
                    help="harvest only what is reachable from the current "
                         "standpoint; do not command the base")
    ap.add_argument("--row", type=int, default=0,
                    help="which trellis row to work (0..%d)" % (N_ROWS - 1))
    ap.add_argument("--verify", action="store_true",
                    help="also check nothing but the fruit moved")
    args, _ = ap.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("grape_harvester", anonymous=False)
    specs = bunch_layout()
    rospy.loginfo("vineyard block: %d clusters over %d rows",
                  len(specs), N_ROWS)

    h = GrapeHarvester()

    if args.check_only:
        h.build_planning_scene(specs)
        sys.exit(0 if h.check_only(specs) else 1)

    h.wait_for_services()
    if not args.no_spawn:
        h.spawn_bunches(specs)
    h.build_planning_scene(specs)

    driver = None
    if not args.no_drive:
        from drive import BaseDriver
        driver = BaseDriver()

    ok = h.run_row(specs, args.row, driver=driver, verify=args.verify)
    rospy.loginfo("result: %s", "SUCCESS" if ok else "FAILED")
    moveit_commander.roscpp_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
