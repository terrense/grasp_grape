#!/usr/bin/env python3
"""Dobot CR5 grape-cluster harvesting demo.

Harvests every cluster on the fruiting wire, one at a time. Per cluster:

    approach standoff -> descend onto the peduncle -> close blades (clamp)
    -> cut (stem joint released, cluster handed to the cutter) -> retreat
    -> carry to its slot in the crate -> open (release)

"Cutting" is modelled with gazebo_ros_link_attacher: a cluster hangs off
trellis::wire_fruit by a runtime fixed joint; the cut attaches it to the robot
flange and *then* removes the wire joint, so the fruit is never in free fall.

Collision safety: every motion is planned by MoveIt against a planning scene
that carries the posts, the canopy wire, the crate, the clusters still on the
vine, and the clusters already lying in the crate. The only object deliberately
removed is the one cluster being cut, and only for the final few centimetres of
its own descent. `--verify` additionally watches the Gazebo poses of everything
the arm is not supposed to touch and reports any that moved.

Run with --check-only to just report IK reachability of every key pose.
"""
import argparse
import copy
import math
import os
import sys
import time

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

import moveit_commander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_models import (VINE_X, WIRE_Z, POST_Y, POST_H, PEDU_LEN, BODY_LEN,
                         BUNCH_Y, CRATE_XY, CRATE_H, GRASP_Z,
                         BUNCH_DROP_BELOW_TCP)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ROBOT_MODEL = "cr5_robot"
# ee_base/tcp_link are lumped into Link6 by the URDF->SDF fixed-joint merge,
# so Link6 is the link that actually exists in Gazebo.
ROBOT_EE_LINK = "Link6"
TRELLIS_MODEL = "trellis"
WIRE_LINK = "wire_fruit"
BUNCH_LINK = "bunch_link"

APPROACH = 0.15                    # standoff along the tool axis, metres
RETREAT = 0.18

BLADE_OPEN = 0.042
BLADE_CLOSED = 0.002               # slight squeeze on the 12 mm peduncle

# Where each harvested cluster is set down inside the crate, as an x offset
# from the crate centre. Keeps the clusters from piling onto each other.
# Slots must be at least one cluster diameter (~0.14 m) apart or the
# fruit piles up on itself in the crate.
CRATE_SLOT_DX = [-0.14, 0.0, 0.14]

# Cutter tilted down by this much. A purely horizontal approach puts the
# forearm at the same height as the top of the cluster and every plan collides;
# tilting down keeps the wrist above the fruit, which is also how a real
# shear-type harvesting head comes onto a peduncle.
PITCH = math.radians(20.0)


def tool_quat(azimuth, pitch=PITCH):
    """Cutter pointing outward from the base and tilted down by `pitch`.

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


def bunch_name(i):
    return "grape_%d" % i


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

        rospy.wait_for_service("/compute_ik", timeout=60.0)
        self.ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        rospy.loginfo("planning frame: %s", self.arm.get_planning_frame())
        rospy.loginfo("end effector  : %s", self.arm.get_end_effector_link())

    def set_speed(self, f):
        self.arm.set_max_velocity_scaling_factor(f)
        self.arm.set_max_acceleration_scaling_factor(f)

    # ------------------------------------------------------- key poses
    def vine_poses(self, i):
        """(pre_grasp, grasp, post_cut) for the cluster at BUNCH_Y[i].

        The tool always advances radially outward from the base, so each
        cluster gets its own azimuth rather than a fixed +x approach.
        """
        y = BUNCH_Y[i]
        az = math.atan2(y, VINE_X)
        grasp = pose_at(VINE_X, y, GRASP_Z, az)
        pre = backed_off(VINE_X, y, GRASP_Z, az, APPROACH)
        post = backed_off(VINE_X, y, GRASP_Z, az, RETREAT, lift=0.05)
        return pre, grasp, post

    def crate_poses(self, slot):
        """(over_crate, release) for one crate slot."""
        x = CRATE_XY[0] + CRATE_SLOT_DX[slot % len(CRATE_SLOT_DX)]
        y = CRATE_XY[1]
        az = math.atan2(y, x)
        drop_z = CRATE_H + BUNCH_DROP_BELOW_TCP + 0.06
        # Same PITCH as the cut: the cluster is rigidly clamped at the moment
        # of the cut, so holding the tool attitude constant keeps it upright.
        return pose_at(x, y, drop_z + 0.12, az), pose_at(x, y, drop_z, az)

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
    def spawn_bunches(self):
        """Spawn the clusters and hang them on the fruiting wire.

        Physics is paused across spawn+attach: a free body would otherwise
        fall for the few hundred ms it takes the attach service to run.
        """
        sdf = open(os.path.join(PKG_DIR, "models", "grape_bunch",
                                "model.sdf")).read()
        self.pause()
        try:
            for i, y in enumerate(BUNCH_Y):
                name = bunch_name(i)
                p = Pose()
                p.position.x, p.position.y, p.position.z = VINE_X, y, WIRE_Z
                self.spawn(name, sdf, "", p, "world")
                # wall clock, not rospy.sleep: /clock is frozen while physics
                # is paused, so a sim-time sleep would never return
                time.sleep(0.2)
                r = self._att(self.attach, TRELLIS_MODEL, WIRE_LINK,
                              name, BUNCH_LINK)
                rospy.loginfo("hung %s on the wire (ok=%s)", name, r.ok)
                time.sleep(0.1)
        finally:
            self.unpause()
        rospy.sleep(0.5)

    def _box(self, name, x, y, z, sx, sy, sz):
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.w = 1.0
        self.scene.add_box(name, ps, size=(sx, sy, sz))

    def _cyl(self, name, x, y, z, radius, height):
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.w = 1.0
        self.scene.add_cylinder(name, ps, height=height, radius=radius)

    def build_planning_scene(self):
        """Collision geometry for MoveIt.

        Deliberately *not* included: the fruiting wire (8 mm, 75 mm above the
        grasp point -- adding it only produces spurious planning failures) and
        the peduncles, which the blades are supposed to close on.
        """
        self._box("ground", 0, 0, -0.03, 4.0, 4.0, 0.05)
        self._cyl("post_n", VINE_X, POST_Y, POST_H / 2, 0.05, POST_H)
        self._cyl("post_s", VINE_X, -POST_Y, POST_H / 2, 0.05, POST_H)
        self._box("canopy_wire", VINE_X, 0, 1.62, 0.02, 2 * POST_Y, 0.02)
        self._box("crate", CRATE_XY[0], CRATE_XY[1], CRATE_H / 2,
                  0.48, 0.32, CRATE_H)

        # berry bodies only -- the top of the body sits 75 mm below the grasp
        # point, so the blades never have to plan through it
        body_top = WIRE_Z - PEDU_LEN
        for i in range(len(BUNCH_Y)):
            self._cyl(bunch_name(i), VINE_X, BUNCH_Y[i],
                      body_top - BODY_LEN / 2, 0.065, BODY_LEN)
        rospy.sleep(1.0)
        rospy.loginfo("planning scene: %s",
                      sorted(self.scene.get_known_object_names()))

    def add_settled_cluster(self, name):
        """Re-add a released cluster where it actually came to rest, so the
        arm plans around the fruit already in the crate."""
        p = self.model_pose(name)
        if p is None:
            rospy.logwarn("  could not read %s pose; crate contents not added "
                          "to the planning scene", name)
            return
        # model origin is the top of the peduncle; the berry body hangs below
        self._cyl(name + "_in_crate",
                  p.position.x, p.position.y,
                  p.position.z - PEDU_LEN - BODY_LEN / 2.0,
                  0.075, BODY_LEN)
        rospy.sleep(0.4)
        rospy.loginfo("  %s added to the planning scene where it landed", name)

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
        planner pick any of the many IK solutions, and the arm posture it
        lands in decides whether the following straight-line descent is
        possible at all.
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = "cr5_arm"
        req.ik_request.ik_link_name = self.arm.get_end_effector_link()
        req.ik_request.robot_state = self.robot.get_current_state()
        req.ik_request.avoid_collisions = avoid
        ps = PoseStamped()
        ps.header.frame_id = self.arm.get_planning_frame()
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
    def harvest(self, i, slot):
        name = bunch_name(i)
        pre, grasp, post = self.vine_poses(i)
        over, release = self.crate_poses(slot)
        rospy.loginfo("################ harvesting %s (y=%+.2f) ############",
                      name, BUNCH_Y[i])

        self.set_speed(0.35)
        if not self.gripper(BLADE_OPEN, "open"):
            return False
        if not self.go_pose(pre, "%s pre_grasp" % name):
            return False

        # The cluster body sits 75 mm under the grasp point, so keeping it as
        # world collision geometry makes the planner refuse the last few
        # centimetres of the descent -- the very motion whose whole purpose is
        # to close on this fruit. Drop it here; it comes back as an *attached*
        # object once it is cut.
        self.scene.remove_world_object(name)
        rospy.sleep(0.4)
        if not self.go_cartesian(grasp, "%s grasp" % name):
            return False

        rospy.loginfo("--- clamp + cut ---")
        self.gripper(BLADE_CLOSED, "closed")
        rospy.sleep(0.4)
        # hand the cluster to the cutter *before* releasing the stem
        r1 = self._att(self.attach, ROBOT_MODEL, ROBOT_EE_LINK, name, BUNCH_LINK)
        rospy.loginfo("  clamped in cutter (ok=%s)", r1.ok)
        r2 = self._att(self.detach, TRELLIS_MODEL, WIRE_LINK, name, BUNCH_LINK)
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
        held.pose.position.x = -(BUNCH_DROP_BELOW_TCP - BODY_LEN / 2.0)
        held.pose.orientation.w = 1.0
        self.scene.attach_box(
            ROBOT_EE_LINK, name, pose=held,
            size=(BODY_LEN + 0.02, 0.19, 0.19),
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
        r3 = self._att(self.detach, ROBOT_MODEL, ROBOT_EE_LINK, name, BUNCH_LINK)
        rospy.loginfo("  released into crate (ok=%s)", r3.ok)
        self.gripper(BLADE_OPEN, "open")
        rospy.sleep(1.5)
        self.add_settled_cluster(name)

        self.set_speed(0.35)
        if not self.go_pose(over, "%s clear of crate" % name):
            rospy.logwarn("  could not lift clear of the crate")
        return True

    # ----------------------------------------------------------------- run
    def check_only(self):
        """IK reachability preflight -- no motion."""
        all_ok = True
        checks = []
        for i in range(len(BUNCH_Y)):
            pre, grasp, post = self.vine_poses(i)
            checks += [("b%d pre_grasp" % i, pre), ("b%d grasp" % i, grasp),
                       ("b%d post_cut" % i, post)]
        for s in range(len(BUNCH_Y)):
            over, rel = self.crate_poses(s)
            checks += [("slot%d over" % s, over), ("slot%d release" % s, rel)]
        for label, pose in checks:
            target = self.solve_ik(pose)
            r = math.sqrt(pose.position.x ** 2 + pose.position.y ** 2
                          + (pose.position.z - 0.897) ** 2)
            rospy.loginfo("%-16s (%+.3f, %+.3f, %.3f)  r=%.3f  %s",
                          label, pose.position.x, pose.position.y,
                          pose.position.z, r,
                          "OK" if target else "UNREACHABLE")
            all_ok = all_ok and bool(target)
        rospy.loginfo("preflight: %s",
                      "ALL POSES REACHABLE" if all_ok else "SOME POSES FAILED")
        return all_ok

    def run(self, verify=False):
        baseline = {}
        if verify:
            m = rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                       timeout=20)
            for n, p in zip(m.name, m.pose):
                baseline[n] = (p.position.x, p.position.y, p.position.z)

        if not self.go_named("scan"):
            rospy.logerr("cannot reach scan pose")
            return False

        harvested, failed = [], []
        for i in range(len(BUNCH_Y)):
            if self.harvest(i, i):
                harvested.append(bunch_name(i))
            else:
                failed.append(bunch_name(i))
                rospy.logerr("!!! %s NOT harvested, continuing", bunch_name(i))
                # leave the arm somewhere sane before the next attempt
                self.set_speed(0.35)
                self.go_named("scan")

        self.set_speed(0.35)
        self.go_named("scan")

        rospy.loginfo("=" * 62)
        rospy.loginfo("harvested %d/%d: %s", len(harvested), len(BUNCH_Y),
                      ", ".join(harvested) if harvested else "none")
        if failed:
            rospy.logerr("failed: %s", ", ".join(failed))
        self.report(baseline if verify else None)
        return not failed

    def report(self, baseline):
        m = rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                   timeout=20)
        crate_x, crate_y = CRATE_XY
        in_crate = 0
        for i in range(len(BUNCH_Y)):
            n = bunch_name(i)
            if n not in m.name:
                continue
            p = m.pose[m.name.index(n)].position
            ok = (abs(p.x - crate_x) < 0.25 and abs(p.y - crate_y) < 0.20
                  and p.z < 0.7)
            in_crate += int(ok)
            rospy.loginfo("  %-9s at (%+.3f, %+.3f, %.3f)  %s", n, p.x, p.y, p.z,
                          "IN CRATE" if ok else "NOT IN CRATE")
        rospy.loginfo("clusters in crate: %d/%d", in_crate, len(BUNCH_Y))

        if baseline:
            rospy.loginfo("-- collision check: things the arm must not move --")
            moved = []
            for n, p in zip(m.name, m.pose):
                if n.startswith("grape") or n not in baseline:
                    continue
                b = baseline[n]
                d = math.sqrt((p.position.x - b[0]) ** 2
                              + (p.position.y - b[1]) ** 2
                              + (p.position.z - b[2]) ** 2)
                if d > 0.005:
                    moved.append((n, d))
            if moved:
                for n, d in moved:
                    rospy.logerr("  %s MOVED by %.3f m -- something was hit", n, d)
            else:
                rospy.loginfo("  trellis, crate and ground all unmoved: "
                              "no unintended contact")
        rospy.loginfo("=" * 62)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check-only", action="store_true",
                    help="report IK reachability of every key pose and exit")
    ap.add_argument("--no-spawn", action="store_true",
                    help="assume the clusters are already in the world")
    ap.add_argument("--verify", action="store_true",
                    help="also check nothing but the fruit moved")
    args, _ = ap.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("grape_harvester", anonymous=False)
    h = GrapeHarvester()

    if args.check_only:
        h.build_planning_scene()
        sys.exit(0 if h.check_only() else 1)

    h.wait_for_services()
    if not args.no_spawn:
        h.spawn_bunches()
    h.build_planning_scene()
    ok = h.run(verify=args.verify)
    rospy.loginfo("result: %s", "SUCCESS" if ok else "FAILED")
    moveit_commander.roscpp_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
