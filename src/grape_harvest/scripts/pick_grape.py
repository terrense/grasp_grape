#!/usr/bin/env python3
"""Dobot CR10 grape harvesting from the mobile platform, under the polytunnel.

Per cluster: stow the arm, drive along the row's working lane until the cluster
is abreast of the column, unstow, cut it into the deck crate, stow, move on.

The cut cycle itself:

    approach standoff -> slide the basket over the bunch -> fire the laser
    -> cut (stem joint released, cluster handed to the head) -> retreat
    -> carry to its slot in the deck crate -> release

Severing is modelled in two halves. A Gazebo ray sensor on the head reports
what the beam is hitting and how far off it is, and the cut only happens if
the beam holds on the stem inside its effective range for LASER_DWELL
seconds. The mechanical consequence goes through gazebo_ros_link_attacher: a
cluster hangs off its row's cordon by a runtime fixed joint, and the cut
attaches it to the head *before* removing the cordon joint, so the fruit is
never in free fall.

Why one stop per cluster and why the lane hugs the row
-----------------------------------------------------
The block is planted with the fruit at 1.40-1.70 m and the rows 2.2-3.0 m
apart, and the row spacing is uneven, so there is no constant aisle geometry to
lean on. From the centre of an aisle the fruit is 1.28-1.78 m away, past even a
CR10; make_models.lane_x() therefore parks the platform LANE_STANDOFF = 0.80 m
off the row it is picking, which brings the worst cluster to about 1.23 m. That
still only leaves roughly +/-0.6 m of usable travel along the row from any one
standpoint, which is less than the spacing between clusters -- hence a stop per
cluster rather than a stop per panel.

Frames
------
The arm base and the crate both ride on the platform, so neither sits at a
constant place in the world. Vine-side poses are built in the world frame (the
fruit does not move) but their azimuth is measured from wherever the arm column
currently is; crate-side poses are built in `base_link` and pushed through TF
into the planning frame. Cluster geometry comes from make_models.bunch_layout(),
which is also what generated the world, so the scene and the motion plan cannot
drift apart.

Collision safety: every motion is planned by MoveIt against a planning scene
carrying the posts, the canopy wires, the ground and every cluster still on the
vine. The crate and the vehicle are robot links, so MoveIt already avoids them.
The polytunnel is left out on purpose -- it is a single span whose only uprights
are along the two outer edges, 1.8 m from the nearest working lane, and its arch
clears 2.2 m everywhere the platform drives. The only object deliberately
removed is the one cluster being cut, and only for the final few centimetres of
its own descent. `--verify` additionally watches the Gazebo poses of everything
the arm is not supposed to touch and reports any that moved.
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
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import (GetPositionIK, GetPositionIKRequest,
                             GetPlanningScene, GetPlanningSceneRequest,
                             GetStateValidity, GetStateValidityRequest)

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from sensor_msgs.msg import JointState, LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grape_detect import GrapeDetector
from contact_guard import ContactGuard
from make_models import (POST_H, PANEL_W, PANELS_PER_ROW, N_ROWS, ROW_LEN,
                         LANE_STANDOFF, CRATE_BASE_XYZ, CRATE_INNER,
                         CRATE_SLOTS, row_x, lane_x, canopy_z, tunnel_bounds,
                         bunch_layout, grape_bunch)

ROBOT_MODEL = "cr10_robot"
# ee_base/tcp_link are lumped into Link6 by the URDF->SDF fixed-joint merge,
# so Link6 is the link that actually exists in Gazebo.
ROBOT_EE_LINK = "Link6"
TRELLIS_MODEL = "trellis"
BUNCH_LINK = "bunch_link"
# The vehicle body as Gazebo sees it. URDF fixed joints are merged during the
# SDF conversion, so crate_link, base_link, the skid plate and the masts are
# all one link and it carries the name of the root of that chain.
VEHICLE_BODY = "base_footprint"

APPROACH = 0.15                    # standoff along the tool axis, metres
RETREAT = 0.18

# ------------------------------------------------------------------ laser
# The head has no jaws. A ray sensor on laser_link reports what the beam is
# actually hitting and how far away it is, and the cut is modelled as holding
# that beam on the target for LASER_DWELL seconds -- energy delivered, not a
# geometric assertion. LASER_RANGE is the effective cutting distance; the
# sensor itself sees further (0.60 m) so the node can tell "on target but too
# far" from "nothing in the beam".
LASER_TOPIC = "cut_laser/scan"
LASER_RANGE = 0.35                 # effective cutting distance, metres
LASER_WORKING = 0.184              # emitter to TCP; the beam is aimed at
                                   # the tool centre point, not along the
                                   # approach axis
LASER_DWELL = 1.5                  # seconds on target to sever a peduncle
LASER_SETTLE = 0.4                 # let the arm stop ringing before firing

# Teleop mode: the arm only works while the platform is standing still. The
# threshold is generous because ground truth twist is noisy on the clod track,
# and because starting a cut while the base is still rolling is the one thing
# that must not happen -- the beam would walk off the stem.
# Visual servo. The deadband is set just under the beam spot so the arm does
# not chase noise it cannot resolve; the ceiling rejects a detection that is too
# far from the prior to be the cluster we came for -- a neighbouring bunch in
# frame should be ignored, not chased.
SERVO_DEADBAND = 0.008             # m; below this the aim is good enough
SERVO_MAX = 0.25                   # m; beyond this, disbelieve the detection
SERVO_TRIES = 2

STOPPED_SPEED = 0.03               # m/s below which the base counts as parked
SETTLE_AFTER_STOP = 1.0            # s of standing still before the arm moves

# The collar is open at the bottom, so a cut bunch keeps hanging where it was
# rather than dropping onto a floor, and releasing it over the crate is just a
# matter of undoing the attacher joint -- no wrist flip, which is one less
# awkward posture for the planner to find.

CRATE_T = 0.014                    # crate floor thickness, matches the xacro
CRATE_FLOOR_Z = CRATE_BASE_XYZ[2] + CRATE_T
DROP_CLEARANCE = 0.06              # fruit hangs this far above the crate floor
                                   # before release. 0.12 was tried and the
                                   # bunch bounced back out of the crate; 0.05
                                   # left it still inside the collar. This is
                                   # the gap between those.
LANDING_SETTLE = 2.5               # s to let a dropped bunch come to rest
                                   # before anything measures where it is

# Whether fruit already in the crate goes into the planning scene. Off by
# default. The intent was to stop the arm ploughing through the pile on the
# next drop, but the head has to come down into the crate to release at all, so
# a box sitting there puts the arm in collision with its own load the instant it
# is added: every plan after the first cut then aborts with
# START_STATE_IN_COLLISION, including the stow that has to happen before the
# platform may drive. The crate is one bunch deep and the head always comes in
# from directly above, so the risk it was guarding against is small. Turn it on
# with --track-crate if the crate ever gets deep enough to matter.
TRACK_CRATE_CONTENTS = False

# Prefilter only: a cluster is attempted if its grasp point is within this
# straight-line distance of the arm base. The CR10 reaches 1300 mm to the
# flange and the cutter TCP sits 95 mm beyond it; 1.30 m leaves the last stretch
# of the envelope alone, where the postures are singular and the straight-line
# descent afterwards will not run. Whether a given pose actually solves is an IK
# question, not a radius one -- this only avoids burning planning time on
# obvious non-starters, and --check-only reports what really solves.
#
# Why the platform hugs the row instead of driving down the middle: rows are
# 2.2-3.0 m apart, so an aisle centre is 1.10-1.50 m from the vine, and with
# fruit at 1.40-1.70 m against an arm base at 0.75 m the straight-line distance
# from mid-aisle runs 1.28-1.78 m. Even a CR10 loses the far half of the block
# from there. make_models.lane_x() therefore parks LANE_STANDOFF (0.80 m) off
# the row, which brings the worst cluster back to about 1.23 m.
REACH = 1.30
REACH_MIN = 0.35

# Cutter tilted down by this much. A purely horizontal approach puts the
# forearm at the same height as the top of the cluster and every plan collides;
# tilting down keeps the wrist above the fruit, which is also how a real
# shear-type harvesting head comes onto a peduncle.
PITCH = math.radians(20.0)


def tool_quat(azimuth, pitch=PITCH):
    """Cutter pointing outward from the arm column and tilted down by `pitch`.

    Tool +Z is the approach axis (out of the Link6 flange); the beam fires
    along it and the catch basket hangs along tool +X, which points down.

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


def pose_at(x, y, z, azimuth, pitch=PITCH):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation = tool_quat(azimuth, pitch)
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
        self.arm = moveit_commander.MoveGroupCommander("arm")

        self.arm.set_planning_time(20.0)
        self.arm.set_num_planning_attempts(12)
        self.arm.set_goal_position_tolerance(0.003)
        self.arm.set_goal_orientation_tolerance(0.02)
        self.speed = 0.35
        self.set_speed(0.35)

        self.beam = None            # latest LaserScan from the cutting head
        rospy.Subscriber(LASER_TOPIC, LaserScan, self._beam_cb, queue_size=1)
        self.twist = None           # base velocity, for the teleop mode
        rospy.Subscriber("/ground_truth/state", Odometry, self._odom_cb,
                         queue_size=1)

        self.planning_frame = self.arm.get_planning_frame()

        # The platform moves, so nothing on the robot has a fixed world pose.
        # Everything crate-side is resolved through TF at the moment it is used.
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        rospy.wait_for_service("/compute_ik", timeout=60.0)
        self.ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.wait_for_service("/get_planning_scene", timeout=60.0)
        self.get_scene = rospy.ServiceProxy("/get_planning_scene",
                                            GetPlanningScene)
        self.scene_pub = rospy.Publisher("/planning_scene", PlanningScene,
                                         queue_size=1)

        self.slots_used = 0
        self.in_scene = set()
        self.track_crate = TRACK_CRATE_CONTENTS
        self.detector = None        # set by main() when --servo is given
        self.guard = None           # set by main() when --guard is given
        self.jvel = []
        rospy.Subscriber("/joint_states", JointState, self._js_cb,
                         queue_size=1)

        rospy.loginfo("planning frame: %s", self.planning_frame)
        rospy.loginfo("end effector  : %s", self.arm.get_end_effector_link())

    def set_speed(self, f):
        self.speed = f
        self.arm.set_max_velocity_scaling_factor(f)
        self.arm.set_max_acceleration_scaling_factor(f)

    # ---------------------------------------------------------------- laser
    def _beam_cb(self, msg):
        self.beam = msg

    def _odom_cb(self, msg):
        self.twist = msg.twist.twist

    def _js_cb(self, msg):
        self.jvel = [v for n, v in zip(msg.name, msg.velocity)
                     if n.startswith("joint")]

    def moving(self, still=0.02):
        return bool(self.jvel) and max(abs(v) for v in self.jvel) > still

    # Moves during which touching something is the expected outcome rather
    # than a collision.
    #
    #   grasp / release   the descent closes the collar around a bunch, and
    #                     stopping when the bunch pushes back is the move
    #                     working, not failing
    #   comply            the yield away from a contact. It necessarily starts
    #                     in one
    #   servo             a few mm of re-aim with the collar already around the
    #                     fruit, same
    #
    # Retreat is not on this list. A snag on the way out, with fruit in the
    # collar, is a real problem and stopping is the right answer.
    CONTACT_EXPECTED = ("grasp", "release", "comply", "servo")

    def contact_is_expected(self, label):
        return any(k in label for k in self.CONTACT_EXPECTED)

    def guarded_wait(self, label, timeout=40.0):
        """Wait out a non-blocking move with the force sensor watching.

        Returns True if the move ran to a stop, False if the guard tripped and
        stopped it. Completion is judged from joint velocity rather than from
        the action result, because the point of starting the move non-blocking
        is to be able to interrupt it.
        """
        t0 = rospy.Time.now()
        rate = rospy.Rate(50)
        started = False
        still_since = None
        while not rospy.is_shutdown():
            if self.guard.tripped():
                f, tq = self.guard.contact()
                self.arm.stop()
                if self.contact_is_expected(label):
                    # the collar is on the bunch. That is where this move was
                    # trying to get to, so stop here and call it arrived --
                    # pressing on would only crush fruit the arm cannot push
                    # through anyway.
                    rospy.loginfo("  contact during %s: %.1f N -- the collar "
                                  "is on the bunch, cutting from here",
                                  label, f)
                    return True
                rospy.logwarn("  CONTACT during %s: %.1f N, %.1f Nm -- "
                              "stopping", label, f, tq)
                return False
            if self.moving():
                started = True
                still_since = None
            elif started:
                if still_since is None:
                    still_since = rospy.Time.now()
                elif (rospy.Time.now() - still_since).to_sec() > 0.4:
                    return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("  %s did not settle in %.0f s", label, timeout)
                self.arm.stop()
                return True
            rate.sleep()
        return False

    def comply(self, label):
        """Give way to whatever the tool is leaning on.

        Item 3. Holding a commanded pose against fruit is the wrong behaviour
        for a head that has to close around it; yielding in proportion to the
        force is what an impedance-controlled wrist would do, and this is the
        task-level version of it.
        """
        if self.guard is None:
            return
        f, _ = self.guard.contact()
        if f < 8.0:      # free-motion noise reaches 4.5 N; stay clear of it
            return
        dx, dy, dz = self.guard.yield_offset()
        # sensor frame is the tool frame here (the sensor sits on ee_joint)
        try:
            m = self._mat(self.planning_frame, "ee_base")
        except Exception:
            return
        wx = m[0, 0] * dx + m[0, 1] * dy + m[0, 2] * dz
        wy = m[1, 0] * dx + m[1, 1] * dy + m[1, 2] * dz
        wz = m[2, 0] * dx + m[2, 1] * dy + m[2, 2] * dz
        mag = math.sqrt(wx * wx + wy * wy + wz * wz)
        if mag < 0.002:
            return
        rospy.loginfo("  compliance: %.1f N on the tool, yielding %.0f mm "
                      "before %s", f, mag * 1000, label)
        cur = self._mat(self.planning_frame, "tcp_link")
        p = Pose()
        p.position.x = cur[0, 3] + wx
        p.position.y = cur[1, 3] + wy
        p.position.z = cur[2, 3] + wz
        q = tft.quaternion_from_matrix(cur)
        p.orientation = Quaternion(*q)
        self.go_cartesian(p, "%s comply" % label)

    def base_speed(self):
        if self.twist is None:
            return 0.0
        return math.hypot(self.twist.linear.x, self.twist.linear.y)

    def beam_range(self):
        """Closest thing the beam is touching, or None if it is pointing at
        nothing inside the sensor's range."""
        if self.beam is None:
            return None
        good = [r for r in self.beam.ranges
                if self.beam.range_min < r < self.beam.range_max]
        return min(good) if good else None

    def tcp_error(self, target):
        """How far the tool actually is from where it was told to go.

        Worth measuring rather than assuming: the base is a floating joint on a
        clod track, so a plan made in the world frame can be spoiled either by
        joint tracking or by the platform settling under the arm, and the two
        need different fixes.
        """
        m = self._mat(self.planning_frame, "tcp_link")
        dx = m[0, 3] - target.position.x
        dy = m[1, 3] - target.position.y
        dz = m[2, 3] - target.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz), (dx, dy, dz),             (m[0, 3], m[1, 3], m[2, 3])

    def servo_correct(self, spec, grasp):
        """Nudge the tool until the beam is on the stem the camera can see.

        Returns the corrected pose, or the original if the camera has nothing
        useful to say. Failing to see anything is not an error: the prior
        coordinate is still a reasonable aim, and refusing to cut because the
        detector blinked would be worse than cutting where we already believed
        the stem was.
        """
        if self.detector is None:
            return grasp

        pose = grasp
        for attempt in range(SERVO_TRIES):
            rospy.sleep(0.4)                    # let the image catch up
            got = self.detector.detect()
            if got is None:
                rospy.logwarn("  servo: nothing in view, keeping the prior aim")
                return pose
            (cx, cy, cz), npix, rng, truncated = got

            # camera optical frame -> planning frame
            try:
                m = self._mat(self.planning_frame, self.detector.frame)
            except Exception as e:
                rospy.logwarn("  servo: no TF for %s (%s)",
                              self.detector.frame, e)
                return pose
            v = m.dot([cx, cy, cz, 1.0])
            seen = (v[0], v[1], v[2])

            dx = seen[0] - pose.position.x
            dy = seen[1] - pose.position.y
            dz = seen[2] - pose.position.z
            d = math.sqrt(dx * dx + dy * dy + dz * dz)

            rospy.loginfo("  servo: %d px at %.3f m, stem seen at "
                          "(%.3f, %.3f, %.3f), %.0f mm from the aim%s",
                          npix, rng, seen[0], seen[1], seen[2], d * 1000,
                          "  [height truncated, using lateral only]"
                          if truncated else "")

            if d > SERVO_MAX:
                rospy.logwarn("  servo: %.0f mm off the prior, that is not the "
                              "cluster we came for; ignoring", d * 1000)
                return pose
            if d < SERVO_DEADBAND:
                rospy.loginfo("  servo: within %.0f mm, good enough",
                              SERVO_DEADBAND * 1000)
                return pose

            corrected = copy.deepcopy(pose)
            corrected.position.x = seen[0]
            corrected.position.y = seen[1]
            if not truncated:
                corrected.position.z = seen[2]
            if not self.go_cartesian(corrected, "%s servo %d"
                                     % (spec["name"], attempt + 1)):
                rospy.logwarn("  servo: could not move onto the corrected aim")
                return pose
            pose = corrected
        return pose

    def fire(self, name, target=None):
        """Hold the beam on the peduncle long enough to sever it.

        Returns True if the beam stayed on something inside LASER_RANGE for
        LASER_DWELL seconds. This is what replaces closing the jaws: there is
        nothing to grip with, so the only question is whether the beam is
        actually landing on the stem, and the ray sensor answers it against the
        real scene rather than against an assumed pose.
        """
        rospy.sleep(LASER_SETTLE)
        if target is not None:
            e, d3, act = self.tcp_error(target)
            rospy.loginfo("  tcp is %.4f m from the commanded pose "
                          "(d=[%+.3f %+.3f %+.3f], at [%.3f %.3f %.3f])",
                          e, d3[0], d3[1], d3[2], act[0], act[1], act[2])
        d = self.beam_range()
        if d is None:
            rospy.logerr("  laser sees nothing in the beam; not firing")
            if self.beam is not None:
                rospy.logerr("  raw ranges: %s  (min=%.3f max=%.3f, %d rays)",
                             ["%.3f" % r for r in self.beam.ranges],
                             self.beam.range_min, self.beam.range_max,
                             len(self.beam.ranges))
            else:
                rospy.logerr("  no LaserScan received at all on %s",
                             LASER_TOPIC)
            try:
                m = self._mat("laser_link", self.planning_frame)
                import numpy as _np
                for nm in (name,):
                    pm = self.model_pose(nm)
                    if pm is None:
                        continue
                    v = m.dot([pm.position.x, pm.position.y,
                               pm.position.z, 1.0])
                    rospy.logerr("  %s origin in laser frame: "
                                 "(%.3f, %.3f, %.3f)  -- beam runs along +z",
                                 nm, v[0], v[1], v[2])
            except Exception as e:
                rospy.logwarn("  beam diagnostic TF failed: %s", e)
            return False
        if d > LASER_RANGE:
            rospy.logerr("  target at %.3f m is beyond the %.2f m effective "
                         "range; not firing", d, LASER_RANGE)
            return False

        rospy.loginfo("  laser ON, target at %.3f m, dwelling %.1f s",
                      d, LASER_DWELL)
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while (rospy.Time.now() - t0).to_sec() < LASER_DWELL:
            if rospy.is_shutdown():
                return False
            d = self.beam_range()
            if d is None or d > LASER_RANGE:
                rospy.logwarn("  beam came off %s at %.1f s; aborting the cut",
                              name, (rospy.Time.now() - t0).to_sec())
                return False
            rate.sleep()
        rospy.loginfo("  laser OFF, %s severed", name)
        return True

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
        # straight back along the approach axis, no lift: with the fruit at
        # 1.4-1.7 m the arm is already near full stretch, and lifting on the way
        # out extends it further instead of unloading it
        post = backed_off(x, y, z, az, RETREAT)
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
        # The collar is open at the bottom, so releasing is just detaching: the
        # bunch drops straight out. Both poses keep the normal cutting attitude.
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
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state",
                                            SetModelState)
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
                # each row has its own cordon height, so this cannot be a
                # single scene-wide constant any more
                p.position.z = spec["cordon_z"]
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

        Deliberately not included: the fruiting wires (4 mm radius, just above
        the grasp points -- adding them only produces spurious planning
        failures), the peduncles, which the beam is supposed to land on,
        and the polytunnel. The tunnel is a single span with its uprights only
        along the two outer edges, and the nearest one is 1.8 m from the
        nearest working lane against a 1.3 m arm, so it cannot be hit; its arch
        clears 2.2 m everywhere the platform drives.
        """
        x0, x1, y0, y1 = tunnel_bounds()
        self._box("ground", (x0 + x1) / 2.0, (y0 + y1) / 2.0, -0.03,
                  x1 - x0, y1 - y0, 0.05)

        for r in range(N_ROWS):
            x = row_x(r)
            for i in range(PANELS_PER_ROW + 1):
                self._cyl("post_r%d_%d" % (r, i), x,
                          -ROW_LEN / 2.0 + i * PANEL_W, POST_H / 2,
                          0.05, POST_H)
            self._box("canopy_r%d" % r, x, 0.0, canopy_z(r),
                      0.02, ROW_LEN, 0.02)

        # berry bodies only -- the top of each body sits pedu_len below its
        # row's cordon, so the head never has to plan through it
        for spec in specs:
            self._cyl(spec["name"], spec["x"], spec["y"],
                      spec["cordon_z"] - spec["pedu_len"]
                      - spec["body_len"] / 2.0,
                      spec["r_top"] * 0.9, spec["body_len"])

        rospy.sleep(1.0)
        rospy.loginfo("planning scene: %d objects", len(self.in_scene))

    def base_xy(self):
        """Vehicle position from ground truth, or None."""
        p = self.model_pose(ROBOT_MODEL)
        return None if p is None else (p.position.x, p.position.y)

    def report_base_shift(self, name):
        """How far the platform moved while the arm was working.

        It should not have moved at all -- the wheels are stopped and nothing
        commands it. Anything here is the arm pushing off something, which
        under kinematic joints it can do with unlimited force.
        """
        was = getattr(self, "_base_at_pick", None)
        now = self.base_xy()
        if was is None or now is None:
            return
        dx, dy = now[0] - was[0], now[1] - was[1]
        d = math.sqrt(dx * dx + dy * dy)
        if d < 0.02:
            return
        rospy.logwarn("  the platform moved %.3f m during %s "
                      "(dx %+.3f, dy %+.3f) -- the wheels were stopped, so "
                      "the arm pushed off something", d, name, dx, dy)

    def add_settled_cluster(self, spec):
        """Re-add a released cluster where it actually came to rest.

        The crate rides on the robot, so a cluster lying in it moves with the
        vehicle and its world-frame collision object would go stale the moment
        the platform drives off. It is therefore attached to `crate_link`
        rather than added to the world -- MoveIt then carries it along.
        """
        if not self.track_crate:
            return
        name = spec["name"]
        p = self.model_pose(name)
        if p is None:
            rospy.logwarn("  could not read %s pose; crate contents not added "
                          "to the planning scene", name)
            return
        bx, by, _ = self.world_to_base(p.position.x, p.position.y,
                                       p.position.z)

        # Lying on the floor, not standing on end. A 0.30 m bunch modelled
        # upright in a 0.20 m crate puts its top 0.17 m proud of the rim, which
        # is exactly where the collar is once the arm has let go -- the planner
        # then refuses the stow pose and the row run stops after one cluster.
        # Dropped fruit topples; the long axis goes along the crate.
        lie_z = CRATE_FLOOR_Z + spec["r_top"] + 0.01
        # keep it inside the crate footprint even if the drop scattered
        half = (CRATE_INNER[0] / 2.0 - spec["r_top"],
                CRATE_INNER[1] / 2.0 - spec["r_top"])
        bx = max(CRATE_BASE_XYZ[0] - half[0],
                 min(CRATE_BASE_XYZ[0] + half[0], bx))
        by = max(CRATE_BASE_XYZ[1] - half[1],
                 min(CRATE_BASE_XYZ[1] + half[1], by))

        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y = bx, by
        ps.pose.position.z = lie_z
        ps.pose.orientation.w = 1.0
        self.scene.attach_box("crate_link", name + "_in_crate", pose=ps,
                              size=(spec["body_len"] + 0.02,
                                    2 * spec["r_top"] + 0.02,
                                    2 * spec["r_top"] + 0.02),
                              touch_links=["crate_link", "base_link"])
        rospy.sleep(0.4)
        rospy.loginfo("  %s added to the crate contents", name)

    # ------------------------------------------------------------- motions
    # The two arm links measured brushing the crate rim while the tool is
    # down inside it. Link1 against the crate is already exempt in the SRDF.
    CRATE_BRUSH_LINKS = ("Link2", "Link3")

    def allow_crate_contact(self, on):
        """Turn collision checking against the crate on or off for the arm.

        Scoped to the crate phase by the caller. Measured penetration when the
        arm is placing fruit is about 3 mm -- the forearm clipping the rim of a
        plastic crate on its own vehicle -- and refusing to plan out of it is
        what ends the row run. Everything else, the vine included, keeps
        checking normally.
        """
        try:
            req = GetPlanningSceneRequest()
            req.components.components = (
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
            acm = self.get_scene(req).scene.allowed_collision_matrix
            names = list(acm.entry_names)
            if "crate_link" not in names:
                return
            j = names.index("crate_link")
            touched = 0
            for link in self.CRATE_BRUSH_LINKS:
                if link not in names:
                    continue
                i = names.index(link)
                acm.entry_values[i].enabled[j] = on
                acm.entry_values[j].enabled[i] = on
                touched += 1
            if not touched:
                return
            ps = PlanningScene()
            ps.is_diff = True
            ps.allowed_collision_matrix = acm
            self.scene_pub.publish(ps)
            rospy.sleep(0.3)
            rospy.loginfo("  crate contact %s for %s",
                          "allowed" if on else "checked again",
                          ", ".join(self.CRATE_BRUSH_LINKS))
        except Exception as e:
            rospy.logwarn("  could not adjust the crate collision pair: %s", e)

    def why_invalid(self, label):
        """Name the pair that is in collision, instead of guessing.

        move_group reports a rejected plan as "seems to be invalid (possibly
        due to postprocessing)" and never says what it hit.
        /check_state_validity does say, for the state the arm is actually in.
        """
        try:
            rospy.wait_for_service("/check_state_validity", timeout=5.0)
            sv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
            req = GetStateValidityRequest()
            req.robot_state = self.current_state()
            req.group_name = "arm"
            res = sv(req)
        except Exception as e:
            rospy.logwarn("  could not check state validity: %s", e)
            return
        if res.valid:
            rospy.logwarn("  %s: the start state is collision free, so the "
                          "rejected plan collides somewhere along its length, "
                          "not at the start", label)
            return
        deepest = {}
        for c in res.contacts:
            k = "%s <-> %s" % (c.contact_body_1, c.contact_body_2)
            deepest[k] = max(deepest.get(k, 0.0), c.depth)
        if not deepest:
            rospy.logwarn("  %s: start state invalid but no contact pair "
                          "reported", label)
            return
        # 1 cm of link padding on each side, so anything under about 20 mm is
        # two links passing close rather than interfering.
        rospy.logwarn("  %s: start state is IN COLLISION: %s", label,
                      "; ".join("%s %.1f mm deep" % (k, v * 1000)
                                for k, v in sorted(deepest.items(),
                                                   key=lambda kv: -kv[1])))

    def go_named(self, name, tries=3):
        rospy.loginfo("arm -> named pose %s", name)
        for attempt in range(1, tries + 1):
            self.arm.set_start_state_to_current_state()
            self.arm.set_named_target(name)
            ok = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            if ok:
                return True
            if attempt == tries:
                self.why_invalid("named pose %s" % name)
        return False

    def current_state(self):
        """Robot state *including* whatever is attached to the cutter.

        RobotCommander.get_current_state() reports joint values only. Seeding a
        collision-aware IK request with it means IK cannot see a clamped
        cluster, so it happily returns a solution that the planner then refuses
        -- and the refusal surfaces as a bare "ABORTED: TIMED_OUT", which says
        nothing about the real cause.
        """
        try:
            req = GetPlanningSceneRequest()
            req.components.components = (
                PlanningSceneComponents.ROBOT_STATE
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            return self.get_scene(req).scene.robot_state
        except rospy.ServiceException as e:
            rospy.logwarn("planning scene state unavailable (%s); falling "
                          "back to joint values only", e)
            return self.robot.get_current_state()

    def solve_ik(self, pose, avoid=True, timeout=3.0):
        """Collision-aware IK seeded from the current state.

        Going through IK explicitly and then planning in joint space is much
        more repeatable than handing OMPL a pose goal: a pose goal lets the
        planner pick any of the many IK solutions, and the arm posture it lands
        in decides whether the following straight-line descent is possible.
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = self.arm.get_end_effector_link()
        req.ik_request.robot_state = self.current_state()
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
            if self.guard is not None:
                self.guard.reset()
                self.arm.go(wait=False)
                ok = self.guarded_wait(label)
            else:
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
            # compute_cartesian_path returns a *path*, not a trajectory: the
            # waypoints carry no timing. Handing that straight to execute() is
            # accepted and reported as success, and the arm does not move --
            # which is exactly how this failed for weeks. Every "cartesian
            # coverage 100%" followed by a tool 0.5 m from where it was told to
            # be was this. Retime it against the current state first.
            plan = self.arm.retime_trajectory(
                self.robot.get_current_state(), plan,
                velocity_scaling_factor=self.speed,
                acceleration_scaling_factor=self.speed)
            if self.guard is not None:
                self.guard.reset()
                self.arm.execute(plan, wait=False)
                held = self.guarded_wait(label)
                self.arm.stop()
                if held:
                    return True
                return False
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
    def abandon(self, spec):
        """Let go of a cluster that is already cut and still in the cutter.

        Without this the arm keeps carrying it: the Gazebo joint and the MoveIt
        attached box both survive a failed retreat, so every later plan is made
        with a phantom cluster welded to the flange. That is what turned one
        failed retreat into three in the first run of this code.
        """
        # abandon can fire from inside the crate phase, which has the
        # crate pair open; close it again rather than leaving the arm free to
        # plan through the crate for the rest of the run
        self.allow_crate_contact(False)
        name = spec["name"]
        rospy.logwarn("  abandoning %s: releasing it where the arm stands",
                      name)
        self.scene.remove_attached_object(ROBOT_EE_LINK, name=name)
        rospy.sleep(0.3)
        self.scene.remove_world_object(name)
        self._att(self.detach, ROBOT_MODEL, ROBOT_EE_LINK, name, BUNCH_LINK)
        rospy.sleep(0.5)

    def harvest(self, spec, slot):
        name = spec["name"]
        pre, grasp, post = self.vine_poses(spec)
        over, release = self.crate_poses(slot, spec["hang_below_tcp"])
        self._base_at_pick = self.base_xy()
        rospy.loginfo("########### harvesting %s  (%.2f, %.2f, %.2f) #######",
                      name, spec["x"], spec["y"], spec["grasp_z"])

        self.set_speed(0.35)
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

        self.comply("the cut")
        rospy.loginfo("--- laser cut ---")
        grasp = self.servo_correct(spec, grasp)
        if not self.fire(name, grasp):
            # nothing has been cut, so the cluster is still on the vine and
            # there is nothing to put down: a plain failure, not an abandon
            return False
        # hand the cluster to the head *before* releasing the stem, so it never
        # free-falls out of the basket
        r1 = self._att(self.attach, ROBOT_MODEL, ROBOT_EE_LINK, name,
                       BUNCH_LINK)
        rospy.loginfo("  held in the basket (ok=%s)", r1.ok)
        r2 = self._att(self.detach, TRELLIS_MODEL,
                       "wire_r%d_fruit" % spec["row"], name, BUNCH_LINK)
        rospy.loginfo("  stem cut (ok=%s)", r2.ok)
        if self.guard is not None:
            # The bunch now hangs on the sensor. Re-fit the gravity model
            # rather than taring: its weight swings round the sensor frame as
            # the wrist moves, the same way the head's does, and a fixed bias
            # would leave a few newtons of phantom force through the retreat.
            rospy.sleep(0.4)
            self.guard.calibrate()

        # Bring the cluster back as geometry carried by the arm, so the planner
        # keeps avoiding it for the rest of the cycle. (attach_cylinder does not
        # exist in the MoveIt 1 python interface -- a box hull is what the API
        # supports.)
        held = PoseStamped()
        held.header.frame_id = "tcp_link"
        # Tool +X points *down*, so the cluster hangs along +X. tool_quat is
        # Rz(az)*Ry(90+PITCH) and the X column of that comes out at world
        # (-sin(PITCH)cos(az), -sin(PITCH)sin(az), -cos(PITCH)) -- z = -0.94 at
        # PITCH = 20 deg.
        #
        # The fixed-pedestal version had this negative, which hung the
        # collision box 0.29 m *above* the cutter instead of below it. Nothing
        # caught it while every cluster had the same peduncle length and the
        # box stopped just short of the canopy wire; once bunch_layout()
        # randomised the grasp height the box started colliding with
        # canopy_r<n> on the retreat, and the resulting unsampleable goal was
        # reported only as ABORTED: TIMED_OUT.
        #
        # The cross-section is oversized to absorb the PITCH-induced
        # misalignment -- collision padding, not a fruit model.
        held.pose.position.x = (spec["hang_below_tcp"]
                                - spec["body_len"] / 2.0)
        held.pose.orientation.w = 1.0
        # Link2..Link6 are all in touch_links, and the cross-section padding is
        # small, because of how this arm has to stand to reach 1.4-1.7 m fruit:
        # the shoulder is at 0.75 m, so the arm points *up* and the forearm runs
        # alongside whatever is hanging from the cutter. Contact between the
        # carried cluster and the forearm is geometrically unavoidable and
        # physically just a brush, but MoveIt counts it as a collision and
        # refuses every retreat -- which is exactly what the first CR10 run did,
        # cutting 3 clusters and failing to retreat with all 3. The upper arm
        # had to be added too: the cluster centre sits about 0.27 m below the
        # tool, i.e. around 1.34 m, which is between the shoulder at 0.75 m and
        # the wrist at 1.6 m, so it runs alongside Link2 as well. Only Link1 is
        # left out -- fruit down at the shoulder means the plan is genuinely
        # wrong and the run should stop.
        #
        # The cost of this is real: the sim no longer catches the cutter
        # pressing fruit against the arm. That is an acceptable trade for a
        # compliant bunch on a stem, but it would not be for a rigid payload.
        self.scene.attach_box(
            ROBOT_EE_LINK, name, pose=held,
            size=(spec["body_len"] + 0.02,
                  2 * spec["r_top"] + 0.02, 2 * spec["r_top"] + 0.02),
            touch_links=["Link6", "Link5", "Link4", "Link3", "Link2",
                         "ee_base", "laser_link", "tcp_link",
                         "ee_camera_link", "catch_basket"])
        rospy.sleep(0.5)

        # Carrying the clamped cluster: slower, so the extra rigid constraint
        # link_attacher put between the fruit and the flange disturbs the
        # kinematically driven joints as little as possible.
        self.set_speed(0.2)

        # Past this point the cluster is off the vine and in the cutter, so a
        # failure cannot just return -- it has to put the fruit down first.
        rospy.loginfo("--- retreat ---")
        if not self.go_cartesian(post, "%s post_cut" % name):
            self.abandon(spec)
            return False

        rospy.loginfo("--- carry to crate slot %d ---", slot)
        self.allow_crate_contact(True)
        if not self.go_pose(over, "%s over_crate" % name):
            self.abandon(spec)
            return False
        self.go_cartesian(release, "%s release" % name)

        rospy.loginfo("--- release ---")
        self.scene.remove_attached_object(ROBOT_EE_LINK, name=name)
        rospy.sleep(0.3)
        self.scene.remove_world_object(name)
        r3 = self._att(self.detach, ROBOT_MODEL, ROBOT_EE_LINK, name,
                       BUNCH_LINK)
        if self.guard is not None:
            # empty again: back to the head on its own
            rospy.sleep(0.4)
            self.guard.calibrate()
        rospy.loginfo("  released into the crate (ok=%s)", r3.ok)
        rospy.sleep(LANDING_SETTLE)

        # Stand it in its slot, then fix it to the vehicle. Without the fix a
        # row's worth of fruit shakes back out of an open crate on the clod
        # track (six cut in one run, four left on the ground behind the
        # platform, one flung 100 m); without the placing first, the fixed
        # joint has to resolve whatever interpenetration the drop left behind
        # and it resolves it into the chassis.
        self.stow_in_crate(spec, slot)
        r4 = self._att(self.attach, ROBOT_MODEL, VEHICLE_BODY, name,
                       BUNCH_LINK)
        rospy.loginfo("  riding in the crate (ok=%s)", r4.ok)
        self.report_base_shift(name)

        # Lift clear first, THEN tell the planner about the fruit that is now
        # lying in the crate. Doing it the other way round drops a collision
        # object around the head while the head is still down inside the crate,
        # and the next plan starts in collision with it -- which is what made
        # "clear of crate" fail on the first cut that otherwise worked.
        self.set_speed(0.35)
        if not self.go_pose(over, "%s clear of crate" % name):
            rospy.logwarn("  could not lift clear of the crate")
        self.allow_crate_contact(False)
        self.add_settled_cluster(spec)
        return True

    def stow_in_crate(self, spec, slot):
        """Set the bunch down in its assigned slot, upright on the floor."""
        dx, dy = CRATE_SLOTS[slot % len(CRATE_SLOTS)]
        # model origin is the top of the peduncle, so the whole bunch sits
        # above it: floor + peduncle + body puts the berries on the floor
        bz = CRATE_FLOOR_Z + spec["pedu_len"] + spec["body_len"]
        wx, wy, wz = self.base_to_world(CRATE_BASE_XYZ[0] + dx,
                                        CRATE_BASE_XYZ[1] + dy, bz)
        st = ModelState()
        st.model_name = spec["name"]
        st.reference_frame = "world"
        st.pose.position.x, st.pose.position.y, st.pose.position.z = wx, wy, wz
        st.pose.orientation.w = 1.0
        try:
            self.set_state(st)
            rospy.loginfo("  set down in slot %d", slot)
        except rospy.ServiceException as e:
            rospy.logwarn("  could not place %s in the crate: %s",
                          spec["name"], e)

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
            lane = lane_x(row)           # LANE_STANDOFF off the row, not mid-aisle
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

    def run_teleop(self, specs, verify=False):
        """Somebody else drives; the arm decides when there is work.

        The split is deliberate: the base is the part a person has intuitions
        about (how close to the vine, when to stop), and the arm is the part
        that benefits from not being hand-flown. So the arm stays stowed
        whenever the platform is rolling, and the moment it stops it looks for
        anything in range and cuts it.

        There is no row changing here and no steering: see teleop_base.py for
        why the lane matters that much.
        """
        baseline = {}
        if verify:
            m = rospy.wait_for_message("/gazebo/model_states", ModelStates,
                                       timeout=20)
            for n, p in zip(m.name, m.pose):
                baseline[n] = (p.position.x, p.position.y, p.position.z)

        remaining = set(x["name"] for x in specs)
        harvested, failed = [], []
        rospy.loginfo("=" * 62)
        rospy.loginfo("TELEOP: drive with teleop_base.py; the arm cuts whatever")
        rospy.loginfo("comes within %.2f m while the platform is stopped.",
                      REACH)
        rospy.loginfo("=" * 62)
        self.go_named("stow")

        stowed = True
        parked_since = None
        idle_note = rospy.Time.now()
        rate = rospy.Rate(4)

        while not rospy.is_shutdown():
            if self.slots_used >= len(CRATE_SLOTS):
                rospy.logwarn_throttle(
                    20.0, "crate full (%d slots): drive on, but nothing more "
                    "can be cut", len(CRATE_SLOTS))
                rate.sleep()
                continue

            if self.base_speed() > STOPPED_SPEED:
                parked_since = None
                if not stowed:
                    rospy.loginfo("platform moving: stowing the arm")
                    self.set_speed(0.35)
                    self.go_named("stow")
                    stowed = True
                rate.sleep()
                continue

            if parked_since is None:
                parked_since = rospy.Time.now()
            if (rospy.Time.now() - parked_since).to_sec() < SETTLE_AFTER_STOP:
                rate.sleep()
                continue

            here = []
            for spec in specs:
                if spec["name"] not in remaining:
                    continue
                ok, d = self.reachable(spec)
                if ok:
                    here.append((d, spec))
            if not here:
                if (rospy.Time.now() - idle_note).to_sec() > 15.0:
                    rospy.loginfo("parked, nothing in reach: drive on")
                    idle_note = rospy.Time.now()
                rate.sleep()
                continue

            here.sort(key=lambda t: t[0])
            d, spec = here[0]
            rospy.loginfo("-" * 62)
            rospy.loginfo("in range: %s at %.2f m", spec["name"], d)
            remaining.discard(spec["name"])
            stowed = False
            if self.harvest(spec, self.slots_used):
                harvested.append(spec["name"])
                self.slots_used += 1
                rospy.loginfo("cut %d so far; %d crate slots left",
                              len(harvested),
                              len(CRATE_SLOTS) - self.slots_used)
            else:
                failed.append(spec["name"])
                rospy.logerr("!!! %s not cut", spec["name"])
            self.set_speed(0.35)
            self.go_named("stow")
            stowed = True
            self.report(specs, harvested, baseline if verify else None)

        rospy.loginfo("teleop session over: %d cut, %d failed",
                      len(harvested), len(failed))
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
            # The cluster's model origin is the TOP of its peduncle, so even
            # sitting on the crate floor the origin is a whole bunch-length
            # above it (~0.92 m in base_link against a 0.77 m ceiling). Compare
            # the berry body centre instead, or every successful drop reads as
            # a miss.
            spec = by_name.get(name)
            drop = 0.0
            if spec:
                drop = spec["pedu_len"] + spec["body_len"] / 2.0
            # the crate rides on the deck, so "in the crate" is a base_link
            # question, not a world one
            bx, by, bz = self.world_to_base(p.x, p.y, p.z - drop)
            ok = (abs(bx - CRATE_BASE_XYZ[0]) < CRATE_INNER[0] / 2 + 0.05
                  and abs(by - CRATE_BASE_XYZ[1]) < CRATE_INNER[1] / 2 + 0.05
                  and bz < CRATE_BASE_XYZ[2] + CRATE_INNER[2] + 0.10)
            in_crate += int(ok)
            rospy.loginfo("  %-16s body centre in base_link (%+.3f, %+.3f, %.3f)  %s",
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
    ap.add_argument("--teleop", action="store_true",
                    help="a human drives the base; the arm cuts whatever comes "
                         "within reach whenever the platform is stopped")
    ap.add_argument("--row", type=int, default=0,
                    help="which trellis row to work (0..%d)" % (N_ROWS - 1))
    ap.add_argument("--guard", action="store_true",
                    help="watch the wrist force sensor: stop a motion that runs "
                         "into something, and yield to contact before cutting")
    ap.add_argument("--servo", action="store_true",
                    help="correct the aim from the eye-in-hand camera before "
                         "firing, instead of trusting the prior coordinate")
    ap.add_argument("--track-crate", action="store_true",
                    help="put fruit already in the crate into the planning "
                         "scene (see TRACK_CRATE_CONTENTS)")
    ap.add_argument("--verify", action="store_true",
                    help="also check nothing but the fruit moved")
    args, _ = ap.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("grape_harvester", anonymous=False)
    specs = bunch_layout()
    rospy.loginfo("vineyard block: %d clusters over %d rows",
                  len(specs), N_ROWS)

    h = GrapeHarvester()
    if args.track_crate:
        h.track_crate = True
    if args.guard:
        h.guard = ContactGuard()
        if h.guard.ready(10.0):
            rospy.loginfo("contact guard ON: trip at %.0f N, compliance %.1f "
                          "mm/N", ContactGuard.TRIP_FORCE,
                          ContactGuard.COMPLIANCE * 1000)
        else:
            rospy.logerr("no wrench on /ee_ft; running without the guard")
            h.guard = None
    if args.servo:
        h.detector = GrapeDetector()
        rospy.loginfo("visual servo ON: the aim comes from the camera, "
                      "not from the scene generator")
        rospy.sleep(1.5)            # let the first frames arrive

    if args.check_only:
        h.build_planning_scene(specs)
        sys.exit(0 if h.check_only(specs) else 1)

    h.wait_for_services()
    if not args.no_spawn:
        h.spawn_bunches(specs)
    h.build_planning_scene(specs)

    if args.teleop:
        ok = h.run_teleop(specs, verify=args.verify)
        rospy.loginfo("result: %s", "SUCCESS" if ok else "FAILED")
        moveit_commander.roscpp_shutdown()
        sys.exit(0 if ok else 1)

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
