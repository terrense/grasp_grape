#!/usr/bin/env python3
"""Base motion for the vineyard platform.

The aisles are straight and axis aligned, so this is a lane keeper rather than
a planner: drive along +/-y at a fixed x, holding heading. Feedback comes from
/ground_truth/state for the same reason world_tf.py exists -- wheel odometry
slips too much on the clod track to place the arm afterwards.

Run standalone for the clearance test:
    rosrun grape_harvest drive.py _row:=0 _stow_first:=true

Add _excite:=true to run the VIO excitation pulses first; VINS-Mono will not
initialise from a constant-velocity crawl.
"""
import math
import os
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_models import lane_x, ROW_LEN, row_x, N_ROWS, LANE_STANDOFF


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BaseDriver(object):
    V_MAX = 0.45          # m/s; a harvesting platform crawls
    V_MIN = 0.10          # m/s; floor under the approach, see the note in
                          # drive_to_y about the last few centimetres
    HALF_TRACK = 0.32     # m; must match wheel_dy in the xacro
    W_MAX = 0.7
    K_ALONG = 0.9
    K_LOOK = 2.2          # how hard a lane offset bends the aim heading
    AIM_MAX = 0.6         # rad; cap so it never aims across the row
    K_YAW = 2.0
    # No integral term here on purpose. The plant already integrates (heading
    # becomes lateral position) and the aim heading below is proportional
    # action on the lateral error, so an integrator on the heading error would
    # be the second one in series behind the vehicle's lag. Measured with one
    # in: 10.7 m off lane and five timeouts in a row run, against zero timeouts
    # without. If a steady lateral bias ever needs removing, integrate the
    # cross-track error into the aim heading instead -- one integrator, not two.
    LANE_WARN = 0.12      # m; lane error worth warning about, because it
                          # comes straight off the arm reach margin

    def __init__(self, cmd_topic="/cmd_vel"):
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.pose = None      # (x, y, yaw)
        self.vel = (0.0, 0.0)  # (linear speed, yaw rate) from ground truth
        self.wheels = {}       # wheel joint -> (velocity, effort)
        rospy.Subscriber("/ground_truth/state", Odometry, self._cb,
                         queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self._js_cb,
                         queue_size=1)
        rospy.loginfo("waiting for ground truth ...")
        while self.pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("base at (%.2f, %.2f) yaw %.1f deg",
                      self.pose[0], self.pose[1], math.degrees(self.pose[2]))

    def _js_cb(self, msg):
        for i, n in enumerate(msg.name):
            if n.startswith("wheel_"):
                v = msg.velocity[i] if i < len(msg.velocity) else 0.0
                e = msg.effort[i] if i < len(msg.effort) else 0.0
                self.wheels[n] = (v, e)

    def why_stuck(self, cmd):
        """Wheels turning or not: that is the whole question."""
        spin = max((abs(v) for v, _ in self.wheels.values()), default=0.0)
        rospy.logwarn("  commanded %.2f m/s, ground truth %.3f m/s, "
                      "fastest wheel %.2f rad/s (= %.2f m/s at the rim)",
                      cmd, self.vel[0], spin, spin * 0.16)
        for n in sorted(self.wheels):
            v, e = self.wheels[n]
            rospy.logwarn("    %-16s %+7.2f rad/s  effort %+8.1f", n, v, e)
        if spin * 0.16 > 0.05 and self.vel[0] < 0.02:
            rospy.logwarn("  -> wheels turning, platform not moving: wedged "
                          "or slipping, not a command problem")
        elif spin * 0.16 <= 0.05:
            rospy.logwarn("  -> wheels not turning: the command is not "
                          "becoming torque")

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.vel = (math.hypot(msg.twist.twist.linear.x,
                               msg.twist.twist.linear.y),
                    msg.twist.twist.angular.z)

    def stop(self):
        t = Twist()
        for _ in range(6):
            self.pub.publish(t)
            rospy.sleep(0.05)

    def drive_to_y(self, target_y, lane_x, heading=math.pi / 2,
                   tol=0.05, timeout=180.0):
        """Drive along the aisle to world y=target_y, holding x=lane_x."""
        rospy.loginfo("drive -> y=%+.2f (lane x=%+.2f)", target_y, lane_x)
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)

        # +1 when the robot faces +y, -1 when it faces -y: converts world-frame
        # errors into body-frame forward / left-of-track
        sgn = 1.0 if math.sin(heading) > 0 else -1.0
        while not rospy.is_shutdown():
            x, y, yaw = self.pose
            along = (target_y - y) * sgn
            # y only. Adding the lane error here made the test unsatisfiable:
            # the platform cannot strafe, so closing a lane error means running
            # along the row and back, and the test rejects that for being off
            # in y. reachable() is where a wide stop actually gets judged.
            if abs(target_y - y) < tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("drive timed out at y=%.2f (target %.2f, lane "
                              "error %+.3f, yaw %.1f deg)",
                              y, target_y, x - lane_x, math.degrees(yaw))
                self.why_stuck(self.K_ALONG * along)
                self.stop()
                return False
            cross = (x - lane_x) * sgn

            # Steer to a heading that *aims back at the lane*, rather than
            # summing a cross-track term and a heading term. Summing them has a
            # biased equilibrium: the robot can sit at a steady lane offset
            # where -K_CROSS*cross exactly cancels K_YAW*yaw_err, and it then
            # drives the whole row crabbed and off-line. That is how an earlier
            # run ended up 0.92 m off lane and wedged in the vine row.
            aim = math.atan(self.K_LOOK * cross)
            aim = max(-self.AIM_MAX, min(self.AIM_MAX, aim))
            # heading + aim, not heading - sgn*aim. sgn is already inside
            # cross, and applying it twice inverted the correction whenever the
            # platform faced +y -- which is the whole row. The lane error then
            # grew every stop instead of closing: measured -0.021, -0.035,
            # -0.166, -1.049, -2.228 m over consecutive stops, ending with the
            # platform pointing backwards.
            #
            # yaw is CCW from +x. Facing +y and left of the lane (x < lane_x),
            # the platform has to turn clockwise to get back, so the target
            # heading must drop below pi/2. cross < 0 there, so aim < 0, and
            # heading + aim is exactly that. Facing -y the signs flip twice and
            # it still holds.
            yaw_err = wrap((heading + aim) - yaw)

            t = Twist()
            v = self.K_ALONG * along
            # Floor: a pure proportional law hands over a vanishing command as
            # it converges, and a vanishing command loses to the steering term.
            if abs(v) < self.V_MIN:
                v = math.copysign(self.V_MIN, along)
            t.linear.x = max(-self.V_MAX, min(self.V_MAX, v))

            w = self.K_YAW * yaw_err

            # Skid steer subtracts w * track/2 from the inner side. Cap w so
            # that side never reverses while there is still ground to cover:
            # steering that stops a wheel also stops the platform.
            w_cap = min(self.W_MAX, abs(t.linear.x) / self.HALF_TRACK)
            t.angular.z = max(-w_cap, min(w_cap, w))
            # Slow down only when the platform is genuinely crabbing, i.e. its
            # heading is far from the lane direction. Testing yaw_err here
            # instead would count the aim offset the controller is deliberately
            # holding as misalignment: a 0.12 m lane error alone puts aim past
            # the threshold, and the platform then crawls at a third speed for
            # the whole row (measured: 7.6 m in 180 s).
            if abs(wrap(heading - yaw)) > 0.35:
                t.linear.x *= 0.35
            self.pub.publish(t)
            rate.sleep()
        self.stop()
        x, y, yaw = self.pose
        err = x - lane_x
        rospy.loginfo("  arrived (%.3f, %.3f) yaw %.1f deg  "
                      "[lane error %+.3f m]",
                      x, y, math.degrees(yaw), err)
        if abs(err) > self.LANE_WARN:
            rospy.logwarn("  %.2f m off the lane: the arm has about 0.07 m of "
                          "reach margin at this standoff, so some fruit here "
                          "will be out of range", abs(err))
        return True

    def excite(self, pulses=4, v=0.35, dt=0.9):
        """Short forward/back pulses to give a monocular VIO something to work
        with before the run proper.

        VINS-Mono cannot initialise without acceleration excitation: it needs to
        observe gravity and metric scale, and it checks the variance of linear
        acceleration over its window before it will even try. A ground platform
        creeping down a lane at constant velocity produces almost none, which is
        why the estimator sat at "IMU excitation not enouth" through a 7.6 m
        drive. Alternating accelerations, on the other hand, are exactly what it
        wants, and on the clod track they come with pitch and roll for free.

        This is a real-deployment manoeuvre, not a simulation trick: ground
        robots running monocular VIO do this, or get pushed, before they trust
        the estimate.
        """
        rospy.loginfo("VIO excitation: %d pulses", pulses)
        rate = rospy.Rate(50)
        for i in range(pulses):
            for sign in (1.0, -1.0):
                t = Twist()
                t.linear.x = sign * v
                t0 = rospy.Time.now()
                while (rospy.Time.now() - t0).to_sec() < dt:
                    if rospy.is_shutdown():
                        return
                    self.pub.publish(t)
                    rate.sleep()
        self.stop()
        rospy.sleep(0.5)
        rospy.loginfo("VIO excitation done; base at (%.2f, %.2f)",
                      self.pose[0], self.pose[1])

    def face_heading(self, heading, tol=0.03, timeout=40.0):
        rospy.loginfo("turn -> %.1f deg", math.degrees(heading))
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            err = wrap(heading - self.pose[2])
            if abs(err) < tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("turn timed out, error %.3f rad", err)
                self.stop()
                return False
            t = Twist()
            t.angular.z = max(-self.W_MAX, min(self.W_MAX, 1.6 * err))
            self.pub.publish(t)
            rate.sleep()
        self.stop()
        return True


def main():
    rospy.init_node("base_driver")
    k = rospy.get_param("~row", rospy.get_param("~aisle", 0))
    stow_first = rospy.get_param("~stow_first", True)
    # the lane the platform works row k from: LANE_STANDOFF off the row, not
    # the middle of the aisle. Row spacing is uneven (2.2-3.0 m), so an aisle
    # centre is not a fixed distance from the vine and the arm cannot reach
    # the fruit from there.
    lane = lane_x(k)

    if stow_first:
        import moveit_commander
        moveit_commander.roscpp_initialize([])
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.4)
        rospy.loginfo("stowing the arm before driving")
        arm.set_named_target("stow")
        if not arm.go(wait=True):
            rospy.logerr("could not reach the stow pose; not driving")
            return
        arm.stop()

    d = BaseDriver()
    if rospy.get_param("~excite", False):
        d.excite()
    d.face_heading(math.pi / 2)
    d.drive_to_y(ROW_LEN / 2 + 1.5, lane)
    rospy.sleep(1.0)
    # turn around before the return leg, rather than asking the lane keeper to
    # reverse and rotate at the same time
    d.face_heading(-math.pi / 2)
    d.drive_to_y(-ROW_LEN / 2 - 1.5, lane, heading=-math.pi / 2)
    rospy.loginfo("row %d pass complete (lane x=%.2f, standoff %.2f m)",
                  k, lane, LANE_STANDOFF)


if __name__ == "__main__":
    main()
