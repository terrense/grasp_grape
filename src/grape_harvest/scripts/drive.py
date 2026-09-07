#!/usr/bin/env python3
"""Base motion for the vineyard platform.

The aisles are straight and axis aligned, so this is a lane keeper rather than
a planner: drive along +/-y at a fixed x, holding heading. Feedback comes from
/ground_truth/state for the same reason world_tf.py exists -- wheel odometry
slips too much on the clod track to place the arm afterwards.

Run standalone for the clearance test:
    rosrun grape_harvest drive.py _row:=0 _stow_first:=true
"""
import math
import os
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_models import lane_x, ROW_LEN, row_x, N_ROWS, LANE_STANDOFF


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BaseDriver(object):
    V_MAX = 0.45          # m/s; a harvesting platform crawls
    W_MAX = 0.7
    K_ALONG = 0.9
    K_CROSS = 1.4
    K_YAW = 2.0

    def __init__(self, cmd_topic="/cmd_vel"):
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.pose = None      # (x, y, yaw)
        rospy.Subscriber("/ground_truth/state", Odometry, self._cb,
                         queue_size=1)
        rospy.loginfo("waiting for ground truth ...")
        while self.pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("base at (%.2f, %.2f) yaw %.1f deg",
                      self.pose[0], self.pose[1], math.degrees(self.pose[2]))

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

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
            if abs(target_y - y) < tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("drive timed out at y=%.2f", y)
                self.stop()
                return False
            cross = (x - lane_x) * sgn
            yaw_err = wrap(heading - yaw)

            t = Twist()
            t.linear.x = max(-self.V_MAX,
                             min(self.V_MAX, self.K_ALONG * along))
            # crab back onto the lane centre, then straighten up
            t.angular.z = max(-self.W_MAX,
                              min(self.W_MAX,
                                  -self.K_CROSS * cross + self.K_YAW * yaw_err))
            self.pub.publish(t)
            rate.sleep()
        self.stop()
        x, y, yaw = self.pose
        rospy.loginfo("  arrived (%.3f, %.3f) yaw %.1f deg  "
                      "[lane error %+.3f m]",
                      x, y, math.degrees(yaw), x - lane_x)
        return True

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
