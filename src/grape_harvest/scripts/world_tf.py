#!/usr/bin/env python3
"""Publish the world -> odom correction from Gazebo ground truth.

Why this exists: the skid-steer plugin broadcasts odom -> base_footprint from
wheel encoders, which slip badly on the clod track. MoveIt plans the arm in the
`world` frame against clusters whose positions are fixed in the world, so it
needs an accurate world -> base_footprint. This node closes that loop the same
way AMCL publishes map -> odom: it does not touch odom -> base_footprint, it
just corrects the frame above it.

This is a placeholder for VINS-Mono. Swap this node for the VIO pose output and
nothing downstream changes -- that is the whole point of correcting above odom
rather than faking odom itself.
"""
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def mat_from_pose(p, q):
    m = tft.quaternion_matrix(q)
    m[0:3, 3] = p
    return m


def mat_from_tf(t):
    return mat_from_pose(
        [t.transform.translation.x, t.transform.translation.y,
         t.transform.translation.z],
        [t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w])


class WorldTf(object):
    def __init__(self):
        self.world = rospy.get_param("~world_frame", "world")
        self.odom = rospy.get_param("~odom_frame", "odom")
        self.base = rospy.get_param("~base_frame", "base_footprint")
        self.gt_body = rospy.get_param("~gt_body_frame", "base_link")

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        self.br = tf2_ros.TransformBroadcaster()
        self.warned = False
        self.n = 0
        rospy.Subscriber("/ground_truth/state", Odometry, self.cb,
                         queue_size=1)
        rospy.loginfo("world_tf: %s -> %s, ground truth on %s",
                      self.world, self.odom, self.gt_body)

    def cb(self, msg):
        # ground truth pose of gt_body (base_link) in the world
        T_w_gt = mat_from_pose(
            [msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z],
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        try:
            # gt_body -> base: a fixed offset, but read it from TF so a change
            # to the URDF cannot silently desynchronise this node
            T_gt_base = mat_from_tf(self.buf.lookup_transform(
                self.gt_body, self.base, rospy.Time(0), rospy.Duration(0.2)))
            T_odom_base = mat_from_tf(self.buf.lookup_transform(
                self.odom, self.base, rospy.Time(0), rospy.Duration(0.2)))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            if not self.warned:
                rospy.logwarn("world_tf waiting for TF: %s", e)
                self.warned = True
            return
        self.warned = False

        T_w_base = T_w_gt.dot(T_gt_base)
        T_w_odom = T_w_base.dot(np.linalg.inv(T_odom_base))

        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) \
            else rospy.Time.now()
        t.header.frame_id = self.world
        t.child_frame_id = self.odom
        t.transform.translation.x = T_w_odom[0, 3]
        t.transform.translation.y = T_w_odom[1, 3]
        t.transform.translation.z = T_w_odom[2, 3]
        q = tft.quaternion_from_matrix(T_w_odom)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)
        self.n += 1
        if self.n == 1:
            rospy.loginfo("world_tf: publishing %s -> %s", self.world, self.odom)


if __name__ == "__main__":
    rospy.init_node("world_tf")
    WorldTf()
    rospy.spin()
