#!/usr/bin/env python3
"""Find the cluster in front of the laser head, from the eye-in-hand camera.

Why this exists: until now the arm drove open loop to a coordinate that
make_models.py had generated, so the simulation was telling the robot where the
fruit was. A real machine has no such oracle -- it has a camera. This closes
that loop: detect the bunch, work out where its stem is, and correct the aim
before firing.

The detector is deliberately simple. Ripe fruit against a canopy is close to
the easiest colour segmentation problem there is, and a hand-written threshold
keeps the dependency list short and the failure modes obvious. It is not meant
to survive a real orchard; it is meant to make the control loop real. Swapping
it for a learned detector changes this file and nothing else.

Images are decoded with numpy rather than cv_bridge: rgb8 is height x width x 3
bytes and 32FC1 is a float per pixel, so the dependency buys nothing here.

Run standalone to see what it sees:
    rosrun grape_harvest grape_detect.py
"""
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image

# Berries are rendered with diffuse (0.36, 0.10, 0.46): strongly purple, and
# nothing else in the scene is. The test is "red and blue both clearly above
# green" rather than fixed thresholds, so it survives the shading gradient
# across a round bunch and the dimming under the polytunnel film.
GREEN_MARGIN = 18       # how far R and B must exceed G, 0-255
MIN_PIXELS = 120        # smaller blobs are noise or a bunch too far to cut
MAX_RANGE = 0.60        # m; ignore fruit beyond the laser's reach envelope


class GrapeDetector(object):
    def __init__(self, ns="/ee_camera"):
        self.K = None
        self.rgb = None
        self.depth = None
        rospy.Subscriber(ns + "/color/camera_info", CameraInfo, self._info_cb,
                         queue_size=1)
        rospy.Subscriber(ns + "/color/image_raw", Image, self._rgb_cb,
                         queue_size=1)
        rospy.Subscriber(ns + "/depth/image_raw", Image, self._depth_cb,
                         queue_size=1)
        self.frame = ns.strip("/") + "_optical_frame"

    # ------------------------------------------------------------ callbacks
    def _info_cb(self, msg):
        self.K = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])   # fx fy cx cy

    def _rgb_cb(self, msg):
        if msg.encoding != "rgb8":
            rospy.logwarn_throttle(10.0, "unexpected colour encoding %r",
                                   msg.encoding)
            return
        a = np.frombuffer(msg.data, dtype=np.uint8)
        self.rgb = a.reshape(msg.height, msg.width, 3)

    def _depth_cb(self, msg):
        if msg.encoding != "32FC1":
            rospy.logwarn_throttle(10.0, "unexpected depth encoding %r",
                                   msg.encoding)
            return
        a = np.frombuffer(msg.data, dtype=np.float32)
        self.depth = a.reshape(msg.height, msg.width)
        self.stamp = msg.header.stamp
        self.frame = msg.header.frame_id or self.frame

    # ------------------------------------------------------------- detect
    def mask(self):
        """Pixels that look like fruit."""
        if self.rgb is None:
            return None
        r = self.rgb[:, :, 0].astype(np.int16)
        g = self.rgb[:, :, 1].astype(np.int16)
        b = self.rgb[:, :, 2].astype(np.int16)
        return (r - g > GREEN_MARGIN) & (b - g > GREEN_MARGIN)

    def detect(self):
        """Return (stem_xyz, n_pixels, range, height_truncated), or None.

        `height_truncated` means the blob ran into the top of the frame, so the
        bunch is taller than the view and the height should not be trusted.

        The aim point is not the centroid of the berries: the beam has to land
        on the peduncle, which is above the fruit. So the blob's *top* is found
        and the aim point placed a little above it. Getting this wrong puts the
        beam into the middle of the bunch, which would char fruit rather than
        cut a stem.
        """
        m = self.mask()
        if m is None or self.depth is None or self.K is None:
            return None
        if m.sum() < MIN_PIXELS:
            return None

        ys, xs = np.nonzero(m)
        d = self.depth[ys, xs]
        good = np.isfinite(d) & (d > 0.05) & (d < MAX_RANGE)
        if good.sum() < MIN_PIXELS:
            return None
        ys, xs, d = ys[good], xs[good], d[good]

        # median range over the blob: robust to the odd pixel that reads
        # through a gap between berries onto the canopy behind
        z = float(np.median(d))

        # top of the bunch, averaged across a band so one stray pixel cannot
        # define it
        top = int(np.percentile(ys, 3))
        band = ys <= max(top + 3, np.percentile(ys, 8))
        u = float(np.mean(xs[band]))
        v = float(np.mean(ys[band]))

        # If the blob reaches the top edge, the bunch is taller than the view
        # and this "top" is the image border, not the fruit. The lateral fix is
        # still sound; the height is not, and saying so is better than
        # inventing one.
        truncated = bool(ys.min() <= 2)

        fx, fy, cx, cy = self.K
        # a little above the top of the fruit is the stem
        v_stem = v - 0.010 * fy / max(z, 1e-3)      # ~10 mm up, in pixels

        x = (u - cx) * z / fx
        y = (v_stem - cy) * z / fy
        return (x, y, z), int(good.sum()), z, truncated

    def point_msg(self):
        got = self.detect()
        if got is None:
            return None
        (x, y, z), n, rng, _ = got
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = self.frame
        p.point.x, p.point.y, p.point.z = x, y, z
        return p


def main():
    rospy.init_node("grape_detect")
    det = GrapeDetector()
    pub = rospy.Publisher("~stem", PointStamped, queue_size=1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        got = det.detect()
        if got is None:
            rospy.loginfo_throttle(3.0, "no cluster in view")
        else:
            (x, y, z), n, rng, trunc = got
            rospy.loginfo_throttle(
                1.0, "stem at (%+.3f, %+.3f, %.3f) in %s, %d px, range %.3f m%s",
                x, y, z, det.frame, n, rng,
                "  [blob hits the frame edge, height unreliable]"
                if trunc else "")
            pub.publish(det.point_msg())
        rate.sleep()


if __name__ == "__main__":
    main()
