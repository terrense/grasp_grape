#!/usr/bin/env python3
"""Contact sensing for the arm: a reactive stop, and a compliant response.

Two things the arm could not do before, both needing the same input -- what
force is on the tool.

    reactive stop (item 4)   the planner only avoids obstacles it was told
                             about. A canopy is not one of them. If the tool
                             runs into something, stop the trajectory instead of
                             pushing a position command through it.

    admittance (item 3)      when contact is expected -- the collar closing
                             around a bunch -- pushing to a position is the
                             wrong goal. Shape how the tool behaves against the
                             contact instead: yield in proportion to the force.

Gravity compensation is the part that makes it work, and the first two versions
of this file did not have it. The sensor carries the head and collar, 1.77 kg,
so it reads 17.4 N standing still; and that vector rotates in the sensor frame
as the wrist moves, which a fixed bias cannot follow. Measured over a move with
nothing in the way:

    free-space wrench       median   p99    max
    raw                      17.4    18.0   19.6
    minus a static bias      11.2    24.8   25.4     <- what it used to use
    minus a gravity model     0.1     0.9    4.5

The threshold was 25 N. Free motion peaked at 25.4 N. So every trip the guard
ever reported was the arm's own weight swinging round, which is what the runs
showed: contacts announced in clear air, at 108 N and 1160 N and other numbers
no 1.8 kg head could produce.

Subtracting the modelled weight instead leaves 4.5 N of noise, which leaves
room for a threshold that means something. What remains after that is
tare()-able: an attached cluster adds its own steady pull, and taring the
compensated residual after a grasp takes it out.

The torque model comes from the same measurement. At rest tau0 = r x g0 with g0
known, which fixes r up to its component along g0; the minimum-norm solution
reproduces tau0 and tracks well enough as the wrist turns.
"""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import WrenchStamped

# The plugin publishes the wrench in the frame of the joint's child link.
SENSOR_FRAME = "Link6"
WORLD_FRAME = "world"


def _quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def _mul(R, v):
    return tuple(sum(R[i][j] * v[j] for j in range(3)) for i in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


class ContactGuard(object):
    # Free motion peaks at 4.5 N once gravity is modelled out, so 12 N is a
    # contact the arm is leaning on with better than a 2x margin. The old 25 N
    # was not chosen from data and sat below the free-motion peak.
    TRIP_FORCE = 12.0
    TRIP_TORQUE = 3.0

    # A contact has to last this long to count. Starting and stopping a
    # trajectory puts a transient through the sensor, and a threshold with no
    # time term reads that as a collision. Something the tool is actually
    # leaning on does not go away.
    TRIP_HOLD = 0.20

    # Admittance: metres of yield per newton, capped. Deliberately soft and
    # small -- this is meant to take the edge off a contact, not to turn the
    # arm into a spring that oscillates against the vine.
    COMPLIANCE = 0.0016
    MAX_YIELD = 0.05

    def __init__(self, topic="/ee_ft"):
        self.w = None
        self.mg = 17.40           # weight of the head, refined by calibrate()
        self.r = (0.0, 0.0, 0.0)  # its moment arm in the sensor frame
        self.bias = (0.0,) * 6    # residual, e.g. a cluster in the collar
        self.over_since = None
        self._buf = tf2_ros.Buffer()
        self._lis = tf2_ros.TransformListener(self._buf)
        rospy.Subscriber(topic, WrenchStamped, self._cb, queue_size=1)

    def _cb(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        self.w = (f.x, f.y, f.z, t.x, t.y, t.z)

    def ready(self, timeout=5.0):
        t0 = rospy.Time.now()
        while self.w is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rospy.sleep(0.05)
        return self.calibrate()

    # ---------------------------------------------------------------- model
    def _R_world_to_sensor(self):
        t = self._buf.lookup_transform(SENSOR_FRAME, WORLD_FRAME,
                                       rospy.Time(0), rospy.Duration(0.5))
        return _quat_to_R(t.transform.rotation)

    def calibrate(self):
        """Learn the head's weight and moment arm from one static reading.

        Must be called with the arm still and nothing in the collar. The force
        gives mg directly. The torque gives the moment arm only up to its
        component along gravity, which no single pose can observe; the
        minimum-norm r reproduces the reading and tracks well enough as the
        wrist turns.
        """
        if self.w is None:
            return False
        try:
            R = self._R_world_to_sensor()
        except Exception as e:
            rospy.logwarn("contact guard: no TF for gravity compensation "
                          "(%s); falling back to a static bias, which is not "
                          "good enough to trust", e)
            self.bias = self.w
            self.mg = None
            return True
        f0 = self.w[:3]
        t0 = self.w[3:]
        self.mg = _norm(f0)
        g = _mul(R, (0.0, 0.0, -self.mg))
        gg = sum(x * x for x in g)
        self.r = tuple(x / gg for x in _cross(g, t0)) if gg > 1e-9 else (0.0,) * 3
        self.bias = (0.0,) * 6
        rospy.loginfo("contact guard: head weighs %.2f N (%.2f kg), moment arm "
                      "%.0f mm", self.mg, self.mg / 9.81, _norm(self.r) * 1000)
        return True

    def _expected(self):
        """The wrench the head's own weight should produce right now."""
        if self.mg is None:
            return (0.0,) * 6
        try:
            R = self._R_world_to_sensor()
        except Exception:
            return (0.0,) * 6
        g = _mul(R, (0.0, 0.0, -self.mg))
        return tuple(g) + _cross(self.r, g)

    # ---------------------------------------------------------------- signal
    def residual(self):
        """The six-vector left after the head's own weight and any tare."""
        if self.w is None:
            return (0.0,) * 6
        e = self._expected()
        return tuple(self.w[i] - e[i] - self.bias[i] for i in range(6))

    def tare(self):
        """Zero the residual: takes out a cluster's weight after a grasp.

        With gravity modelled this is a small correction rather than the whole
        signal, so it no longer has to be called before every move.
        """
        if self.w is None:
            return
        e = self._expected()
        self.bias = tuple(self.w[i] - e[i] for i in range(6))
        self.over_since = None

    def reset(self):
        """Forget any contact in progress."""
        self.over_since = None

    def contact(self):
        """(force_N, torque_Nm) that is not the head's own weight."""
        d = self.residual()
        return _norm(d[:3]), _norm(d[3:])

    def tripped(self):
        """True once the threshold has been exceeded continuously for
        TRIP_HOLD. A momentary spike is not a collision."""
        f, t = self.contact()
        if not (f > self.TRIP_FORCE or t > self.TRIP_TORQUE):
            self.over_since = None
            return False
        now = rospy.Time.now()
        if self.over_since is None:
            self.over_since = now
            return False
        return (now - self.over_since).to_sec() >= self.TRIP_HOLD

    def yield_offset(self):
        """How far to give way, in metres, along the contact direction.

        Returns (dx, dy, dz) in the sensor frame. The caller decides what to do
        with it; this class does not command motion.
        """
        d = self.residual()[:3]
        mag = _norm(d)
        if mag < 1e-6:
            return (0.0, 0.0, 0.0)
        y = min(self.MAX_YIELD, self.COMPLIANCE * mag)
        return tuple(y * x / mag for x in d)


def main():
    """Watch the wrist. Useful for seeing what a contact actually looks like."""
    rospy.init_node("contact_guard")
    g = ContactGuard()
    if not g.ready(10.0):
        rospy.logerr("no wrench on /ee_ft")
        return
    rospy.loginfo("watching; trip at %.0f N / %.0f Nm held for %.2f s",
                  g.TRIP_FORCE, g.TRIP_TORQUE, g.TRIP_HOLD)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        f, t = g.contact()
        if g.tripped():
            rospy.logwarn("CONTACT  %.1f N  %.1f Nm", f, t)
        else:
            rospy.loginfo_throttle(2.0, "  %.1f N  %.1f Nm", f, t)
        rate.sleep()


if __name__ == "__main__":
    main()
