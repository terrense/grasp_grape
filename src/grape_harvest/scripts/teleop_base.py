#!/usr/bin/env python3
"""Keyboard throttle for the harvesting platform: forward and back only.

Deliberately no steering. The arm reaches the fruit from a lane 0.80 m off the
row and the whole reach budget is spent on that standoff, so a hand-steered
platform wandering off the lane would simply put the fruit out of range. Driving
is therefore one axis: along the row. Changing rows is not modelled here.

A keyboard has no key-up event, so a held key looks exactly like a tapped one.
The command is latched and expires HOLD seconds after the last press, which is
what makes it feel like a throttle rather than a toggle.

    w / up     forward          s / down   back
    space      stop             + / -      speed
    q          quit
"""
import os
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

HOLD = 0.4                 # s; command expires this long after the last press
RATE = 20.0
V_MIN, V_MAX = 0.05, 0.60
V_STEP = 0.05

HELP = """
  vineyard platform throttle -- forward/back only

    w / arrow-up      drive forward (+y, down the row)
    s / arrow-down    drive back
    space             stop
    + / -             speed  (%.2f - %.2f m/s)
    q                 quit

  The arm works on its own: stop next to a bunch and it will cut it.
"""


def read_key(timeout):
    """One keypress, or None. Arrow keys arrive as a 3-byte escape sequence."""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        rest = ""
        while select.select([sys.stdin], [], [], 0.0005)[0]:
            rest += sys.stdin.read(1)
        return {"[A": "w", "[B": "s"}.get(rest, "")
    return ch


def main():
    rospy.init_node("teleop_base")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    speed = 0.30
    cmd = 0.0
    last = rospy.Time.now()

    sys.stdout.write(HELP % (V_MIN, V_MAX))
    sys.stdout.flush()

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            k = read_key(1.0 / RATE)
            if k in ("w", "W"):
                cmd, last = +1.0, rospy.Time.now()
            elif k in ("s", "S"):
                cmd, last = -1.0, rospy.Time.now()
            elif k == " ":
                cmd = 0.0
            elif k in ("+", "="):
                speed = min(V_MAX, speed + V_STEP)
            elif k in ("-", "_"):
                speed = max(V_MIN, speed - V_STEP)
            elif k in ("q", "Q", "\x03"):
                break

            # latch expiry: this is what makes a held key behave like a throttle
            if cmd != 0.0 and (rospy.Time.now() - last).to_sec() > HOLD:
                cmd = 0.0

            t = Twist()
            t.linear.x = cmd * speed
            pub.publish(t)

            sys.stdout.write("\r  speed %.2f m/s   cmd %+5.2f    " %
                             (speed, t.linear.x))
            sys.stdout.flush()
            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        for _ in range(5):
            pub.publish(Twist())
            rospy.sleep(0.02)
        sys.stdout.write("\n  stopped\n")


if __name__ == "__main__":
    main()
