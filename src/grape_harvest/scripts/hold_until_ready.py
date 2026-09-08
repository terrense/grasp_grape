#!/usr/bin/env python3
"""Hold physics until arm_controller is loaded, then release it.

Needed only since the joints became torque driven. Under Joint::SetPosition the
arm was pinned from the first step whether a controller existed or not; under
PID nothing holds it until arm_controller starts, and Gazebo was unpausing at
0.35 s while the controller spawner took another two seconds. A 24 kg CR10
free-falls a long way in two seconds: every joint ended against its stop,
joint4 overshot its 3.1416 limit to 3.5 rad, and MoveIt then refused to plan
because the start state was out of bounds.

Waiting for the controller to be *running* deadlocks: controller_manager
processes a switch in the plugin's update loop, and that loop does not run
while physics is paused, so it can never reach running and this node would
never unpause. Loading, on the other hand, happens in the service callback and
does complete while paused. So this waits for loaded, not running: by then the
spawner has queued the switch, and it is applied on the very first physics
step, which is early enough that the arm never falls.

Wall-clock sleeps throughout, deliberately: sim time does not advance while
paused, so rospy.sleep() would never return.
"""
import time

import rospy
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty

WANT = "arm_controller"
TIMEOUT = 60.0
SETTLE = 1.0          # s after the controller appears, for the switch to queue


def main():
    rospy.init_node("hold_until_ready", disable_signals=True)
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=60.0)
    rospy.wait_for_service("/gazebo/unpause_physics", timeout=60.0)
    listing = rospy.ServiceProxy("/controller_manager/list_controllers",
                                 ListControllers)
    unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

    t0 = time.time()
    while time.time() - t0 < TIMEOUT:
        try:
            names = [c.name for c in listing().controller]
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "list_controllers: %s", e)
            names = []
        if WANT in names:
            rospy.loginfo("%s loaded after %.1f s; releasing physics in %.1f s",
                          WANT, time.time() - t0, SETTLE)
            time.sleep(SETTLE)
            unpause()
            rospy.loginfo("physics running")
            return
        time.sleep(0.2)

    rospy.logerr("%s never loaded in %.0f s; unpausing anyway rather than "
                 "leaving the simulation frozen", WANT, TIMEOUT)
    unpause()


if __name__ == "__main__":
    main()
