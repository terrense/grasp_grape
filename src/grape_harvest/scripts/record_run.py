#!/usr/bin/env python3
"""Record /harvest_cam/image_raw straight into an mp4.

Frames are piped into ffmpeg as raw video, so there is no dependency on
cv_bridge or OpenCV and no pile of intermediate PNGs.

    rosrun grape_harvest record_run.py _out:=/mnt/c/Users/.../harvest.mp4
"""
import os
import subprocess
import sys

import rospy
from sensor_msgs.msg import Image

# rgb8 and bgr8 are the two encodings gazebo_ros_camera emits
PIX_FMT = {"rgb8": "rgb24", "bgr8": "bgr24"}


# Re-time the file if the encoded rate is off by more than this.
RETIME_TOL = 0.05


class Recorder(object):
    """Encodes at the rate the camera actually delivers, not the rate it was
    asked for. Gazebo drops sensor frames whenever rendering falls behind, so
    hard-coding 30 fps produces a video that plays fast -- a 115 s run came out
    as 71 s of footage. `probe` seconds of frames are timed first (and kept),
    and that measured rate is what ffmpeg is told.

    The probe alone is not enough. It runs before the harvester has spawned the
    fruit, so it times an empty scene; once ~1300 berry spheres are in the world
    the camera settles at well under half the probed rate, and the file ends up
    playing fast anyway. So close() re-measures over the whole recording and, if
    the encode rate was off by more than RETIME_TOL, re-times the file in a
    second pass. Frames per *simulated* second is the right quantity here: it is
    what makes the video play at the speed the robot actually moved."""

    def __init__(self, out, fps, topic, probe=5.0):
        self.out = out
        self.fps = fps            # 0 => measure it
        self.probe = probe
        self.proc = None
        self.n = 0
        self.warned = False
        self.pending = []         # frames buffered during the probe window
        self.t0 = None
        d = os.path.dirname(out)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        self.sub = rospy.Subscriber(topic, Image, self.cb, queue_size=4)
        rospy.loginfo("recording %s -> %s", topic, out)

    def start(self, msg):
        pf = PIX_FMT.get(msg.encoding)
        if pf is None:
            rospy.logerr("unsupported image encoding %r", msg.encoding)
            rospy.signal_shutdown("bad encoding")
            return False
        # stdin=PIPE only; ffmpeg is put in its own process group so a SIGINT
        # aimed at this node does not kill the encoder before it flushes
        cmd = ["ffmpeg", "-y", "-loglevel", "error",
               "-f", "rawvideo", "-pix_fmt", pf,
               "-s", "%dx%d" % (msg.width, msg.height),
               "-r", str(self.fps), "-i", "-",
               "-an", "-c:v", "libx264", "-preset", "veryfast",
               "-crf", "20", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", self.out]
        rospy.loginfo("ffmpeg: %dx%d %s @ %s fps",
                      msg.width, msg.height, msg.encoding, self.fps)
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                     preexec_fn=os.setpgrp)
        self.expect = msg.width * msg.height * (3 if pf != "gray" else 1)
        return True

    def cb(self, msg):
        if self.proc is None:
            now = rospy.Time.now().to_sec()
            if self.t0 is None:
                self.t0 = now
            self.pending.append(msg.data)
            if self.fps == 0:
                elapsed = now - self.t0
                if elapsed < self.probe or len(self.pending) < 5:
                    return
                self.fps = round(len(self.pending) / elapsed, 2)
                rospy.loginfo("measured camera rate: %.2f fps over %.1f s",
                              self.fps, elapsed)
            if not self.start(msg):
                return
            for data in self.pending:
                self.proc.stdin.write(data)
                self.n += 1
            self.pending = []
            return
        if len(msg.data) != self.expect:
            if not self.warned:
                rospy.logwarn("frame size %d != expected %d; check the step "
                              "field", len(msg.data), self.expect)
                self.warned = True
        try:
            self.proc.stdin.write(msg.data)
            self.n += 1
        except (BrokenPipeError, ValueError):
            rospy.logerr("ffmpeg pipe closed after %d frames", self.n)
            rospy.signal_shutdown("ffmpeg died")

    def close(self):
        if self.proc is None:
            rospy.logwarn("no frames received -- was the camera sensor up?")
            return
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        self.proc.wait()
        sz = os.path.getsize(self.out) if os.path.exists(self.out) else 0
        rospy.loginfo("wrote %d frames (%.1f s at %s fps), %.1f MB -> %s",
                      self.n, self.n / float(self.fps), self.fps,
                      sz / 1e6, self.out)
        self.retime()

    def retime(self):
        """Fix the playback rate if the probe over-estimated it."""
        elapsed = rospy.Time.now().to_sec() - self.t0
        if elapsed <= 0 or self.n < 30:
            return
        true_fps = self.n / elapsed
        err = abs(true_fps - self.fps) / float(self.fps)
        rospy.loginfo("whole-run rate %.2f fps over %.1f s (encoded at %s)",
                      true_fps, elapsed, self.fps)
        if err <= RETIME_TOL:
            return

        tmp = self.out + ".retime.mp4"
        factor = self.fps / true_fps
        rospy.logwarn("encoded rate is %.0f%% off; re-timing by x%.3f so the "
                      "video plays at the speed the robot moved",
                      err * 100, factor)
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-i", self.out,
               "-vf", "setpts=PTS*%.6f" % factor, "-r", "25",
               "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", tmp]
        if subprocess.call(cmd) != 0 or not os.path.exists(tmp):
            rospy.logerr("re-time pass failed; keeping the fast original")
            return
        os.replace(tmp, self.out)
        rospy.loginfo("re-timed: %.1f s of video at %.2f fps -> %s",
                      self.n / true_fps, true_fps, self.out)


def main():
    rospy.init_node("harvest_recorder", anonymous=True)
    out = rospy.get_param("~out", os.path.expanduser("~/harvest.mp4"))
    fps = rospy.get_param("~fps", 0)          # 0 = measure the real rate
    topic = rospy.get_param("~topic", "/harvest_cam/image_raw")
    r = Recorder(out, fps, topic)
    rospy.on_shutdown(r.close)
    rospy.spin()


if __name__ == "__main__":
    main()
