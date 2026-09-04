#!/usr/bin/env python3
"""Generate the SDF models and the world for the vineyard block.

Frame: Gazebo world origin sits in the first aisle, +x across the rows,
+y along the rows. The robot is a free body now, so the world origin is NOT
the robot base -- see pick_grape.py, which resolves everything through TF.

Layout constants live here and are imported by pick_grape.py, so the scene and
the motion plan cannot drift apart. bunch_layout() is seeded, so the same
vineyard comes back every run.
"""
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
MODELS = os.path.join(PKG, "models")
WORLDS = os.path.join(PKG, "worlds")

# ------------------------------------------------------------------ layout
PANEL_W = 2.00          # trellis panel width = post spacing along a row
PANELS_PER_ROW = 4
N_ROWS = 4
# Aisle pitch. The brief asks for at least 0.75 x panel width so the robot can
# drive through with the arm stowed; 0.80 x leaves a little more margin.
ROW_DX = 0.80 * PANEL_W                     # 1.60 m
ROW_X0 = 0.58                               # first row
ROW_LEN = PANELS_PER_ROW * PANEL_W          # 8.00 m
POST_H = 1.85
WIRE_Z = 1.22                               # fruiting wire
CANOPY_Z = 1.62
LAYOUT_SEED = 11

# Harvest crate rides on the platform. Expressed in the robot base_link frame:
# floor centre on the rear deck, plus the inner size and the drop slots.
CRATE_BASE_XYZ = (-0.52, 0.0, 0.47)
CRATE_INNER = (0.44, 0.44, 0.20)
CRATE_SLOTS = [(dx, dy) for dx in (-0.11, 0.11)
               for dy in (-0.14, 0.0, 0.14)]

# Fixed observation camera. Gazebo camera links look down their +X axis, the
# same convention the GUI <camera> pose uses, so both share this pose.
CAM_POSE = (-3.20, -5.00, 2.60, 0.0, 0.331, 0.785)


def row_x(i):
    """World x of trellis row i."""
    return ROW_X0 + i * ROW_DX


def aisle_x(k):
    """World x of aisle k. There are N_ROWS+1 aisles; aisle 0 is in front of
    row 0, aisle k>0 runs between row k-1 and row k."""
    return ROW_X0 - ROW_DX / 2.0 + k * ROW_DX


def panel_y(p):
    """World y of the centre of panel p within a row."""
    return -ROW_LEN / 2.0 + (p + 0.5) * PANEL_W


def bunch_layout():
    """Every cluster in the block, as a list of spec dicts.

    Cluster count per panel, hanging depth, cluster size and berry count are
    all randomised -- a real vineyard is not a grid of identical fruit. The
    *hanging depth* is randomised through the peduncle length rather than by
    moving the attachment point, so every cluster still hangs off the fruiting
    wire the way a real shoot does, while the grasp height genuinely varies.
    """
    rnd = random.Random(LAYOUT_SEED)
    out = []
    for r in range(N_ROWS):
        x = row_x(r)
        for p in range(PANELS_PER_ROW):
            cy = panel_y(p)
            n = rnd.choice([1, 2, 2, 3])
            span = PANEL_W - 0.6
            for k in range(n):
                off = 0.0 if n == 1 else (k / float(n - 1) - 0.5) * span
                pedu = round(rnd.uniform(0.12, 0.30), 3)
                body = round(rnd.uniform(0.20, 0.32), 3)
                out.append(dict(
                    name="grape_r%dp%d_%d" % (r, p, k),
                    row=r, panel=p,
                    x=x, y=round(cy + off + rnd.uniform(-0.07, 0.07), 3),
                    pedu_len=pedu,
                    body_len=body,
                    r_top=round(rnd.uniform(0.058, 0.082), 3),
                    n_berries=rnd.randint(32, 58),
                    mass=round(rnd.uniform(0.18, 0.32), 3),
                    seed=rnd.randint(1, 100000),
                    # derived, so callers never recompute them inconsistently
                    grasp_z=round(WIRE_Z - pedu / 2.0, 4),
                    hang_below_tcp=round(pedu / 2.0 + body, 4)))
    return out


# ---------------------------------------------------------------- grape bunch
def grape_bunch(name="grape_bunch", seed=7, n_berries=46,
                pedu_len=0.15, body_len=0.26,
                r_top=0.072, r_bot=0.018, berry_r=0.0145, mass=0.25,
                **_ignored):
    """Model origin sits at the wire; the peduncle hangs down from z=0."""
    rnd = random.Random(seed)
    berries = []
    for i in range(n_berries):
        t = (i + 0.5) / n_berries              # 0 at top of body, 1 at the tip
        z = -pedu_len - t * body_len
        rad = (r_top + (r_bot - r_top) * t) * math.sqrt(rnd.random())
        ang = rnd.uniform(0, 2 * math.pi)
        berries.append((rad * math.cos(ang), rad * math.sin(ang),
                        z + rnd.uniform(-0.012, 0.012)))

    vis = []
    for i, (x, y, z) in enumerate(berries):
        vis.append(
            "\n      <visual name=\"berry_%d\">"
            "\n        <pose>%.4f %.4f %.4f 0 0 0</pose>"
            "\n        <geometry><sphere><radius>%.4f</radius></sphere></geometry>"
            "\n        <material>"
            "\n          <ambient>0.20 0.05 0.28 1</ambient>"
            "\n          <diffuse>0.36 0.10 0.46 1</diffuse>"
            "\n          <specular>0.35 0.30 0.40 32</specular>"
            "\n        </material>"
            "\n      </visual>" % (i, x, y, z, berry_r))
    vis_xml = "".join(vis)

    cz = -pedu_len - body_len / 2.0
    pz = -pedu_len / 2.0

    # One cylinder hull instead of ~46 sphere collisions: far cheaper, and it
    # keeps the ODE contact solver stable when the cluster swings.
    body_col = (
        "\n      <collision name=\"body_col\">"
        "\n        <pose>0 0 %.4f 0 0 0</pose>"
        "\n        <geometry><cylinder><radius>%.4f</radius>"
        "<length>%.4f</length></cylinder></geometry>"
        "\n        <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>"
        "\n      </collision>" % (cz, r_top * 0.85, body_len))

    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="%s">
    <link name="bunch_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 %.4f 0 0 0</pose>
        <mass>%.3f</mass>
        <inertia><ixx>0.004</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.004</iyy><iyz>0</iyz><izz>0.002</izz></inertia>
      </inertial>

      <visual name="peduncle_vis">
        <pose>0 0 %.4f 0 0 0</pose>
        <geometry><cylinder><radius>0.006</radius><length>%.3f</length></cylinder></geometry>
        <material>
          <ambient>0.16 0.20 0.06 1</ambient>
          <diffuse>0.32 0.40 0.12 1</diffuse>
        </material>
      </visual>
      <collision name="peduncle_col">
        <pose>0 0 %.4f 0 0 0</pose>
        <geometry><cylinder><radius>0.006</radius><length>%.3f</length></cylinder></geometry>
        <surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface>
      </collision>%s%s
    </link>
  </model>
</sdf>
""" % (name, cz, mass, pz, pedu_len, pz, pedu_len, vis_xml, body_col)


# -------------------------------------------------------------------- trellis
def trellis():
    """The whole block: N_ROWS rows of PANELS_PER_ROW panels.

    Anchored to the world by fixed joints rather than <static> so
    gazebo_ros_link_attacher can joint clusters onto the fruiting wires.
    Canopy leaves are visual-only (no collision) -- that mirrors the leaf
    removal a real vineyard does before harvest, which opens up the fruiting
    zone, and it keeps the arm from being blocked by foliage it would in
    practice just push aside.
    """
    rnd = random.Random(3)
    parts = []
    for r in range(N_ROWS):
        x = row_x(r)
        # PANELS_PER_ROW panels need PANELS_PER_ROW+1 posts
        for i in range(PANELS_PER_ROW + 1):
            y = -ROW_LEN / 2.0 + i * PANEL_W
            tag = "r%d_%d" % (r, i)
            parts.append(
                "\n    <link name=\"post_%s\">"
                "\n      <pose>%.3f %.3f %.3f 0 0 0</pose>"
                "\n      <inertial><mass>20</mass>"
                "\n        <inertia><ixx>5</ixx><ixy>0</ixy><ixz>0</ixz>"
                "<iyy>5</iyy><iyz>0</iyz><izz>0.2</izz></inertia>"
                "\n      </inertial>"
                "\n      <visual name=\"v\">"
                "\n        <geometry><cylinder><radius>0.045</radius>"
                "<length>%.3f</length></cylinder></geometry>"
                "\n        <material><ambient>0.24 0.17 0.10 1</ambient>"
                "<diffuse>0.42 0.30 0.18 1</diffuse></material>"
                "\n      </visual>"
                "\n      <collision name=\"c\">"
                "\n        <geometry><cylinder><radius>0.045</radius>"
                "<length>%.3f</length></cylinder></geometry>"
                "\n      </collision>"
                "\n    </link>"
                "\n    <joint name=\"fix_post_%s\" type=\"fixed\">"
                "\n      <parent>world</parent><child>post_%s</child>"
                "\n    </joint>"
                % (tag, x, y, POST_H / 2, POST_H, POST_H, tag, tag))

        # one continuous fruiting wire and one canopy wire per row
        for wz, wtag in ((WIRE_Z, "fruit"), (CANOPY_Z, "canopy")):
            tag = "r%d_%s" % (r, wtag)
            parts.append(
                "\n    <link name=\"wire_%s\">"
                "\n      <pose>%.3f 0 %.3f 1.5708 0 0</pose>"
                "\n      <inertial><mass>1.0</mass>"
                "\n        <inertia><ixx>0.2</ixx><ixy>0</ixy><ixz>0</ixz>"
                "<iyy>0.2</iyy><iyz>0</iyz><izz>0.001</izz></inertia>"
                "\n      </inertial>"
                "\n      <visual name=\"v\">"
                "\n        <geometry><cylinder><radius>0.004</radius>"
                "<length>%.2f</length></cylinder></geometry>"
                "\n        <material><ambient>0.30 0.30 0.32 1</ambient>"
                "<diffuse>0.55 0.56 0.58 1</diffuse></material>"
                "\n      </visual>"
                "\n      <collision name=\"c\">"
                "\n        <geometry><cylinder><radius>0.004</radius>"
                "<length>%.2f</length></cylinder></geometry>"
                "\n      </collision>"
                "\n    </link>"
                "\n    <joint name=\"fix_wire_%s\" type=\"fixed\">"
                "\n      <parent>world</parent><child>wire_%s</child>"
                "\n    </joint>"
                % (tag, x, wz, ROW_LEN, ROW_LEN, tag, tag))

    leaves = []
    for r in range(N_ROWS):
        x = row_x(r)
        n = int(ROW_LEN / 0.16)
        for i in range(n):
            y = -ROW_LEN / 2.0 + (i + 0.5) * (ROW_LEN / n)
            leaves.append(
                "\n      <visual name=\"leaf_r%d_%d\">"
                "\n        <pose>%.3f %.3f %.3f 0 %.2f %.2f</pose>"
                "\n        <geometry><box><size>0.02 0.20 0.22</size></box></geometry>"
                "\n        <material><ambient>0.08 0.20 0.05 1</ambient>"
                "<diffuse>0.16 0.42 0.10 1</diffuse></material>"
                "\n      </visual>"
                % (r, i, x + rnd.uniform(-0.05, 0.05), y,
                   rnd.uniform(CANOPY_Z - 0.20, CANOPY_Z + 0.12),
                   rnd.uniform(-0.4, 0.4), rnd.uniform(-0.5, 0.5)))

    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="trellis">%s
    <link name="canopy">
      <pose>0 0 0 0 0 0</pose>
      <inertial><mass>2.0</mass>
        <inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia>
      </inertial>%s
    </link>
    <joint name="fix_canopy" type="fixed">
      <parent>world</parent><child>canopy</child>
    </joint>
  </model>
</sdf>
""" % ("".join(parts), "".join(leaves))


# ------------------------------------------------------------------- terrain
def terrain(seed=23):
    """Clods and ruts scattered down the aisles.

    Flattened boxes at random yaw rather than spheres: a half-buried sphere
    gives a point contact that launches a rigid four-wheeler, whereas low
    slabs behave like the dried clods and wheel ruts the tyres are chosen for.
    """
    rnd = random.Random(seed)
    parts = []
    i = 0
    for k in range(N_ROWS + 1):
        ax = aisle_x(k)
        for _ in range(26):
            sx = rnd.uniform(0.12, 0.34)
            sy = rnd.uniform(0.12, 0.34)
            sz = rnd.uniform(0.02, 0.07)
            x = ax + rnd.uniform(-0.55, 0.55)
            y = rnd.uniform(-ROW_LEN / 2 - 1.5, ROW_LEN / 2 + 1.5)
            shade = rnd.uniform(0.30, 0.46)
            parts.append(
                "\n      <visual name=\"clod_%d\"><pose>%.3f %.3f %.4f 0 0 %.3f</pose>"
                "\n        <geometry><box><size>%.3f %.3f %.3f</size></box></geometry>"
                "\n        <material><ambient>%.2f %.2f %.2f 1</ambient>"
                "<diffuse>%.2f %.2f %.2f 1</diffuse></material>"
                "\n      </visual>"
                "\n      <collision name=\"clod_%d_c\"><pose>%.3f %.3f %.4f 0 0 %.3f</pose>"
                "\n        <geometry><box><size>%.3f %.3f %.3f</size></box></geometry>"
                "\n        <surface><friction><ode><mu>1.1</mu><mu2>1.1</mu2></ode></friction></surface>"
                "\n      </collision>"
                % (i, x, y, sz / 2, rnd.uniform(0, 3.14), sx, sy, sz,
                   shade * 0.7, shade * 0.55, shade * 0.4,
                   shade, shade * 0.8, shade * 0.58,
                   i, x, y, sz / 2, rnd.uniform(0, 3.14), sx, sy, sz))
            i += 1

    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="dirt_track">
    <static>true</static>
    <link name="link">%s
    </link>
  </model>
</sdf>
""" % "".join(parts)


# --------------------------------------------------------------------- world
def world():
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="vineyard">

    <include><uri>model://sun</uri></include>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <surface><friction><ode><mu>1.1</mu><mu2>1.1</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <material>
            <ambient>0.22 0.18 0.13 1</ambient>
            <diffuse>0.42 0.34 0.24 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <include><uri>model://trellis</uri><pose>0 0 0 0 0 0</pose></include>
    <include><uri>model://dirt_track</uri><pose>0 0 0 0 0 0</pose></include>

    <!-- Creates/removes fixed joints between links at runtime. Per cluster the
         demo uses it three times: hang it on the wire, clamp it in the cutter,
         and release it over the crate. -->
    <plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/>

    <!-- Fixed observation camera, published on /harvest_cam/image_raw. Used to
         record the run: a clean render at a steady frame rate, independent of
         whatever the GUI camera happens to be doing. -->
    <model name="harvest_cam">
      <static>true</static>
      <pose>%.3f %.3f %.3f %.3f %.3f %.3f</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <camera>
            <horizontal_fov>1.15</horizontal_fov>
            <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>120</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>false</visualize>
          <plugin name="harvest_cam_plugin" filename="libgazebo_ros_camera.so">
            <cameraName>harvest_cam</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>harvest_cam_link</frameName>
            <updateRate>30.0</updateRate>
          </plugin>
        </sensor>
      </link>
    </model>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver><type>quick</type><iters>60</iters><sor>1.3</sor></solver>
        <constraints>
          <cfm>0.0</cfm><erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.74 0.88 1</background>
      <shadows>true</shadows>
    </scene>

    <gui>
      <camera name="user_camera">
        <pose>%.3f %.3f %.3f %.3f %.3f %.3f</pose>
      </camera>
    </gui>

  </world>
</sdf>
""" % (CAM_POSE + CAM_POSE)


def write(name, sdf, desc):
    d = os.path.join(MODELS, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(d, "model.config"), "w") as f:
        f.write("""<?xml version="1.0"?>
<model>
  <name>%s</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>%s</description>
</model>
""" % (name, desc))
    print("  %s/model.sdf  (%d bytes)" % (name, len(sdf)))


if __name__ == "__main__":
    print("generating models into %s" % MODELS)
    write("trellis", trellis(),
          "%dx%d VSP grape trellis block, world-anchored"
          % (N_ROWS, PANELS_PER_ROW))
    write("dirt_track", terrain(), "Clods and ruts down the aisles")

    os.makedirs(WORLDS, exist_ok=True)
    with open(os.path.join(WORLDS, "vineyard.world"), "w") as f:
        f.write(world())
    print("  worlds/vineyard.world")

    # Clusters are spawned at runtime with per-cluster SDF (see pick_grape.py),
    # so there is no grape_bunch model directory any more.
    old = os.path.join(MODELS, "grape_bunch")
    if os.path.isdir(old):
        for fn in os.listdir(old):
            os.remove(os.path.join(old, fn))
        os.rmdir(old)
        print("  removed stale models/grape_bunch (now generated per cluster)")
    old = os.path.join(MODELS, "harvest_crate")
    if os.path.isdir(old):
        for fn in os.listdir(old):
            os.remove(os.path.join(old, fn))
        os.rmdir(old)
        print("  removed stale models/harvest_crate (crate now rides on the robot)")

    b = bunch_layout()
    print("\nblock: %d rows x %d panels, row pitch %.2f m (%.2f x panel width)"
          % (N_ROWS, PANELS_PER_ROW, ROW_DX, ROW_DX / PANEL_W))
    print("rows at x = %s" % ", ".join("%.2f" % row_x(i) for i in range(N_ROWS)))
    print("aisles at x = %s" % ", ".join("%.2f" % aisle_x(k)
                                         for k in range(N_ROWS + 1)))
    print("%d clusters total" % len(b))
    for r in range(N_ROWS):
        per = [len([x for x in b if x["row"] == r and x["panel"] == p])
               for p in range(PANELS_PER_ROW)]
        print("  row %d: %s clusters per panel" % (r, per))
    zs = [x["grasp_z"] for x in b]
    print("grasp height %.3f .. %.3f m (peduncle length varies)"
          % (min(zs), max(zs)))
