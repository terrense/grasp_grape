#!/usr/bin/env python3
"""Generate the SDF models and the world for the greenhouse vineyard block.

Frame: Gazebo world origin is west of the block, +x across the rows, +y along
them. The robot is a free body, so the world origin is NOT the robot base --
see pick_grape.py, which resolves everything through TF.

Layout constants live here and are imported by pick_grape.py and drive.py, so
the scene, the motion plan and the base controller cannot drift apart.
Everything random is seeded, so the same vineyard comes back every run.

Three things make this block different from a textbook VSP vineyard:

  * Row spacing is *not* uniform. Real blocks are planted to whatever the land
    and the tractor allow, so each gap is drawn independently from
    ROW_GAP_RANGE. row_x() therefore accumulates gaps instead of multiplying a
    pitch, and there is no single "row pitch" constant to rely on.
  * Fruit hangs high, 1.40-1.70 m. The cordon height varies per row and the
    peduncle length varies per cluster; between them the grasp points spread
    across that band.
  * The whole block sits under one big single-span polytunnel. Its legs are
    only along the two outer edges, so no upright ever lands in a working
    aisle -- that is what keeps the platform's lane clear.
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
ROW_LEN = PANELS_PER_ROW * PANEL_W          # 8.00 m

# Row spacing varies row to row; these are the gaps *between* adjacent rows,
# so there are N_ROWS-1 of them.
ROW_GAP_RANGE = (2.20, 3.00)
ROW_X0 = 2.00           # first row; leaves room for the outer working lane
LAYOUT_SEED = 11

# Cordon (fruiting wire) height per row, and the peduncle length per cluster.
# Together these put the grasp points in the 1.40-1.70 m band the block is
# planted to. main() asserts the realised range.
CORDON_RANGE = (1.56, 1.78)
PEDU_RANGE = (0.14, 0.30)
CANOPY_OVER_CORDON = 0.40
POST_H = 2.30

# Working lane: how far the platform's centreline stands off the row it is
# picking. Not the aisle centre -- with 2.2-3.0 m spacing the centre is
# 1.10-1.50 m out, and even a CR10 cannot reach 1.70 m high fruit from there.
# At 0.80 m the straight-line distance to the fruit is 1.04-1.25 m, inside the
# CR10 envelope with margin, and the 0.56 m wide chassis still leaves >1.1 m
# on the far side of the narrowest aisle.
LANE_STANDOFF = 0.80

# ------------------------------------------------------------- start pose
# Every run begins here, and the spot is painted on the ground. VINS-Mono
# initialises its own frame wherever it is switched on, so a fixed, marked
# start is what makes two runs comparable -- and makes a drifted estimate
# obvious by eye. launch/vineyard_gazebo.launch must agree with these.
START_X = None          # filled in below, once lane_x() exists
START_Y = -5.20
START_YAW = 1.5708      # facing +y, down the lane
PAD_SIZE = 1.60

# Harvest crate rides on the platform. Expressed in the robot base_link frame:
# floor centre on the rear deck, plus the inner size and the drop slots.
CRATE_BASE_XYZ = (-0.52, 0.0, 0.47)
CRATE_INNER = (0.44, 0.44, 0.20)
CRATE_SLOTS = [(dx, dy) for dx in (-0.11, 0.11)
               for dy in (-0.14, 0.0, 0.14)]

# ------------------------------------------------------------- polytunnel
# One single-span arch over the whole block. Legs only along the two outer
# edges: an internal upright would sit in a working aisle and block the base.
TUNNEL_MARGIN_X = 2.60      # clear ground outside the outer rows
TUNNEL_MARGIN_Y = 2.00      # overhang past the row ends
TUNNEL_LEG_H = 2.20         # straight leg before the arch springs
TUNNEL_RISE = 2.80          # arch rise above the springing -> ridge at 5.00 m
TUNNEL_HOOP_DY = 3.00       # hoop spacing along the rows
TUNNEL_PIPE_R = 0.032       # 64 mm galvanised hoop tube
TUNNEL_ARC_SEGS = 16
FILM_SILL = 0.30            # film starts this far above the springing;
                            # below it the sides are rolled up


def _rng(tag):
    """Independent seeded stream per feature, so adding one does not reshuffle
    the others."""
    return random.Random("%s-%d" % (tag, LAYOUT_SEED))


def row_gaps():
    """The N_ROWS-1 gaps between adjacent rows, in metres."""
    rnd = _rng("gaps")
    return [round(rnd.uniform(*ROW_GAP_RANGE), 3) for _ in range(N_ROWS - 1)]


_GAPS = row_gaps()


def row_x(i):
    """World x of trellis row i. Accumulates the gaps -- there is no pitch."""
    return round(ROW_X0 + sum(_GAPS[:i]), 3)


def lane_x(i):
    """World x the platform drives along while picking row i.

    Offset to the -x side of the row, LANE_STANDOFF away, not the aisle centre.
    """
    return round(row_x(i) - LANE_STANDOFF, 3)


def start_pose():
    """(x, y, yaw) the robot is spawned at: on row 0's lane, south of the block."""
    return (lane_x(0), START_Y, START_YAW)


def aisle_x(k):
    """World x of the centre of aisle k, for terrain scatter and clearance
    checks. Aisle 0 is outside row 0; aisle k>0 runs between rows k-1 and k."""
    if k == 0:
        return round(row_x(0) - TUNNEL_MARGIN_X / 2.0, 3)
    if k >= N_ROWS:
        return round(row_x(N_ROWS - 1) + TUNNEL_MARGIN_X / 2.0, 3)
    return round((row_x(k - 1) + row_x(k)) / 2.0, 3)


def panel_y(p):
    """World y of the centre of panel p within a row."""
    return -ROW_LEN / 2.0 + (p + 0.5) * PANEL_W


def cordon_z(r):
    """Fruiting wire height of row r. Varies row to row."""
    rnd = _rng("cordon-%d" % r)
    return round(rnd.uniform(*CORDON_RANGE), 3)


def canopy_z(r):
    return round(cordon_z(r) + CANOPY_OVER_CORDON, 3)


def tunnel_bounds():
    """(x_left, x_right, y_south, y_north) footprint of the polytunnel."""
    return (round(row_x(0) - TUNNEL_MARGIN_X, 3),
            round(row_x(N_ROWS - 1) + TUNNEL_MARGIN_X, 3),
            round(-ROW_LEN / 2.0 - TUNNEL_MARGIN_Y, 3),
            round(ROW_LEN / 2.0 + TUNNEL_MARGIN_Y, 3))


def bunch_layout():
    """Every cluster in the block, as a list of spec dicts.

    Cluster count per panel, hanging depth, cluster size and berry count are
    all randomised -- a real vineyard is not a grid of identical fruit. The
    hanging depth is randomised through the peduncle length rather than by
    moving the attachment point, so every cluster still hangs off its row's
    cordon the way a real shoot does, while the grasp height genuinely varies.
    """
    rnd = _rng("bunches")
    out = []
    for r in range(N_ROWS):
        x = row_x(r)
        cz = cordon_z(r)
        for p in range(PANELS_PER_ROW):
            cy = panel_y(p)
            n = rnd.choice([1, 2, 2, 3])
            span = PANEL_W - 0.6
            for k in range(n):
                off = 0.0 if n == 1 else (k / float(n - 1) - 0.5) * span
                pedu = round(rnd.uniform(*PEDU_RANGE), 3)
                body = round(rnd.uniform(0.20, 0.32), 3)
                out.append(dict(
                    name="grape_r%dp%d_%d" % (r, p, k),
                    row=r, panel=p,
                    x=x, y=round(cy + off + rnd.uniform(-0.07, 0.07), 3),
                    cordon_z=cz,
                    pedu_len=pedu,
                    body_len=body,
                    r_top=round(rnd.uniform(0.058, 0.082), 3),
                    n_berries=rnd.randint(32, 58),
                    mass=round(rnd.uniform(0.18, 0.32), 3),
                    seed=rnd.randint(1, 100000),
                    # derived, so callers never recompute them inconsistently
                    grasp_z=round(cz - pedu / 2.0, 4),
                    hang_below_tcp=round(pedu / 2.0 + body, 4)))
    return out


# ---------------------------------------------------------------- grape bunch
def grape_bunch(name="grape_bunch", seed=7, n_berries=46,
                pedu_len=0.15, body_len=0.26,
                r_top=0.072, r_bot=0.018, berry_r=0.0145, mass=0.25,
                **_ignored):
    """Model origin sits at the cordon; the peduncle hangs down from z=0."""
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
                "\n        <geometry><cylinder><radius>0.05</radius>"
                "<length>%.2f</length></cylinder></geometry>"
                "\n        <material><ambient>0.20 0.13 0.07 1</ambient>"
                "<diffuse>0.38 0.26 0.14 1</diffuse></material>"
                "\n      </visual>"
                "\n      <collision name=\"c\">"
                "\n        <geometry><cylinder><radius>0.05</radius>"
                "<length>%.2f</length></cylinder></geometry>"
                "\n      </collision>"
                "\n    </link>"
                "\n    <joint name=\"fix_post_%s\" type=\"fixed\">"
                "\n      <parent>world</parent><child>post_%s</child>"
                "\n    </joint>"
                % (tag, x, y, POST_H / 2, POST_H, POST_H, tag, tag))

        # one continuous fruiting wire and one canopy wire per row, at this
        # row's own cordon height
        for wz, wtag in ((cordon_z(r), "fruit"), (canopy_z(r), "canopy")):
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
    rnd = _rng("leaves")
    for r in range(N_ROWS):
        x = row_x(r)
        cz = canopy_z(r)
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
                   rnd.uniform(cz - 0.20, cz + 0.12),
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


# ----------------------------------------------------------------- polytunnel
def _arch_points():
    """Points along one hoop, from the left footing over the ridge to the right.

    Straight legs up to TUNNEL_LEG_H, then a circular arc of rise TUNNEL_RISE.
    For half-span a and rise h the arc radius is (a^2 + h^2) / 2h.
    """
    x0, x1, _, _ = tunnel_bounds()
    a = (x1 - x0) / 2.0
    xc = (x0 + x1) / 2.0
    h = TUNNEL_RISE
    R = (a * a + h * h) / (2.0 * h)
    # centre of the arc sits below the springing line
    zc = TUNNEL_LEG_H + h - R
    half = math.asin(a / R)

    pts = [(x0, 0.0), (x0, TUNNEL_LEG_H)]
    for i in range(TUNNEL_ARC_SEGS + 1):
        t = -half + (2.0 * half) * i / float(TUNNEL_ARC_SEGS)
        pts.append((xc + R * math.sin(t), zc + R * math.cos(t)))
    pts.append((x1, TUNNEL_LEG_H))
    pts.append((x1, 0.0))
    return pts


def greenhouse():
    """One big single-span polytunnel over the whole block.

    Steel hoops on TUNNEL_HOOP_DY centres, tied by ridge and side purlins, with
    the film stretched over them. The uprights are only the two outer legs of
    each hoop -- nothing lands in a working aisle, which is the whole point of
    a single span here: the platform can drive any lane without dodging posts.

    The film is visual-only and does not cast shadows. Giving 300 m^2 of
    polythene a collision surface would cost a lot and buy nothing (the arm
    tops out around 2.2 m, the ridge is at 5 m), and a shadow-casting film
    turns the whole block dark grey in Gazebo's renderer.
    """
    x0, x1, y0, y1 = tunnel_bounds()
    pts = _arch_points()
    n_hoops = int(round((y1 - y0) / TUNNEL_HOOP_DY)) + 1
    hoop_ys = [y0 + i * (y1 - y0) / float(n_hoops - 1) for i in range(n_hoops)]

    parts = []
    idx = 0
    for hy in hoop_ys:
        for (ax, az), (bx, bz) in zip(pts[:-1], pts[1:]):
            dx, dz = bx - ax, bz - az
            L = math.hypot(dx, dz)
            if L < 1e-6:
                continue
            # cylinder's own axis is +z, so pitch it into the chord direction
            pitch = math.atan2(dx, dz)
            parts.append(
                "\n      <visual name=\"hoop_%d\">"
                "\n        <pose>%.4f %.4f %.4f 0 %.5f 0</pose>"
                "\n        <geometry><cylinder><radius>%.4f</radius>"
                "<length>%.4f</length></cylinder></geometry>"
                "\n        <material><ambient>0.42 0.44 0.46 1</ambient>"
                "<diffuse>0.68 0.70 0.72 1</diffuse></material>"
                "\n      </visual>"
                "\n      <collision name=\"hoop_%d_c\">"
                "\n        <pose>%.4f %.4f %.4f 0 %.5f 0</pose>"
                "\n        <geometry><cylinder><radius>%.4f</radius>"
                "<length>%.4f</length></cylinder></geometry>"
                "\n      </collision>"
                % (idx, (ax + bx) / 2.0, hy, (az + bz) / 2.0, pitch,
                   TUNNEL_PIPE_R, L,
                   idx, (ax + bx) / 2.0, hy, (az + bz) / 2.0, pitch,
                   TUNNEL_PIPE_R, L))
            idx += 1

    # purlins tying the hoops together, at the ridge and a few arc stations
    span_y = y1 - y0
    mid_y = (y0 + y1) / 2.0
    arc = pts[2:-2]                       # the arc proper, legs excluded
    for j in (0, len(arc) // 4, len(arc) // 2, 3 * len(arc) // 4, len(arc) - 1):
        px, pz = arc[j]
        parts.append(
            "\n      <visual name=\"purlin_%d\">"
            "\n        <pose>%.4f %.4f %.4f 1.5708 0 0</pose>"
            "\n        <geometry><cylinder><radius>0.020</radius>"
            "<length>%.3f</length></cylinder></geometry>"
            "\n        <material><ambient>0.42 0.44 0.46 1</ambient>"
            "<diffuse>0.68 0.70 0.72 1</diffuse></material>"
            "\n      </visual>"
            "\n      <collision name=\"purlin_%d_c\">"
            "\n        <pose>%.4f %.4f %.4f 1.5708 0 0</pose>"
            "\n        <geometry><cylinder><radius>0.020</radius>"
            "<length>%.3f</length></cylinder></geometry>"
            "\n      </collision>"
            % (j, px, mid_y, pz, span_y, j, px, mid_y, pz, span_y))

    # The film: one thin slab per arc chord, running the full length. It starts
    # above FILM_SILL, leaving the sides open -- that is how a tunnel is
    # actually run through harvest (side film rolled up for ventilation), and
    # it also means the observation camera can see the work rather than
    # everything behind a sheet of grey polythene.
    film = []
    for k, (((ax, az), (bx, bz))) in enumerate(zip(pts[:-1], pts[1:])):
        dx, dz = bx - ax, bz - az
        L = math.hypot(dx, dz)
        if L < 1e-6:
            continue
        if (az + bz) / 2.0 < TUNNEL_LEG_H + FILM_SILL:
            continue
        pitch = math.atan2(dx, dz)
        film.append(
            "\n      <visual name=\"film_%d\">"
            "\n        <pose>%.4f %.4f %.4f 0 %.5f 0</pose>"
            "\n        <cast_shadows>false</cast_shadows>"
            "\n        <geometry><box><size>0.012 %.3f %.4f</size></box></geometry>"
            "\n        <material>"
            "\n          <ambient>0.82 0.86 0.88 0.55</ambient>"
            "\n          <diffuse>0.90 0.94 0.96 0.55</diffuse>"
            "\n          <specular>0.45 0.45 0.45 16</specular>"
            "\n        </material>"
            "\n      </visual>"
            % (k, (ax + bx) / 2.0, mid_y, (az + bz) / 2.0, pitch,
               span_y, L))

    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="polytunnel">
    <static>true</static>
    <link name="frame">%s%s
    </link>
  </model>
</sdf>
""" % ("".join(parts), "".join(film))


# ----------------------------------------------------------------- start pad
def start_pad():
    """A painted datum square at the spawn point.

    Visual only: it is paint, and giving it collision geometry would put a
    2 cm lip under the wheels at the exact moment VIO is initialising. The
    cross marks the base_footprint origin and the arrow points along the
    start heading, so a photograph of the first frame is enough to check the
    robot really did start where the estimator thinks it did.
    """
    x, y, yaw = start_pose()
    h = PAD_SIZE / 2.0
    parts = []

    def slab(name, dx, dy, sx, sy, rgb, z=0.004):
        parts.append(
            "\n      <visual name=\"%s\">"
            "\n        <pose>%.4f %.4f %.4f 0 0 %.4f</pose>"
            "\n        <cast_shadows>false</cast_shadows>"
            "\n        <geometry><box><size>%.3f %.3f 0.008</size></box></geometry>"
            "\n        <material><ambient>%s 1</ambient><diffuse>%s 1</diffuse></material>"
            "\n      </visual>"
            % (name, x + dx, y + dy, z, yaw, sx, sy, rgb, rgb))

    # base square
    slab("pad", 0, 0, PAD_SIZE, PAD_SIZE, "0.78 0.78 0.74")
    # cross on the origin
    slab("cross_x", 0, 0, PAD_SIZE * 0.9, 0.06, "0.80 0.12 0.10", 0.006)
    slab("cross_y", 0, 0, 0.06, PAD_SIZE * 0.9, "0.80 0.12 0.10", 0.006)
    # heading arrow along +y
    slab("arrow", 0, h * 0.55, 0.10, h * 0.5, "0.10 0.35 0.80", 0.006)
    slab("arrow_l", -0.11, h * 0.80, 0.22, 0.09, "0.10 0.35 0.80", 0.006)
    slab("arrow_r", 0.11, h * 0.80, 0.22, 0.09, "0.10 0.35 0.80", 0.006)
    # corner blocks, so the pad reads as a fiducial rather than a puddle
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        slab("corner_%d" % i, sx * h * 0.78, sy * h * 0.78, 0.22, 0.22,
             "0.12 0.12 0.12", 0.006)

    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="start_pad">
    <static>true</static>
    <link name="link">%s
    </link>
  </model>
</sdf>
""" % "".join(parts)


# ------------------------------------------------------------------- terrain
def terrain():
    """Clods and ruts scattered down the aisles.

    Flattened boxes at random yaw rather than spheres: a half-buried sphere
    gives a point contact that launches a rigid four-wheeler, whereas low
    slabs behave like the dried clods and wheel ruts the tyres are chosen for.
    """
    rnd = _rng("terrain")
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


# ------------------------------------------------------------------- camera
def cam_pose():
    """Fixed observation camera, framed on the lane the demo actually works.

    Square on to row 0 and level with the middle of it, so the platform stays
    in shot for the whole traverse. The camera has a 1.15 rad horizontal field
    of view, i.e. 33 degrees either side of the axis; at this standoff the ends
    of the row sit about 26 degrees out, which keeps them inside it. Cameras
    placed off the corner of the block, as earlier versions were, lose the
    robot behind the near rows as soon as it starts driving.
    """
    tx, ty = row_x(0), 0.0
    cx, cy, cz = lane_x(0) - 7.20, 0.0, 3.40
    yaw = math.atan2(ty - cy, tx - cx)
    pitch = math.atan2(cz - 1.55, math.hypot(tx - cx, ty - cy))
    return (round(cx, 3), round(cy, 3), round(cz, 3),
            0.0, round(pitch, 3), round(yaw, 3))


# --------------------------------------------------------------------- world
def world():
    cam = cam_pose()
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
    <include><uri>model://polytunnel</uri><pose>0 0 0 0 0 0</pose></include>
    <include><uri>model://start_pad</uri><pose>0 0 0 0 0 0</pose></include>

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
""" % (cam + cam)


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
          "%dx%d grape trellis block, uneven row spacing, world-anchored"
          % (N_ROWS, PANELS_PER_ROW))
    write("dirt_track", terrain(), "Clods and ruts down the aisles")
    write("polytunnel", greenhouse(),
          "Single-span steel-hoop polytunnel over the whole block")
    write("start_pad", start_pad(),
          "Painted datum square at the fixed run start pose")

    os.makedirs(WORLDS, exist_ok=True)
    with open(os.path.join(WORLDS, "vineyard.world"), "w") as f:
        f.write(world())
    print("  worlds/vineyard.world")

    for stale in ("grape_bunch", "harvest_crate"):
        old = os.path.join(MODELS, stale)
        if os.path.isdir(old):
            for fn in os.listdir(old):
                os.remove(os.path.join(old, fn))
            os.rmdir(old)
            print("  removed stale models/%s" % stale)

    b = bunch_layout()
    x0, x1, y0, y1 = tunnel_bounds()
    print("\nblock: %d rows x %d panels" % (N_ROWS, PANELS_PER_ROW))
    print("row gaps      %s m  (range %.2f-%.2f)"
          % (", ".join("%.2f" % g for g in _GAPS), *ROW_GAP_RANGE))
    print("rows at x =   %s" % ", ".join("%.2f" % row_x(i)
                                         for i in range(N_ROWS)))
    print("work lanes =  %s  (standoff %.2f m)"
          % (", ".join("%.2f" % lane_x(i) for i in range(N_ROWS)),
             LANE_STANDOFF))
    print("cordon z =    %s" % ", ".join("%.2f" % cordon_z(r)
                                         for r in range(N_ROWS)))
    print("polytunnel    x %.2f..%.2f (%.1f m span), y %.2f..%.2f, "
          "ridge %.2f m" % (x0, x1, x1 - x0, y0, y1,
                            TUNNEL_LEG_H + TUNNEL_RISE))
    sx, sy, syaw = start_pose()
    print("start pose    x=%.2f y=%.2f yaw=%.4f  (painted pad %.1f m square)"
          % (sx, sy, syaw, PAD_SIZE))
    print("%d clusters total" % len(b))
    for r in range(N_ROWS):
        per = [len([x for x in b if x["row"] == r and x["panel"] == p])
               for p in range(PANELS_PER_ROW)]
        print("  row %d: %s clusters per panel" % (r, per))
    zs = [x["grasp_z"] for x in b]
    print("grasp height  %.3f .. %.3f m  (target band 1.40-1.70)"
          % (min(zs), max(zs)))

    # The whole point of the CR10 swap: check the geometry still closes.
    ARM_Z = 0.75
    worst = max(math.sqrt(LANE_STANDOFF ** 2 + (z - ARM_Z) ** 2) for z in zs)
    print("worst straight-line reach from the lane: %.3f m "
          "(CR10 envelope ~1.35 m)" % worst)
