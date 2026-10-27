#!/usr/bin/env python3
"""
Drone 3D mapping in CoppeliaSim (ZMQ Remote API, CoppeliaSim >= 4.5).

Scene prerequisite:
  * Open the DEFAULT scene (it has a floor at z = 0).
  * Drag the "Quadcopter" model into the scene (Model browser > robots > mobile),
    keep it near the origin. Its built-in script follows the dummy named "target".

Missions (each one includes the previous grade's work):
  --mode path : Grade C   fly waypoints, draw the flown path in the scene
  --mode 2d   : Grade BC  level, spinning single-row depth sensor -> 2D scan
  --mode 3d   : Grade B   tilted depth camera, lawn-mower in X then Y -> 3D point cloud

  --manual N  : (with --mode path) you drag the "target" dummy yourself for N seconds,
                the script only records and draws the path.

Output (folder ./output): path_<mode>.csv, scan2d.npy, scan3d.npy
Then run postprocess.py for voxel downsampling (AB) and height colour map (A).
"""
import argparse
import math
import os
import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ----------------------------------------------------------------------------
# Parameters (tune these)
# ----------------------------------------------------------------------------
OUT_DIR = "output"

RES = 128                    # square depth sensor, RES x RES pixels
FOV = math.radians(60)       # perspective angle
NEAR, FAR = 0.5, 12.0        # clipping planes [m]; NEAR 0.5 hides the drone's own body

SPEED = 1.5                  # target speed [m/s]
AREA = 4.0                   # half size of the flown area [m]
Z_SAFE = 3.5                 # transit altitude, above all obstacles
Z_PATH = 2.5                 # path mission altitude
Z_2D = 1.0                   # 2D scan altitude (the scan slice height)
Z_3D = 3.0                   # 3D scan altitude

PITCH_3D = math.radians(45)  # sensor tilt down: sees ground, walls AND tops
SCAN_DT_3D = 0.2             # [s] between 3D scans
PIXEL_STRIDE_3D = 2          # use every 2nd pixel (4096 pts/frame)
SPIN_RATE = math.radians(180)  # 2D mode: sensor spins like a lidar [rad/s]

# If your point cloud comes out mirrored, flip these:
FLIP_U = False
FLIP_V = False


# ----------------------------------------------------------------------------
# Small API helpers (setter argument order differs between CoppeliaSim versions)
# ----------------------------------------------------------------------------
_new_order = {"pos": True, "mat": True}


def set_pos(sim, h, p):
    p = [float(v) for v in p]
    if _new_order["pos"]:
        try:
            sim.setObjectPosition(h, p, sim.handle_world)
            return
        except Exception:
            _new_order["pos"] = False
    sim.setObjectPosition(h, sim.handle_world, p)


def set_matrix(sim, h, m):
    m = [float(v) for v in m]
    if _new_order["mat"]:
        try:
            sim.setObjectMatrix(h, m, sim.handle_world)
            return
        except Exception:
            _new_order["mat"] = False
    sim.setObjectMatrix(h, sim.handle_world, m)


def get_pos(sim, h):
    return np.array(sim.getObjectPosition(h, sim.handle_world), dtype=float)


def find(sim, names):
    for n in names:
        try:
            return sim.getObject(n)
        except Exception:
            pass
    return None


def find_drone(sim):
    h = find(sim, ["/Quadcopter", "/Quadricopter"])
    if h is None:
        raise RuntimeError("Quadcopter not found. Drag the Quadcopter model into the scene.")
    return h


def find_target(sim):
    h = find(sim, ["/Quadcopter/target", "/target", "/Quadcopter_target",
                   "/Quadricopter_target", "/Quadricopter/target"])
    if h is not None:
        return h
    for o in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if "target" in str(sim.getObjectAlias(o)).lower():
            return o
    raise RuntimeError("Could not find the quadcopter 'target' dummy.")


# ----------------------------------------------------------------------------
# Scene building
# ----------------------------------------------------------------------------
def build_scene(sim):
    """Boxes and cylinders of different heights to be mapped."""
    objs = [  # (shape, size[x,y,z], (x,y), rgb)
        ("box", [1.0, 1.0, 1.5], (-2.0, -2.0), [0.8, 0.3, 0.3]),
        ("cyl", [1.0, 1.0, 2.0], (2.0, -1.5), [0.3, 0.6, 0.8]),
        ("box", [1.5, 0.8, 1.2], (0.0, 1.5), [0.4, 0.8, 0.4]),
        ("cyl", [0.8, 0.8, 1.4], (-2.5, 2.5), [0.9, 0.8, 0.3]),
        ("box", [1.0, 1.0, 2.2], (2.5, 2.5), [0.7, 0.4, 0.8]),
    ]
    for i, (kind, size, (x, y), rgb) in enumerate(objs):
        shape = sim.primitiveshape_cuboid if kind == "box" else sim.primitiveshape_cylinder
        h = sim.createPrimitiveShape(shape, size, 0)
        sim.setObjectAlias(h, f"obstacle_{i}")
        set_pos(sim, h, [x, y, size[2] / 2])
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, rgb)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    print(f"[scene] created {len(objs)} obstacles")


def make_sensor(sim, drone):
    h = find(sim, ["/mapping_sensor"])
    if h is not None:
        return h
    # options: bit0 = explicit handling, bit1 = perspective mode
    h = sim.createVisionSensor(
        3, [RES, RES, 0, 0],
        [NEAR, FAR, FOV, 0.08, 0.08, 0.08, 0, 0, 0, 0, 0])
    sim.setObjectAlias(h, "mapping_sensor")
    sim.setObjectParent(h, drone, True)
    print("[scene] created vision sensor 'mapping_sensor' on the drone")
    return h


# ----------------------------------------------------------------------------
# Sensor geometry
# ----------------------------------------------------------------------------
def sensor_matrix(pos, yaw, pitch):
    """3x4 matrix (as 12-list) of a sensor at `pos`, looking along heading `yaw`,
    tilted `pitch` radians downward. Sensor: +z = viewing direction, +y = image up."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    z = np.array([cy * cp, sy * cp, -sp])
    y = np.array([sp * cy, sp * sy, cp])
    x = np.cross(y, z)  # right-handed; +x_sensor points to the LEFT of the image
    R = np.stack([x, y, z], axis=1)
    M = np.hstack([R, np.asarray(pos).reshape(3, 1)])
    return M


def depth_to_world(depth, rows, cols, M):
    """Depth image (normalised 0..1 between NEAR and FAR) -> world points."""
    f = (RES / 2.0) / math.tan(FOV / 2.0)
    d = depth[np.ix_(rows, cols)]
    U, V = np.meshgrid(cols, rows)
    if FLIP_U:
        U = RES - 1 - U
    if FLIP_V:
        V = RES - 1 - V
    z = NEAR + d * (FAR - NEAR)
    valid = d < 0.995                       # drop "no hit" pixels at the far plane
    x = -(U - (RES - 1) / 2.0) / f * z      # image right = -x_sensor
    y = (V - (RES - 1) / 2.0) / f * z       # row 0 = bottom row in CoppeliaSim
    pc = np.stack([x, y, z], axis=-1)[valid]
    return pc @ M[:, :3].T + M[:, 3]


def acquire(sim, sensor, drone_pos, yaw, pitch, rows, cols):
    M = sensor_matrix(drone_pos, yaw, pitch)
    set_matrix(sim, sensor, M.reshape(-1).tolist())
    sim.handleVisionSensor(sensor)
    buf, res = sim.getVisionSensorDepth(sensor, 0)
    if isinstance(buf, (bytes, bytearray)):
        arr = np.frombuffer(buf, dtype=np.float32)
    else:
        arr = np.asarray(buf, dtype=np.float32)
    depth = arr.reshape(res[1], res[0])
    return depth_to_world(depth, rows, cols, M)


# ----------------------------------------------------------------------------
# Missions: lists of (waypoint, scan_enabled)
# ----------------------------------------------------------------------------
def transit_wrap(start, route, z_mission):
    """Climb to a safe altitude, go to the route start, descend, fly the route,
    climb again and return to the start."""
    first, last = route[0], route[-1]
    wps = [
        (np.array([start[0], start[1], Z_SAFE]), False),
        (np.array([first[0], first[1], Z_SAFE]), False),
        (np.array([first[0], first[1], z_mission]), False),
    ]
    wps += [(np.array([p[0], p[1], z_mission]), True) for p in route]
    wps += [
        (np.array([last[0], last[1], Z_SAFE]), False),
        (np.array([start[0], start[1], Z_SAFE]), False),
    ]
    return wps


def mission_path(start):
    a = AREA
    route = [(-a, -a), (a, -a), (a, a), (-a, a), (-a, -a)]
    wps = transit_wrap(start, route, Z_PATH)
    return [(w, False) for w, _ in wps]


def mission_2d(start):
    a = AREA
    route = [(-a, -a), (a, -a), (a, a), (-a, a), (-a, -a)]
    return transit_wrap(start, route, Z_2D)


def mission_3d(start):
    a, lanes = AREA, np.arange(-AREA, AREA + 0.1, 2.0)
    route = []
    for i, y in enumerate(lanes):               # pass 1: lanes along X
        xs = (-a - 0.5, a + 0.5) if i % 2 == 0 else (a + 0.5, -a - 0.5)
        route += [(xs[0], y), (xs[1], y)]
    for i, x in enumerate(lanes):               # pass 2: lanes along Y
        ys = (-a - 0.5, a + 0.5) if i % 2 == 0 else (a + 0.5, -a - 0.5)
        route += [(x, ys[0]), (x, ys[1])]
    return transit_wrap(start, route, Z_3D)


# ----------------------------------------------------------------------------
# Flight + acquisition loop
# ----------------------------------------------------------------------------
def fly(sim, client, target, drone, sensor, wps, mode):
    dt = sim.getSimulationTimeStep()
    trace = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 200000, [1.0, 0.1, 0.1])

    tp = get_pos(sim, target)
    prev = get_pos(sim, drone)
    path_log, clouds = [], []
    last_scan, yaw = -1e9, 0.0

    if mode == "3d":
        period, rows, cols = SCAN_DT_3D, np.arange(0, RES, PIXEL_STRIDE_3D), np.arange(0, RES, PIXEL_STRIDE_3D)
    else:  # 2d: one row (the centre one), every step
        period, rows, cols = dt, np.array([RES // 2]), np.arange(RES)

    for k, (wp, do_scan) in enumerate(wps):
        seg = wp - tp
        if math.hypot(seg[0], seg[1]) > 1e-3:
            yaw = math.atan2(seg[1], seg[0])        # camera looks along travel direction
        print(f"[fly] waypoint {k + 1}/{len(wps)} -> {np.round(wp, 2)}  scan={do_scan}")
        while np.linalg.norm(wp - tp) > 1e-6:
            d = wp - tp
            n = np.linalg.norm(d)
            tp = tp + d * min(1.0, SPEED * dt / n)
            set_pos(sim, target, tp)
            client.step()

            dp = get_pos(sim, drone)
            t = sim.getSimulationTime()
            path_log.append([t, *dp.tolist()])
            sim.addDrawingObjectItem(trace, [*prev.tolist(), *dp.tolist()])
            prev = dp

            if do_scan and mode in ("2d", "3d") and t - last_scan >= period - 1e-9:
                last_scan = t
                if mode == "3d":
                    pts = acquire(sim, sensor, dp, yaw, PITCH_3D, rows, cols)
                else:
                    pts = acquire(sim, sensor, dp, SPIN_RATE * t, 0.0, rows, cols)
                if len(pts):
                    clouds.append(pts)

    for _ in range(int(3.0 / dt)):     # let the drone settle
        client.step()
    return np.array(path_log), clouds


def manual_record(sim, client, drone, seconds):
    """You drag the 'target' dummy in the GUI; we just record and draw the path."""
    dt = sim.getSimulationTimeStep()
    trace = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 200000, [1.0, 0.1, 0.1])
    prev, log = get_pos(sim, drone), []
    print(f"[manual] move the 'target' dummy yourself for {seconds} s ...")
    for _ in range(int(seconds / dt)):
        client.step()
        dp = get_pos(sim, drone)
        log.append([sim.getSimulationTime(), *dp.tolist()])
        sim.addDrawingObjectItem(trace, [*prev.tolist(), *dp.tolist()])
        prev = dp
        time.sleep(dt)               # roughly real time so you can steer
    return np.array(log)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["path", "2d", "3d"], default="3d")
    ap.add_argument("--manual", type=float, default=0.0,
                    help="path mode: record N seconds while YOU move the target dummy")
    args = ap.parse_args()

    client = RemoteAPIClient()
    sim = client.require("sim") if hasattr(client, "require") else client.getObject("sim")

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.1)

    drone = find_drone(sim)
    target = find_target(sim)
    if find(sim, ["/obstacle_0"]) is None:
        build_scene(sim)
    sensor = make_sensor(sim, drone)

    client.setStepping(True)
    sim.startSimulation()
    client.step()

    start = get_pos(sim, target)
    os.makedirs(OUT_DIR, exist_ok=True)

    if args.mode == "path" and args.manual > 0:
        path = manual_record(sim, client, drone, args.manual)
        clouds = []
    else:
        wps = {"path": mission_path, "2d": mission_2d, "3d": mission_3d}[args.mode](start)
        path, clouds = fly(sim, client, target, drone, sensor, wps, args.mode)

    np.savetxt(f"{OUT_DIR}/path_{args.mode}.csv", path, delimiter=",",
               header="t,x,y,z", comments="")
    print(f"[save] {OUT_DIR}/path_{args.mode}.csv  ({len(path)} samples)")

    if clouds:
        pts = np.vstack(clouds).astype(np.float32)
        np.save(f"{OUT_DIR}/scan{args.mode}.npy", pts)
        print(f"[save] {OUT_DIR}/scan{args.mode}.npy  ({len(pts)} points)")

    client.setStepping(False)
    sim.stopSimulation()
    print("[done]")


if __name__ == "__main__":
    main()
