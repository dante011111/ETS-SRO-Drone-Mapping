#!/usr/bin/env python3
"""
Post-processing of the drone scan.

  python postprocess.py --mode 3d --voxel 0.15        # AB + A (voxel + height colours)
  python postprocess.py --mode 2d                      # top-view of the 2D scan
  python postprocess.py --mode 3d --o3d                # also open an Open3D window
  python postprocess.py --mode 3d --raw                # also show the raw point cloud
  python postprocess.py --mode 3d --size 8             # bigger voxel markers

Needs: numpy, matplotlib   (optional: open3d)
"""
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np

OUT_DIR = "output"
CMAP = "RdYlGn_r"      # green = ground, yellow = mid, red = tallest


# ---------------------------------------------------------------------------
def load_scan(mode):
    """Reads scan<mode>.npy (Python flight script) or scan<mode>.csv (Lua script).
    Files are looked up in ./output first, then in the current folder."""
    for d in (OUT_DIR, "."):
        npy = os.path.join(d, f"scan{mode}.npy")
        csv = os.path.join(d, f"scan{mode}.csv")
        if os.path.exists(npy):
            return np.load(npy)
        if os.path.exists(csv):
            return np.loadtxt(csv, delimiter=",", skiprows=1)
    raise FileNotFoundError(f"scan{mode}.npy / scan{mode}.csv not found")


def clean(pts):
    """Remove NaNs and points clearly below the ground."""
    pts = pts[np.isfinite(pts).all(axis=1)]
    return pts[pts[:, 2] > -0.1]


def voxel_downsample(pts, size):
    """Replace all points inside a voxel by one point -> returns voxel centres."""
    idx = np.floor(pts / size).astype(np.int64)
    uniq = np.unique(idx, axis=0)
    return (uniq + 0.5) * size


def height_colors(pts, cmap=CMAP, pad=0.25):
    """Colour by height. `pad` shifts the low end so the ground is a light green
    instead of the darkest green of the colour map."""
    z = pts[:, 2]
    zmin, zmax = z.min(), z.max()
    rng = max(zmax - zmin, 1e-9)
    zn = (z - (zmin - pad * rng)) / (rng * (1 + pad))
    return plt.get_cmap(cmap)(zn)[:, :3], zmin, zmax


def save_ply(path, pts, rgb):
    rgb8 = (rgb * 255).astype(np.uint8)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(pts, rgb8):
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {c[0]} {c[1]} {c[2]}\n")


def equal_axes(ax, pts):
    mid = (pts.max(0) + pts.min(0)) / 2
    r = (pts.max(0) - pts.min(0)).max() / 2
    ax.set_xlim(mid[0] - r, mid[0] + r)
    ax.set_ylim(mid[1] - r, mid[1] + r)
    ax.set_zlim(max(mid[2] - r, 0), mid[2] + r)


def load_path(mode):
    for d in (OUT_DIR, "."):
        p = os.path.join(d, f"path_{mode}.csv")
        if os.path.exists(p):
            return np.loadtxt(p, delimiter=",", skiprows=1)
    return None


# ---------------------------------------------------------------------------
def run_3d(args):
    os.makedirs(OUT_DIR, exist_ok=True)
    pts = clean(load_scan("3d"))
    vox = voxel_downsample(pts, args.voxel)
    rgb_raw, zmin, zmax = height_colors(pts)
    rgb_vox, _, _ = height_colors(vox)

    print(f"raw points   : {len(pts)}")
    print(f"voxels ({args.voxel} m): {len(vox)}  "
          f"(reduced {100 * (1 - len(vox) / len(pts)):.1f}%)")
    print(f"height range : {zmin:.2f} .. {zmax:.2f} m")

    save_ply(os.path.join(OUT_DIR, "cloud_colored.ply"), pts[::max(1, len(pts) // 200000)],
             rgb_raw[::max(1, len(pts) // 200000)])
    save_ply(os.path.join(OUT_DIR, "voxels_colored.ply"), vox, rgb_vox)

    path = load_path("3d")

    # ---- single "3D Map" figure: voxels coloured by height + flown path in blue
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(vox[:, 0], vox[:, 1], vox[:, 2], c=rgb_vox,
               s=args.size, marker="s", linewidths=0, depthshade=False)
    allp = vox
    if path is not None:
        ax.plot(path[:, 1], path[:, 2], path[:, 3], color="blue", lw=2)
        allp = np.vstack([vox, path[:, 1:4]])
    lo, hi = allp.min(0), allp.max(0)
    ax.set_xlim(lo[0] - 0.2, hi[0] + 0.2)
    ax.set_ylim(lo[1] - 0.2, hi[1] + 0.2)
    ax.set_zlim(min(lo[2], 0) - 0.1, hi[2] + 0.1)
    ax.set_box_aspect((hi - lo) + 0.4)          # true metric proportions
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("3D Map")
    fig.savefig(os.path.join(OUT_DIR, "map3d.png"), dpi=150)

    if args.raw:                                # optional: raw cloud in its own window
        fig2 = plt.figure(figsize=(8, 7))
        ax2 = fig2.add_subplot(111, projection="3d")
        sub = pts[:: max(1, len(pts) // 40000)]
        ax2.scatter(sub[:, 0], sub[:, 1], sub[:, 2], c=height_colors(sub)[0],
                    s=1, depthshade=False)
        ax2.set_box_aspect(pts.max(0) - pts.min(0))
        ax2.set_xlabel("X"); ax2.set_ylabel("Y"); ax2.set_zlabel("Z")
        ax2.set_title(f"Raw point cloud ({len(pts)} pts)")

    if args.o3d:
        try:
            import open3d as o3d
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(pts)
            pc.colors = o3d.utility.Vector3dVector(rgb_raw)
            vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pc, voxel_size=args.voxel)
            o3d.visualization.draw_geometries([vg])
        except ImportError:
            print("open3d not installed: pip install open3d")
    plt.show()


def run_2d(args):
    os.makedirs(OUT_DIR, exist_ok=True)
    pts = clean(load_scan("2d"))
    path = load_path("2d")
    print(f"2D scan points: {len(pts)}")
    plt.figure(figsize=(7, 7))
    plt.scatter(pts[:, 0], pts[:, 1], s=2, c="tab:blue", label="scan hits")
    if path is not None:
        plt.plot(path[:, 1], path[:, 2], "r-", lw=1, label="drone path")
    plt.axis("equal"); plt.grid(True)
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.title("2D scan (top view)")
    plt.legend()
    plt.savefig(os.path.join(OUT_DIR, "map2d.png"), dpi=150)
    plt.show()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["2d", "3d"], default="3d")
    ap.add_argument("--voxel", type=float, default=0.15, help="voxel edge [m]")
    ap.add_argument("--o3d", action="store_true")
    ap.add_argument("--raw", action="store_true", help="also show the raw point cloud")
    ap.add_argument("--size", type=float, default=6.0, help="voxel marker size in the plot")
    a = ap.parse_args()
    run_3d(a) if a.mode == "3d" else run_2d(a)