#!/usr/bin/env python3
"""Regenerates the BetaBot STL meshes and dimensions.json from the Polycam
3D scan (glTF binary, meshes/betabot_scan.glb), using only the standard
library.

The scan is metric (Polycam LiDAR photogrammetry), Y-up (glTF convention),
with the robot front pointing towards -Z of the scan. Output STLs are in
ROS conventions (X forward, Y left, Z up), via:

    X_ros = -Z_scan, Y_ros = -X_scan, Z_ros = Y_scan

Segmentation is done on face centroids along the scan vertical axis:
base (y<0.13), torso (0.13..0.26), arms (0.26..0.36), head (>=0.36).
Each link STL is re-anchored on its joint origin (see dimensions.json).
"""
import json
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
GLB = os.path.join(HERE, "meshes", "betabot_scan.glb")
MESH_DIR = os.path.join(HERE, "meshes")

# ROS <- scan axes
M = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]

SEGMENTS = {
    "base": (0.0, 0.13),
    "torso": (0.13, 0.26),
    "arms": (0.26, 0.36),
    "head": (0.36, 10.0),
}
JOINT_ORIGIN = {
    "base": [0.0, 0.0, 0.025],
    "torso": [0.0, 0.0, 0.207],
    "arms": [0.0, 0.0, 0.285],
    "head": [0.0, 0.0, 0.356],
}


def read_glb(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, version, length = struct.unpack("<III", data[:12])
    if magic != 0x46546C67:
        raise ValueError("not a glTF binary file")
    clen, _ctype = struct.unpack("<I4s", data[12:20])
    gltf = json.loads(data[20:20 + clen])
    off = 20 + clen
    blen, _btype = struct.unpack("<I4s", data[off:off + 8])
    bindata = data[off + 8:off + 8 + blen]
    return gltf, bindata


def read_accessor(gltf, bindata, idx):
    acc = gltf["accessors"][idx]
    bv = gltf["bufferViews"][acc["bufferView"]]
    o = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    count = acc["count"]
    ctypes = {5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2),
              5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}
    fmt, sz = ctypes[acc["componentType"]]
    comp = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[acc["type"]]
    stride = bv.get("byteStride", 0)
    out = []
    if stride == 0:
        stride = sz * comp
    for i in range(count):
        out.append(struct.unpack_from("<" + fmt * comp, bindata, o + i * stride))
    if acc["type"] == "SCALAR":
        return [v[0] for v in out]
    return [list(v) for v in out]


def transform(v):
    return [sum(M[i][k] * v[k] for k in range(3)) for i in range(3)]


def write_stl(path, verts, faces):
    tris = []
    for f in faces:
        p0, p1, p2 = (verts[i] for i in f)
        u = [p1[k] - p0[k] for k in range(3)]
        w = [p2[k] - p0[k] for k in range(3)]
        n = [u[1] * w[2] - u[2] * w[1],
             u[2] * w[0] - u[0] * w[2],
             u[0] * w[1] - u[1] * w[0]]
        ln = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1.0
        tris.append((n[0] / ln, n[1] / ln, n[2] / ln, p0, p1, p2))
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(tris)))
        for t in tris:
            fh.write(struct.pack("<3f", t[0], t[1], t[2]))
            for p in (t[3], t[4], t[5]):
                fh.write(struct.pack("<3f", *p))
            fh.write(b"\0\0")


def main():
    gltf, bindata = read_glb(GLB)
    prim = gltf["meshes"][0]["primitives"][0]
    faces = read_accessor(gltf, bindata, prim["indices"])
    faces = [faces[i:i + 3] for i in range(0, len(faces), 3)]
    positions = read_accessor(gltf, bindata, prim["attributes"]["POSITION"])

    stats = {}
    for name, (lo, hi) in SEGMENTS.items():
        sel = [i for i, f in enumerate(faces)
               if lo <= sum(positions[v][1] for v in f) / 3.0 < hi]
        idxmap = {}
        verts = []
        tris = []
        for i in sel:
            tri = []
            for v in faces[i]:
                if v not in idxmap:
                    idxmap[v] = len(verts)
                    p = transform(positions[v])
                    verts.append([p[k] - JOINT_ORIGIN[name][k] for k in range(3)])
                tri.append(idxmap[v])
            tris.append(tri)
        path = os.path.join(MESH_DIR, "betabot_%s.stl" % name)
        write_stl(path, verts, tris)
        xs = [v[0] for v in verts]
        ys = [v[1] for v in verts]
        zs = [v[2] for v in verts]
        stats[name] = {
            "faces": len(tris),
            "min_link_m": [min(xs), min(ys), min(zs)],
            "max_link_m": [max(xs), max(ys), max(zs)],
            "size_kb": os.path.getsize(path) // 1024,
        }
        print("%s: %d faces -> %s" % (name, len(tris), path))

    # full robot (anchored at ground level, base_footprint)
    verts = []
    for p in positions:
        q = transform(p)
        verts.append(q)
    full_faces = [list(f) for f in faces]
    path = os.path.join(MESH_DIR, "betabot_full.stl")
    write_stl(path, verts, full_faces)
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    stats["full"] = {
        "faces": len(full_faces),
        "min_link_m": [min(xs), min(ys), min(zs)],
        "max_link_m": [max(xs), max(ys), max(zs)],
        "size_kb": os.path.getsize(path) // 1024,
    }
    print("full: %d faces -> %s" % (len(full_faces), path))

    dims = {
        "source": "scan Polycam (LiDAR photogrammetrie) meshes/betabot_scan.glb",
        "echelle": {
            "polycam_unite": "metre",
            "validation": {
                "tete_brique_NXT_ref98_mm": 108.4,
                "tete_mesuree_scan_mm": 115.5,
                "ecart_pourcent": 6.5,
                "tread_radius_nxt_controllers_m": 0.025,
                "tread_radius_scan_m": 0.025,
            },
        },
        "repere_scan_vers_ros": {
            "X_ros": "-Z_scan", "Y_ros": "-X_scan", "Z_ros": "Y_scan",
            "note": "glTF est Y-up ; l'avant du robot scanne pointe vers -Z du scan",
        },
        "dimensions_mm": {
            "longueur_X": (max(xs) - min(xs)) * 1000,
            "largeur_Y": (max(ys) - min(ys)) * 1000,
            "hauteur_Z": (max(zs) - min(zs)) * 1000,
        },
        "joints_world_m": {k: v for k, v in JOINT_ORIGIN.items()},
        "segmentation_y_scan_m": {k: list(v) for k, v in SEGMENTS.items()},
        "meshes": stats,
    }
    with open(os.path.join(HERE, "dimensions.json"), "w") as fh:
        json.dump(dims, fh, indent=2)
    print("dimensions.json mis a jour")


if __name__ == "__main__":
    main()
