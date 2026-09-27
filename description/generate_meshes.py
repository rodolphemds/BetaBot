#!/usr/bin/env python3
"""Genere les meshes STL de BetaBot pour ROS a partir du modele LEGO
BrickLink Studio "meshes/lego model.io" presente dans ce depot.

Principe
--------
1. Le fichier .io est une archive ZIP protegee par mot de passe
   (format BrickLink Studio, mot de passe public "soho0909").
   Elle contient "model2.ldr" : le modele LDraw avec les definitions
   de pieces embarquees (MPD multi-fichiers).
2. Le LDraw utilise le LDU (LDraw Unit) :
       1 stud = 20 LDU = 8.0 mm  =>  1 LDU = 0.4 mm.
   Echelle validee sur les axes Technic du modele :
       Axle 2 = 40 LDU = 16 mm ... Axle 12 = 240 LDU = 96 mm (exacts).
3. Les triangles des pieces sont collectes recursivement (types 3/4,
   sous-references type 1), transformes en repere ROS :
       x_ros = (z_ldr - z0) * 0.4 mm   (avant du robot = +Z LDraw)
       y_ros = (x_ldr - x0) * 0.4 mm
       z_ros = (y_ldr - sol) * 0.4 mm  (Y LDraw = verticale)
   avec le sol au point de contact des pieds (Y_ldr = -740.6).
4. Chaque section (base, torso, head, arms) est exportee en STL binaire
   dans le repere de son link URDF (decimation voxel 2.5 mm).

Sorties : meshes/betabot_{base,torso,head,arms,full}.stl + dimensions.json
"""
import json
import math
import os
import struct
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
IO_FILE = os.path.join(HERE, "meshes", "lego model.io")
IO_PASSWORD = b"soho0909"
LDU_MM = 0.4

# --- Constantes mesurees sur le modele (LDU) --------------------------------
# Contact au sol (extremites des liftarms 32009, les "pieds")
GROUND_Y = -740.61544
# Origine du robot : axe des roues motrices NXT (X_ldr = -1109, Z_ldr = 1214.4)
AXLE_X = -1109.0
AXLE_Z = 1214.4
# Decimation voxel par link (m)
PITCH = {"base": 0.0025, "torso": 0.004, "head": 0.0025, "arms": 0.0025, "full": 0.0025}

#--- MPD parsing -------------------------------------------------------------


def norm(name):
    return name.replace("\\", "/").lower()


def parse_mpd(text):
    sections = {}
    cur = None
    for line in text.splitlines():
        line = line.rstrip("\r")
        if line.startswith("0 FILE "):
            cur = norm(line[7:].strip())
            sections[cur] = []
        elif cur is not None:
            sections[cur].append(line)
    return sections


def mmul(a, b):
    return [sum(a[i + k] * b[k * 3 + j] for k in range(3)) for i in range(3) for j in range(3)]


def ap(m, t, p):
    return [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + t[0],
        m[3] * p[0] + m[4] * p[1] + m[5] * p[2] + t[1],
        m[6] * p[0] + m[7] * p[1] + m[8] * p[2] + t[2],
    ]


def collect(sections, name, m, t, out, depth=0):
    """Collecte recursive des triangles d'une piece (coordonnees monde LDU)."""
    if depth > 30:
        return
    lines = sections.get(norm(name))
    if lines is None:
        return
    for line in lines:
        tok = line.split()
        if not tok:
            continue
        lt = tok[0]
        if lt == "3" and len(tok) >= 11:
            try:
                c = [float(x) for x in tok[2:11]]
            except ValueError:
                continue
            out.append([ap(m, t, c[0:3]), ap(m, t, c[3:6]), ap(m, t, c[6:9])])
        elif lt == "4" and len(tok) >= 14:
            try:
                c = [float(x) for x in tok[2:14]]
            except ValueError:
                continue
            out.append([ap(m, t, c[0:3]), ap(m, t, c[3:6]), ap(m, t, c[6:9])])
            out.append([ap(m, t, c[0:3]), ap(m, t, c[6:9]), ap(m, t, c[9:12])])
        elif lt == "1" and len(tok) >= 15:
            try:
                tt = [float(x) for x in tok[2:5]]
                mm = [float(x) for x in tok[5:14]]
            except ValueError:
                continue
            nm = mmul(m, mm)
            nt = [
                m[0] * tt[0] + m[1] * tt[1] + m[2] * tt[2] + t[0],
                m[3] * tt[0] + m[4] * tt[1] + m[5] * tt[2] + t[1],
                m[6] * tt[0] + m[7] * tt[1] + m[8] * tt[2] + t[2],
            ]
            collect(sections, tok[14], nm, nt, out, depth + 1)


def bbox(tris):
    mn = [math.inf] * 3
    mx = [-math.inf] * 3
    for tri in tris:
        for p in tri:
            for i in range(3):
                mn[i] = min(mn[i], p[i])
                mx[i] = max(mx[i], p[i])
    return mn, mx


def ldu_to_ros(p):
    return (
        (p[2] - AXLE_Z) * LDU_MM / 1000.0,
        (p[0] - AXLE_X) * LDU_MM / 1000.0,
        (p[1] - GROUND_Y) * LDU_MM / 1000.0,
    )


# --- Origines des joints, mesurees sur le modele (monde ROS, sol = z 0) ------
# base_link : axe des chenilles/roues motrices (coherent avec nxt_controllers :
#             tread_radius = 0.025 m)
# torso     : axe du moteur NXT du torse  (t_ldr = (-1109, -222.1, 1238.5))
# head      : axe du cou, brique NXT       (t_ldr = (-1109,   83.0, 1329.0))
# arms      : axe des moteurs d'epaules    (t_ldr = (-1109,  163.0, 1685.0))
JOINTS_WORLD = {
    "base": (0.0, 0.0, 0.025),
    "torso": (0.0096, 0.0, 0.2074),
    "head": (0.0458, 0.0, 0.3294),
    "arms": (0.1882, 0.0, 0.3614),
}


def to_link(p, link):
    wx, wy, wz = ldu_to_ros(p)
    ox, oy, oz = JOINTS_WORLD[link]
    return (wx - ox, wy - oy, wz - oz)


def export_stl(path, tris, link, pitch=PITCH):
    verts = []
    vmap = {}
    faces = []
    for tri in tris:
        f = []
        for p in tri:
            x, y, z = to_link(p, link)
            key = (round(x / pitch), round(y / pitch), round(z / pitch))
            if key not in vmap:
                vmap[key] = len(verts)
                verts.append((x, y, z))
            f.append(vmap[key])
        if len(set(f)) == 3:
            faces.append(f)
    seen = set()
    uniq = []
    for f in faces:
        k = tuple(sorted(f))
        if k not in seen:
            seen.add(k)
            uniq.append(f)
    faces = uniq
    tris2 = [(verts[a], verts[b], verts[c]) for a, b, c in faces]
    with open(path, "wb") as fh:
        fh.write(b"BetaBot LEGO Mindstorms NXT robot".ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(faces)))
        buf = bytearray()
        for a, b, c in tris2:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            ln = math.sqrt(sum(x * x for x in n)) or 1.0
            buf += struct.pack("<3f", n[0] / ln, n[1] / ln, n[2] / ln)
            for vv in (a, b, c):
                buf += struct.pack("<3f", *vv)
            buf += struct.pack("<H", 0)
        fh.write(buf)
    mn = [min(min(a[j], b[j], c[j]) for a, b, c in tris2) for j in range(3)]
    mx = [max(max(a[j], b[j], c[j]) for a, b, c in tris2) for j in range(3)]
    return {
        "faces": len(faces),
        "vertices": len(verts),
        "min_link_m": [round(x, 4) for x in mn],
        "max_link_m": [round(x, 4) for x in mx],
        "size_kb": os.path.getsize(path) // 1024,
    }


def classify(ref, mn, mx):
    """Assigne une piece a un link URDF selon sa nature et sa hauteur."""
    ymin, ymax = mn[1], mx[1]
    if ref in ("53788.dat", "55969.dat", "40490.dat"):
        return "head"
    if ref in ("2695c01.dat", "53793.dat"):
        return "arms"
    if ref == "41239.dat" and ymin >= 100:
        return "arms"
    if ymax <= -260:
        return "base"
    return "torso"


def main():
    with zipfile.ZipFile(IO_FILE) as z:
        mpd = z.read("model2.ldr", pwd=IO_PASSWORD).decode("utf-8-sig", errors="replace")
    sections = parse_mpd(mpd)
    main_lines = sections[norm("Lego model.io")]

    groups = {"base": [], "torso": [], "head": [], "arms": []}
    n_parts = 0
    for line in main_lines:
        tok = line.split()
        if len(tok) >= 15 and tok[0] == "1":
            try:
                t = [float(x) for x in tok[2:5]]
                m = [float(x) for x in tok[5:14]]
            except ValueError:
                continue
            tris = []
            collect(sections, tok[14], m, t, tris, 0)
            if not tris:
                continue
            mn, mx = bbox(tris)
            cx = (mn[0] + mx[0]) / 2.0
            cz = (mn[2] + mx[2]) / 2.0
            # cluster du robot assemble (le fichier contient aussi des pieces detachees)
            if -1250 <= cx <= -900 and cz >= 990:
                n_parts += 1
                groups[classify(tok[14], mn, mx)].append(tris)

    print("pieces du robot assembles:", n_parts)
    meshes = os.path.join(HERE, "meshes")
    results = {}
    all_tris = []
    for link, tris_list in groups.items():
        tris = [t for lst in tris_list for t in lst]
        all_tris.extend(tris)
        path = os.path.join(meshes, "betabot_%s.stl" % link)
        results[link] = export_stl(path, tris, link, PITCH[link])
        print(link, results[link])
    results["full"] = export_stl(os.path.join(meshes, "betabot_full.stl"), all_tris, "base", PITCH["full"])
    print("full", results["full"])

    # Rapport de cotes
    wmn = [min(min(p[i] for p in tri) for tri in all_tris) for i in range(3)]
    wmx = [max(max(p[i] for p in tri) for tri in all_tris) for i in range(3)]
    size_ldu = [wmx[i] - wmn[i] for i in range(3)]
    size_mm = [d * LDU_MM for d in size_ldu]
    report = {
        "echelle": {"1_LDU_mm": LDU_MM, "1_stud_LDU": 20, "1_stud_mm": 8.0},
        "robot_ldraw_bbox_ldu": {
            "X_largeur": [round(wmn[0], 1), round(wmx[0], 1)],
            "Y_hauteur": [round(wmn[1], 1), round(wmx[1], 1)],
            "Z_longueur": [round(wmn[2], 1), round(wmx[2], 1)],
        },
        "dimensions_mm": {
            "largeur": round(size_mm[0], 1),
            "hauteur": round(size_mm[1], 1),
            "longueur": round(size_mm[2], 1),
        },
        "joints_world_m": {k: v for k, v in JOINTS_WORLD.items()},
        "meshes": results,
    }
    with open(os.path.join(HERE, "dimensions.json"), "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(json.dumps(report["dimensions_mm"], indent=2))


if __name__ == "__main__":
    main()
