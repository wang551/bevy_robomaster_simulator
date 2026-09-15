#!/usr/bin/env python3
"""Compress sub-13mm CAD construction artifacts in GROUND_RMUL.glb to 1.0-1.4mm.

GROUND_RMUL.glb is a direct CAD export of the RMUL field. Besides the intentional
terrain (200/400mm plateaus, ramps, perimeter walls) its SOLID mesh carries
construction plates that stick a few millimeters above the drivable floor:

  * a 12mm base plate under the central pad (with a 1mm mismatched seam on top),
  * 3mm corner plates, 2mm corner pads and a 2-3mm lip along the wall base.

Simulator vehicles are flat-bottomed cylinders pushed by purely horizontal
impulses (CollisionMargin 5mm), so every step above ~5mm is a hard wall and the
center of the field becomes impassable.

This tool compresses every SOLID vertex whose world height lies in (1.5mm,
13.5mm] linearly into the 1.000..1.400mm range: below the 5mm collision margin
(physically flat, like the RMUC floor), yet above the 0.000 main floor so the
painted plate surfaces stay strictly on top of it. Crucially the mapping is
monotonic rather than a clamp-to-one-plane — the 13mm seam overlaps the 12mm pad
top in XZ, and flattening both onto one identical height made them z-fight
(visible flicker at field center). Compression keeps distinct CAD heights
distinct, so no two faces ever share a plane. Vertex count, topology, normals
and per-primitive materials are untouched — the field markings survive verbatim,
just laid almost onto the floor.

World Y = -local Y: the scene root carries a 180-degree rotation about
(1, 0, -1)/sqrt(2), asserted below rather than assumed.

The tool is idempotent (outputs land below the input band) and refuses assets
whose structure differs from the known export (re-export the official model,
then re-run this before shipping it):

    python tools/flatten_rmul_floor.py assets/GROUND_RMUL.glb

Git history is the rollback net for the binary asset.
"""

from __future__ import annotations

import json
import struct
import sys
from collections import Counter
from pathlib import Path

GLB_MAGIC = 0x46546C67  # 'glTF'

# World heights in this half-open band get COMPRESSED (not clamped): the band
# 2..13mm maps linearly onto 1.000..1.400mm. Compression, not clamping to one
# plane, is essential: the 13mm seam lies 1mm above the 12mm pad top with the
# same XZ footprint, and clamping both onto one height made them z-fight
# (visible flicker at field center). Distinct CAD heights stay distinct.
# Everything that matters for gameplay (floor 0.000, plateaus/walls 0.200+)
# sits outside the band. Bounds are padded by half a millimeter because CAD
# values round-trip through float32 imprecisely (the 13mm seam reads back as
# 0.0130000002...), and outputs stay below BAND_MIN so re-runs are no-ops.
BAND_MIN, BAND_MAX = 0.0015, 0.0135
BAND_FLOOR = 0.002  # heights at/below this map flat onto the low target
LOW_TARGET, HIGH_TARGET = 0.001, 0.0014
SCALE = (HIGH_TARGET - LOW_TARGET) / (0.013 - BAND_FLOOR)


def compress(world_y: float) -> float:
    return LOW_TARGET + (min(max(world_y, BAND_FLOOR), 0.013) - BAND_FLOOR) * SCALE

# Root rotation as (x, y, z, w): 180 degrees about (1, 0, -1)/sqrt(2).
EXPECTED_ROOT_ROTATION = (0.7071067811865476, 0.0, -0.7071067811865476, 0.0)
ROTATION_TOL = 1e-3


# --------------------------------------------------------------- container

def read_glb(path: Path) -> tuple[dict, bytes]:
    data = path.read_bytes()
    if len(data) < 12 or struct.unpack_from("<I", data)[0] != GLB_MAGIC:
        sys.exit(f"error: {path} is not a GLB container")
    version, total = struct.unpack_from("<II", data, 4)
    if version != 2:
        sys.exit(f"error: unsupported GLB version {version}")
    doc, bin_chunk, offset = None, b"", 12
    while offset < total:
        clen = struct.unpack_from("<I", data, offset)[0]
        ctype = data[offset + 4 : offset + 8]
        body = data[offset + 8 : offset + 8 + clen]
        if ctype == b"JSON":
            doc = json.loads(body.decode("utf-8"))
        elif ctype == b"BIN\x00":
            bin_chunk = body
        offset += 8 + clen
    if doc is None:
        sys.exit("error: GLB has no JSON chunk")
    return doc, bin_chunk


def write_glb(path: Path, doc: dict, bin_chunk: bytes) -> None:
    json_bytes = json.dumps(doc, ensure_ascii=False, separators=(",", ":")).encode()
    json_bytes += b" " * ((4 - len(json_bytes) % 4) % 4)
    chunks = [(b"JSON", json_bytes)]
    if bin_chunk:
        pad = b"\x00" * ((4 - len(bin_chunk) % 4) % 4)
        chunks.append((b"BIN\x00", bin_chunk + pad))
    out = struct.pack("<III", GLB_MAGIC, 2, 12 + sum(8 + len(b) for _, b in chunks))
    for ctype, body in chunks:
        out += struct.pack("<I", len(body)) + ctype + body
    path.write_bytes(out)


# ------------------------------------------------------------------ checks

def fail_unless(cond: bool, message: str) -> None:
    if not cond:
        sys.exit(f"error: {message}\n(refusing: asset does not match the known export — re-check before flattening)")


def near(a: float, b: float, tol: float) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def assert_known_structure(doc: dict) -> dict:
    nodes = doc.get("nodes", [])
    by_name = {n.get("name"): n for n in nodes}
    for name in ("SHELL", "SOLID"):
        fail_unless(name in by_name, f"node {name!r} not found")
        node = by_name[name]
        fail_unless("mesh" in node, f"node {name!r} has no mesh")
        fail_unless(
            not any(k in node for k in ("translation", "rotation", "scale", "matrix")),
            f"node {name!r} unexpectedly transformed",
        )
    fail_unless(
        len(doc["meshes"][by_name["SOLID"]["mesh"]]["primitives"]) == 8,
        "SOLID mesh does not have exactly 8 primitives",
    )

    roots = [r for s in doc.get("scenes", []) for r in s.get("nodes", [])]
    fail_unless(len(roots) == 1, f"expected a single scene root, found {roots}")
    root = nodes[roots[0]]
    fail_unless(
        set(root.get("children", []))
        == {i for i, n in enumerate(nodes) if n.get("name") in ("SHELL", "SOLID")},
        "scene root does not parent exactly SHELL and SOLID",
    )
    fail_unless("translation" not in root and "scale" not in root and "matrix" not in root,
                "scene root carries translation/scale/matrix")
    fail_unless(
        "rotation" in root
        and near(root["rotation"], EXPECTED_ROOT_ROTATION, ROTATION_TOL),
        f"scene root rotation {root.get('rotation')} is not the expected 180deg about (1,0,-1)/sqrt(2)",
    )
    return by_name


def position_accessor(doc: dict, prim: dict) -> tuple[dict, dict, int]:
    """(accessor, bufferView, byte offset of the first vertex in the BIN chunk)."""
    acc_idx = prim.get("attributes", {}).get("POSITION")
    fail_unless(acc_idx is not None, "primitive without a POSITION attribute")
    acc = doc["accessors"][acc_idx]
    fail_unless(acc.get("componentType") == 5126, "POSITION is not float32")
    fail_unless(acc.get("type") == "VEC3", "POSITION is not VEC3")
    fail_unless("sparse" not in acc, "sparse POSITION accessor not supported")
    view = doc["bufferViews"][acc["bufferView"]]
    fail_unless(view.get("byteStride", 0) in (0, 12), "POSITION bufferView is interleaved")
    fail_unless(
        view["byteLength"] >= acc["count"] * 12,
        "POSITION bufferView smaller than accessor count",
    )
    return acc, view, view["byteOffset"] + acc.get("byteOffset", 0)


# ------------------------------------------------------------------- main

def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} <GROUND_RMUL.glb>")
    path = Path(sys.argv[1])
    doc, bin_chunk = read_glb(path)
    by_name = assert_known_structure(doc)

    solid_prims = doc["meshes"][by_name["SOLID"]["mesh"]]["primitives"]
    data = bytearray(bin_chunk)
    bands_before: Counter[float] = Counter()
    bands_after: Counter[float] = Counter()
    total_hits = 0

    for pi, prim in enumerate(solid_prims):
        acc, _, base = position_accessor(doc, prim)
        count = acc["count"]
        hits = 0
        mins = [float("inf")] * 3
        maxs = [float("-inf")] * 3
        for vi in range(count):
            off = base + vi * 12
            vec = list(struct.unpack_from("<3f", data, off))
            world_y = -vec[1]
            bands_before[round(world_y, 5)] += 1
            if BAND_MIN < world_y <= BAND_MAX:
                vec[1] = -compress(world_y)
                world_y = -vec[1]
                hits += 1
                struct.pack_into("<3f", data, off, *vec)
            bands_after[round(world_y, 5)] += 1
            for c in range(3):
                mins[c] = min(mins[c], vec[c])
                maxs[c] = max(maxs[c], vec[c])
        if hits:
            acc["min"], acc["max"] = mins, maxs
        total_hits += hits
        print(f"  prim {pi}: {hits}/{count} verts compressed into 1.0..1.4mm")

    def band_table(bands: Counter[float]) -> str:
        return ", ".join(
            f"{k * 1000:.2f}mm x{bands[k]}" for k in sorted(bands) if -0.06 < k <= 0.06
        )

    print(f"world-height bands (|y| <= 60mm) before: {band_table(bands_before)}")
    print(f"world-height bands (|y| <= 60mm) after:  {band_table(bands_after)}")

    if total_hits == 0:
        print("already flat: nothing to do")
        return

    write_glb(path, doc, bytes(data))
    print(
        f"wrote {path} ({total_hits} vertices compressed to "
        f"{LOW_TARGET * 1000:.3f}..{HIGH_TARGET * 1000:.3f}mm, bin length unchanged)"
    )


if __name__ == "__main__":
    main()
