#!/usr/bin/env python3
"""Strip stray nodes and their exclusive resources from a GLB asset.

vehicle.glb was exported with two stray laser-detector assemblies
(激光检测总装.001 / .002) parented under VEHICLE->GIMBAL, so every vehicle
spawned from the GLB carried them as floating "tech core" models. The
Blender source is not tracked in this repo, so the GLB itself is the only
place to fix it. This tool performs the surgery at the glTF level:

  * removes the named node subtrees and remaps every node-index reference
    (scenes[].nodes, nodes[].children, skins, animation channel targets)
  * cascade-prunes meshes / materials / textures / images / samplers /
    accessors / bufferViews that lose their last referencer (resources
    shared with surviving nodes are kept)
  * leaves the BIN chunk byte-identical (dead bytes stay, only the JSON
    chunk is rebuilt and the GLB length recomputed)
  * validates the result (index ranges, dangling references, forbidden
    names) before anything is written

Example:
    python tools/strip_vehicle_stray_nodes.py assets/vehicle.glb \
        --names 激光检测总装.001 激光检测总装.002

Git history is the rollback net for the binary asset.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

GLB_MAGIC = 0x46546C67  # 'glTF'

TRACKED = [
    "nodes",
    "meshes",
    "materials",
    "textures",
    "images",
    "samplers",
    "accessors",
    "bufferViews",
    "animations",
    "skins",
]


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


# ------------------------------------------------------------ tree helpers

def dump_tree(doc: dict) -> None:
    nodes = doc.get("nodes", [])
    parents = {}
    for i, n in enumerate(nodes):
        for c in n.get("children", []):
            parents[c] = i

    def walk(i: int, depth: int) -> None:
        n = nodes[i]
        extra = f" [mesh:{n['mesh']}]" if "mesh" in n else ""
        print("  " * depth + n.get("name", f"<node {i}>") + extra)
        for c in n.get("children", []):
            walk(c, depth + 1)

    for i in range(len(nodes)):
        if i not in parents:
            walk(i, 0)


def is_texture_info(obj, parent_key: str) -> bool:
    """A glTF texture-info object: {"index": n, "texCoord": ...} under a *Texture key."""
    return (
        isinstance(obj, dict)
        and isinstance(obj.get("index"), int)
        and (parent_key.endswith("Texture") or "texCoord" in obj)
    )


def walk_texture_infos(obj, parent_key: str, fn) -> None:
    if is_texture_info(obj, parent_key):
        fn(obj)
        return
    if isinstance(obj, dict):
        for key, value in obj.items():
            walk_texture_infos(value, key, fn)
    elif isinstance(obj, list):
        for value in obj:
            walk_texture_infos(value, parent_key, fn)


# ------------------------------------------------------------------ surgery

def strip(doc: dict, names: list[str]) -> None:
    nodes = doc.get("nodes", [])
    targets = [i for i, n in enumerate(nodes) if n.get("name") in names]
    if not targets:
        sys.exit(f"error: none of {names} matched any node name")
    missing = set(names) - {nodes[i]["name"] for i in targets}
    if missing:
        print(f"warning: names not found: {sorted(missing)}")

    removed = set()
    stack = list(targets)
    while stack:
        i = stack.pop()
        if i in removed:
            continue
        removed.add(i)
        stack.extend(nodes[i].get("children", []))

    parents = {}
    for i, n in enumerate(nodes):
        for c in n.get("children", []):
            parents.setdefault(c, i)
    print(f"removing {len(removed)} node(s):")
    for t in sorted(targets):
        p = parents.get(t)
        pname = nodes[p].get("name", f"<node {p}>") if p is not None else "<scene root>"
        print(f"  {nodes[t].get('name')!r} (mesh {nodes[t].get('mesh')}, parent {pname})")

    # Nodes: drop removed, remap every node-index reference.
    node_map = {}
    keep_nodes = []
    for i, n in enumerate(nodes):
        if i not in removed:
            node_map[i] = len(keep_nodes)
            keep_nodes.append(n)
    for n in keep_nodes:
        if "children" in n:
            n["children"] = [node_map[c] for c in n["children"] if c not in removed]
    for scene in doc.get("scenes", []):
        if "nodes" in scene:
            scene["nodes"] = [node_map[r] for r in scene["nodes"] if r not in removed]

    # Animations: drop channels that targeted removed nodes, drop empty animations.
    keep_anims = []
    for anim in doc.get("animations", []):
        channels = []
        for ch in anim.get("channels", []):
            target = ch.get("target", {})
            node_idx = target.get("node")
            if node_idx is not None and node_idx in removed:
                continue
            if node_idx is not None:
                target["node"] = node_map[node_idx]
            channels.append(ch)
        if channels:
            anim["channels"] = channels
            keep_anims.append(anim)

    # Skins: remap joints/skeleton; drop skins (and their node references) that die.
    skin_map = {}
    keep_skins = []
    for i, skin in enumerate(doc.get("skins", [])):
        joints = [node_map[j] for j in skin.get("joints", []) if j not in removed]
        skeleton = skin.get("skeleton")
        if not joints or (skeleton is not None and skeleton in removed):
            continue
        if skeleton is not None:
            skin["skeleton"] = node_map[skeleton]
        skin["joints"] = joints
        skin_map[i] = len(keep_skins)
        keep_skins.append(skin)
    for n in keep_nodes:
        if "skin" in n:
            if n["skin"] in skin_map:
                n["skin"] = skin_map[n["skin"]]
            else:
                del n["skin"]

    # Meshes: keep those referenced by surviving nodes.
    used_meshes = sorted({n["mesh"] for n in keep_nodes if "mesh" in n})
    mesh_map = {old: new for new, old in enumerate(used_meshes)}
    keep_meshes = [doc.get("meshes", [])[old] for old in used_meshes]
    for n in keep_nodes:
        if "mesh" in n:
            n["mesh"] = mesh_map[n["mesh"]]

    prims = [p for m in keep_meshes for p in m.get("primitives", [])]

    # Materials: keep those referenced by surviving primitives.
    used_materials = sorted({p["material"] for p in prims if "material" in p})
    material_map = {old: new for new, old in enumerate(used_materials)}
    keep_materials = [doc.get("materials", [])[old] for old in used_materials]
    for p in prims:
        if "material" in p:
            p["material"] = material_map[p["material"]]

    # Accessors: keep those referenced by primitives, skins, animation samplers.
    used_accessors = set()
    for p in prims:
        if "indices" in p:
            used_accessors.add(p["indices"])
        used_accessors.update(p.get("attributes", {}).values())
        for target in p.get("targets", []):
            used_accessors.update(target.values())
    for skin in keep_skins:
        if "inverseBindMatrices" in skin:
            used_accessors.add(skin["inverseBindMatrices"])
    for anim in keep_anims:
        for sampler in anim.get("samplers", []):
            used_accessors.update((sampler["input"], sampler["output"]))
    used_accessors = sorted(used_accessors)
    accessor_map = {old: new for new, old in enumerate(used_accessors)}
    keep_accessors = [doc.get("accessors", [])[old] for old in used_accessors]
    for p in prims:
        if "indices" in p:
            p["indices"] = accessor_map[p["indices"]]
        if "attributes" in p:
            p["attributes"] = {k: accessor_map[v] for k, v in p["attributes"].items()}
        if "targets" in p:
            p["targets"] = [{k: accessor_map[v] for k, v in t.items()} for t in p["targets"]]
    for skin in keep_skins:
        if "inverseBindMatrices" in skin:
            skin["inverseBindMatrices"] = accessor_map[skin["inverseBindMatrices"]]
    for anim in keep_anims:
        for sampler in anim.get("samplers", []):
            sampler["input"] = accessor_map[sampler["input"]]
            sampler["output"] = accessor_map[sampler["output"]]

    # Textures: keep those referenced by surviving materials.
    used_textures = set()
    for material in keep_materials:
        walk_texture_infos(material, "", lambda ti: used_textures.add(ti["index"]))
    used_textures = sorted(used_textures)
    texture_map = {old: new for new, old in enumerate(used_textures)}
    keep_textures = [doc.get("textures", [])[old] for old in used_textures]
    for material in keep_materials:
        walk_texture_infos(
            material, "", lambda ti: ti.__setitem__("index", texture_map[ti["index"]])
        )

    # Images / samplers: keep those referenced by surviving textures.
    used_images = sorted({t["source"] for t in keep_textures if "source" in t})
    used_samplers = sorted({t["sampler"] for t in keep_textures if "sampler" in t})
    image_map = {old: new for new, old in enumerate(used_images)}
    sampler_map = {old: new for new, old in enumerate(used_samplers)}
    keep_images = [doc.get("images", [])[old] for old in used_images]
    keep_samplers = [doc.get("samplers", [])[old] for old in used_samplers]
    for t in keep_textures:
        if "source" in t:
            t["source"] = image_map[t["source"]]
        if "sampler" in t:
            t["sampler"] = sampler_map[t["sampler"]]

    # Buffer views: keep those referenced by surviving accessors and images.
    used_views = set()
    for accessor in keep_accessors:
        if "bufferView" in accessor:
            used_views.add(accessor["bufferView"])
    for image in keep_images:
        if "bufferView" in image:
            used_views.add(image["bufferView"])
    used_views = sorted(used_views)
    view_map = {old: new for new, old in enumerate(used_views)}
    keep_views = [doc.get("bufferViews", [])[old] for old in used_views]
    for accessor in keep_accessors:
        if "bufferView" in accessor:
            accessor["bufferView"] = view_map[accessor["bufferView"]]
    for image in keep_images:
        if "bufferView" in image:
            image["bufferView"] = view_map[image["bufferView"]]

    doc["nodes"] = keep_nodes
    if "meshes" in doc:
        doc["meshes"] = keep_meshes
    if "materials" in doc:
        doc["materials"] = keep_materials
    if "accessors" in doc:
        doc["accessors"] = keep_accessors
    if "textures" in doc:
        doc["textures"] = keep_textures
    if "images" in doc:
        doc["images"] = keep_images
    if "samplers" in doc:
        doc["samplers"] = keep_samplers
    if "bufferViews" in doc:
        doc["bufferViews"] = keep_views
    if "animations" in doc:
        doc["animations"] = keep_anims
    if "skins" in doc:
        doc["skins"] = keep_skins


# --------------------------------------------------------------- validation

def validate(doc: dict, forbidden: set[str]) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    nodes = doc.get("nodes", [])
    n = {
        "node": len(nodes),
        "mesh": len(doc.get("meshes", [])),
        "material": len(doc.get("materials", [])),
        "accessor": len(doc.get("accessors", [])),
        "texture": len(doc.get("textures", [])),
        "image": len(doc.get("images", [])),
        "sampler": len(doc.get("samplers", [])),
        "bufferView": len(doc.get("bufferViews", [])),
        "skin": len(doc.get("skins", [])),
    }

    def check(idx, kind, what):
        if not 0 <= idx < n[kind]:
            errors.append(f"{what}: {kind} index {idx} out of range (n={n[kind]})")

    for i, node in enumerate(nodes):
        if node.get("name") in forbidden:
            errors.append(f"node {i} still named {node['name']!r}")
        for c in node.get("children", []):
            check(c, "node", f"node {i} children")
            if c == i:
                errors.append(f"node {i} parents itself")
        if "mesh" in node:
            check(node["mesh"], "mesh", f"node {i}")
        if "skin" in node:
            check(node["skin"], "skin", f"node {i}")
    for scene in doc.get("scenes", []):
        for r in scene.get("nodes", []):
            check(r, "node", "scene nodes")
    for m in doc.get("meshes", []):
        for p in m.get("primitives", []):
            if "material" in p:
                check(p["material"], "material", "primitive")
            if "indices" in p:
                check(p["indices"], "accessor", "primitive indices")
            for v in p.get("attributes", {}).values():
                check(v, "accessor", "primitive attributes")
            for target in p.get("targets", []):
                for v in target.values():
                    check(v, "accessor", "primitive targets")
    for skin in doc.get("skins", []):
        for j in skin.get("joints", []):
            check(j, "node", "skin joints")
        if "skeleton" in skin:
            check(skin["skeleton"], "node", "skin skeleton")
        if "inverseBindMatrices" in skin:
            check(skin["inverseBindMatrices"], "accessor", "skin inverseBindMatrices")
    for anim in doc.get("animations", []):
        samplers = anim.get("samplers", [])
        for ch in anim.get("channels", []):
            if not 0 <= ch.get("sampler", -1) < len(samplers):
                errors.append("animation channel sampler index out of range")
            if "node" in ch.get("target", {}):
                check(ch["target"]["node"], "node", "animation target")
        for sampler in samplers:
            check(sampler["input"], "accessor", "animation sampler input")
            check(sampler["output"], "accessor", "animation sampler output")
    for material in doc.get("materials", []):
        refs = []
        walk_texture_infos(material, "", lambda ti: refs.append(ti["index"]))
        for t in refs:
            check(t, "texture", "material texture")
    for texture in doc.get("textures", []):
        if "source" in texture:
            check(texture["source"], "image", "texture source")
        if "sampler" in texture:
            check(texture["sampler"], "sampler", "texture sampler")
    for accessor in doc.get("accessors", []):
        if "bufferView" in accessor:
            check(accessor["bufferView"], "bufferView", "accessor")
    for image in doc.get("images", []):
        if "bufferView" in image:
            check(image["bufferView"], "bufferView", "image")

    views = doc.get("bufferViews", [])
    if views:
        end = max(
            v.get("byteOffset", 0) + v.get("byteLength", 0)
            for v in views
        )
        buffers = doc.get("buffers", [])
        if buffers and "uri" not in buffers[0] and buffers[0].get("byteLength", 0) < end:
            errors.append(
                f"buffer byteLength {buffers[0]['byteLength']} < bufferView end {end}"
            )

    # Warn (not fail) about nodes unreachable from any scene.
    reachable = set()
    stack = [r for s in doc.get("scenes", []) for r in s.get("nodes", [])]
    while stack:
        i = stack.pop()
        if i in reachable:
            continue
        reachable.add(i)
        stack.extend(nodes[i].get("children", []))
    orphans = [i for i in range(len(nodes)) if i not in reachable]
    if orphans:
        names = [nodes[i].get("name", f"<node {i}>") for i in orphans]
        warnings.append(f"nodes unreachable from any scene (pre-existing?): {names}")

    return errors, warnings


# --------------------------------------------------------------------- main

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Remove named node subtrees (and orphaned resources) from a GLB."
    )
    parser.add_argument("glb", type=Path, help="GLB file to fix (modified in place)")
    parser.add_argument(
        "--names",
        nargs="+",
        required=True,
        help="exact node names to remove, subtrees included",
    )
    parser.add_argument("--out", type=Path, help="write the result here instead")
    parser.add_argument("--dry-run", action="store_true", help="validate only")
    parser.add_argument("--tree", action="store_true", help="print the node tree")
    args = parser.parse_args()

    doc, bin_chunk = read_glb(args.glb)
    before = {k: len(doc.get(k, [])) for k in TRACKED}

    strip(doc, args.names)
    errors, warnings = validate(doc, set(args.names))
    for w in warnings:
        print("warn:", w)
    if errors:
        for e in errors:
            print("INVALID:", e)
        sys.exit("refusing to write an invalid file")

    after = {k: len(doc.get(k, [])) for k in TRACKED}
    summary = "  ".join(
        f"{k} {before[k]}->{after[k]}" for k in TRACKED if before[k] or after[k]
    )
    print(summary)

    if args.tree:
        dump_tree(doc)

    if args.dry_run:
        print("dry run: nothing written")
        return
    out_path = args.out or args.glb
    write_glb(out_path, doc, bin_chunk)
    print(f"wrote {out_path} (bin chunk unchanged, {len(bin_chunk)} bytes)")


if __name__ == "__main__":
    main()
