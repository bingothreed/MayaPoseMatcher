# -*- coding: utf-8 -*-
"""
Tests for PoseMatcher's BVH export logic (no Maya scene needed).
Stubs maya.cmds with a tiny fake skeleton; uses the REAL maya.api.OpenMaya
from mayapy for rotation conversion.

Run:
    "C:/Program Files/Autodesk/Maya2025/bin/mayapy.exe" testBVHExport.py [reference.bvh ...]

With reference files as arguments, their HIERARCHY sections are parsed and
joint counts printed (used to validate the parser on real DAZ exports).
"""
import sys, types, os, math, tempfile

# --- stub only maya.cmds (keep the real maya.api.OpenMaya for matrix math) ---
cmds = types.ModuleType("maya.cmds")
FAKE = {
    "hip":    {"translate": (0.0, 0.0, 0.0),    "rotate": (0.0, 0.0, 0.0),  "rotateOrder": 0, "jointOrient": (0.0, 0.0, 0.0), "world": (0.0, 0.0, 0.0),   "children": ["pelvis"]},
    "pelvis": {"translate": (0.0, 1.85, -0.77), "rotate": (10.0, 0.0, 0.0), "rotateOrder": 0, "jointOrient": (0.0, 0.0, 0.0), "world": (0.0, 1.85, -0.77), "children": ["spine1"]},
    "spine1": {"translate": (0.0, 2.0, 0.0),    "rotate": (0.0, 0.0, 15.0), "rotateOrder": 5, "jointOrient": (0.0, 0.0, 0.0), "world": (0.0, 3.85, -0.77), "children": []},
}
def fake_getAttr(attr):
    joint, name = attr.split(".")
    val = FAKE[joint][name]
    if name == "rotateOrder":
        return val
    return [tuple(val)]
def fake_xform(joint, **kw):
    if kw.get("query") and kw.get("worldSpace") and kw.get("translation"):
        return list(FAKE[joint]["world"])
    return None
def fake_listRelatives(joint, **kw):
    if kw.get("children"):
        return FAKE[joint]["children"] or None
    if kw.get("allDescendents"):
        out = []
        def walk(j):
            for c in FAKE[j]["children"]:
                out.append(c)
                walk(c)
        walk(joint)
        return out or None
    return None
cmds.getAttr = fake_getAttr
cmds.xform = fake_xform
cmds.listRelatives = fake_listRelatives
cmds.workspace = lambda *a, **k: tempfile.gettempdir()
cmds.warning = lambda *a, **k: print("WARN:", *a)
cmds.objExists = lambda name: name in FAKE
sys.modules["maya.cmds"] = cmds

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import maya.api.OpenMaya as om2  # real module for independent rotation checks
import PoseMatcher as PM

# --- sanity check of the rotation math used by the exporter ---
e = om2.MEulerRotation(math.radians(10), 0.0, 0.0, 0)   # XYZ: rotX(10)
b = e.asQuaternion().asEulerRotation().reorderIt(2)     # reorder to ZXY
assert abs(math.degrees(b.x) - 10.0) < 1e-9, "ZXY decomposition of rotX wrong"
assert abs(math.degrees(b.y)) < 1e-9 and abs(math.degrees(b.z)) < 1e-9
print("rotation conversion sanity OK")

# --- reference template with different-case names + an extra template joint ---
REF_BVH = """HIERARCHY
ROOT Hip
{
\tOFFSET 0.000000 97.136932 0.486539
\tCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
\tJOINT Pelvis
\t{
\t\tOFFSET 0.000000 1.846634 -0.766539
\t\tCHANNELS 3 Zrotation Xrotation Yrotation
\t\tEnd Site
\t\t{
\t\t\tOFFSET 1.000000 2.000000 3.000000
\t\t}
\t}
\tJOINT Neck
\t{
\t\tOFFSET 0.000000 40.000000 0.000000
\t\tCHANNELS 3 Zrotation Xrotation Yrotation
\t\tEnd Site
\t\t{
\t\t\tOFFSET 0.000000 5.000000 0.000000
\t\t}
\t}
}
MOTION
Frames: 1
Frame Time: 0.0333333
0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000
"""

def count_nodes(node):
    n = 1
    for c in node["children"]:
        n += count_nodes(c)
    return n

def find(node, name):
    if _norm(node["name"]) == _norm(name):
        return node
    for c in node["children"]:
        r = find(c, name)
        if r:
            return r
    return None

def _norm(s):
    import re
    return re.sub(r"[^a-z0-9]", "", s.lower())

tmp = tempfile.gettempdir()
ref_path = os.path.join(tmp, "test_mpm_ref.bvh")
out_path = os.path.join(tmp, "test_mpm_out.bvh")
with open(ref_path, "w", encoding="utf-8") as fp:
    fp.write(REF_BVH)

# --- export WITH reference ---
matched = PM.export_daz_pose_bvh("hip", out_path, reference_bvh=ref_path)
assert matched == 3, f"expected 3 matched joints, got {matched}"
root = PM.parse_bvh_hierarchy(out_path)
assert root["name"] == "Hip", "template root name must be kept"
assert root["offset"][1] == 97.136932, "template root offset must be kept"
pelvis = find(root, "pelvis")
assert pelvis is not None and pelvis["end_site"] == [1.0, 2.0, 3.0], "template End Site must be kept"
spine1 = find(root, "spine1")
assert spine1 is not None, "extra Maya joint must be appended under its matched ancestor"
assert spine1["end_site"] == [0.0, 0.0, 0.0], "appended leaf needs a synthesized End Site"
neck = find(root, "neck")
assert neck is not None, "template-only joint must remain"

with open(out_path, encoding="utf-8") as fp:
    text = fp.read()
frame_line = text.split("MOTION", 1)[1].splitlines()[3]
vals = [float(v) for v in frame_line.split()]
# channels: root 6 + pelvis 3 + spine1 3 + neck 3 = 15
assert len(vals) == 15, f"expected 15 channel values, got {len(vals)}"
assert vals[0:3] == [0.0, 0.0, 0.0], "root position must come from Maya world position"
# pelvis: (10,0,0) in XYZ -> ZXY channels [Z=0, X=10, Y=0]
assert abs(vals[6] - 0.0) < 1e-6 and abs(vals[7] - 10.0) < 1e-6 and abs(vals[8] - 0.0) < 1e-6, f"pelvis rotation wrong: {vals[6:9]}"
# spine1: (0,0,15) in rotateOrder ZYX -> ZXY channels [Z=15, X=0, Y=0]
assert abs(vals[9] - 15.0) < 1e-6 and abs(vals[10]) < 1e-6 and abs(vals[11]) < 1e-6, f"spine1 rotation wrong: {vals[9:12]}"
# neck: template-only -> zeros
assert vals[12:15] == [0.0, 0.0, 0.0], f"neck rotation must be zero: {vals[12:15]}"
print("export WITH reference OK  (values:", vals, ")")

# --- export WITHOUT reference ---
out2 = os.path.join(tmp, "test_mpm_out2.bvh")
matched2 = PM.export_daz_pose_bvh("hip", out2)
assert matched2 == 3
root2 = PM.parse_bvh_hierarchy(out2)
assert root2["name"] == "hip"
assert count_nodes(root2) == 3, "no-reference export must mirror the Maya hierarchy"
with open(out2, encoding="utf-8") as fp:
    frame2 = [float(v) for v in fp.read().split("MOTION", 1)[1].splitlines()[3].split()]
assert len(frame2) == 3 + 3 * 3, f"expected 12 channels, got {len(frame2)}"
print("export WITHOUT reference OK")

# --- DUF export (DAZ pose preset) ---
import gzip as _gzip
import json as _json
duf_out = os.path.join(tmp, "test_mpm_pose.duf")
count = PM.export_daz_pose_duf("hip", duf_out)
assert count == 3
duf = _json.loads(_gzip.decompress(open(duf_out, "rb").read()).decode("utf-8"))
assert duf["file_version"] == "0.6.2.0"
assert duf["asset_info"]["type"] == "preset_pose"
anims = duf["scene"]["animations"]
assert len(anims) == 3 * 10, f"expected 30 entries, got {len(anims)}"
by_url = {a["url"]: a["keys"][0][1] for a in anims}
assert by_url["name://@selection/pelvis:?rotation/x/value"] == 10.0
assert by_url["name://@selection/pelvis:?rotation/y/value"] == 0.0
assert by_url["name://@selection/spine1:?rotation/z/value"] == 15.0  # rotateOrder ZYX passes through 1:1
assert by_url["name://@selection/pelvis:?translation/x/value"] == 0.0
assert by_url["name://@selection/pelvis:?scale/general/value"] == 1.0
assert by_url["name://@selection/hip:?scale/z/value"] == 1.0
print("DUF export OK")

# --- DUF bake_joint_orient vs independent om computation ---
old_jo = FAKE["pelvis"]["jointOrient"]
FAKE["pelvis"]["jointOrient"] = (0.0, 0.0, 15.0)
duf_out2 = os.path.join(tmp, "test_mpm_pose2.duf")
PM.export_daz_pose_duf("hip", duf_out2, bake_joint_orient=True)
FAKE["pelvis"]["jointOrient"] = old_jo
duf2 = _json.loads(_gzip.decompress(open(duf_out2, "rb").read()).decode("utf-8"))
by_url2 = {a["url"]: a["keys"][0][1] for a in duf2["scene"]["animations"]}
r = om2.MEulerRotation(math.radians(10), 0.0, 0.0, 0)
jr = om2.MEulerRotation(0.0, 0.0, math.radians(15), 0)
m = om2.MMatrix()
m.setToProduct(jr.asMatrix(), r.asMatrix())
e = om2.MTransformationMatrix(m).rotation(asQuaternion=True).asEulerRotation().reorderIt(0)
for axis, key in (("x", "rotation/x"), ("y", "rotation/y"), ("z", "rotation/z")):
    got = by_url2[f"name://@selection/pelvis:?{key}/value"]
    exp = math.degrees(getattr(e, axis))
    assert abs(got - exp) < 1e-6, f"baked {key}: {got} != {exp}"
print("DUF bake_joint_orient OK")

# --- optional: parse real DAZ reference files passed on the command line ---
for ref in sys.argv[1:]:
    if os.path.exists(ref):
        r = PM.parse_bvh_hierarchy(ref)
        print(f"parsed {os.path.basename(ref)}: {count_nodes(r)} nodes, root '{r['name']}', root channels {r['channels']}")
    else:
        print(f"SKIP (not found): {ref}")

print("\nALL BVH TESTS PASSED")
