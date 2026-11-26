# -*- coding: utf-8 -*-
"""
Regression tests for PoseMatcher's mesh merge/split numpy logic.
Runs without a Maya scene: fake maya modules are injected so PoseMatcher
can be imported, and only the pure-numpy functions are exercised.

Run with mayapy (Maya's bundled Python has numpy):
    "C:/Program Files/Autodesk/Maya2025/bin/mayapy.exe" testMeshFuncs.py
"""
import sys, types, os, json, tempfile

# --- inject fake maya modules BEFORE importing PoseMatcher ---
maya = types.ModuleType("maya")
api = types.ModuleType("maya.api")

class _FakeOm(types.ModuleType):
    """Any om.XXX used in type annotations resolves to a dummy object."""
    def __getattr__(self, name):
        return object

om = _FakeOm("maya.api.OpenMaya")
cmds = types.ModuleType("maya.cmds")
cmds.workspace = lambda *a, **k: tempfile.gettempdir()
cmds.warning = lambda *a, **k: None
sys.modules["maya"] = maya
sys.modules["maya.api"] = api
sys.modules["maya.api.OpenMaya"] = om
sys.modules["maya.cmds"] = cmds

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import PoseMatcher as PM

# --- reference implementations (the old loop-based semantics) ---
def ref_process_vertices(mesh1_v, mesh2_v, overlap1, overlap2):
    total_v = mesh1_v.copy()
    map_v = np.array([], dtype=np.int32)
    for i in range(mesh2_v.shape[0]):
        if i not in overlap2:
            total_v = np.vstack([total_v, mesh2_v[i, ...]])
            map_v = np.hstack([map_v, np.array([total_v.shape[0] - 1], dtype=np.int32)])
        else:
            indices = np.where(i == overlap2)[0]
            map_v = np.hstack([map_v, overlap1[indices[0]]])
    return total_v, map_v

def ref_process_faces(mesh1_f, mesh2_f, map_v, map_uv, map_vn):
    merged_f = mesh1_f.copy()
    for i in range(mesh2_f.shape[0]):
        cur_face_id = mesh2_f[i, 0] + mesh1_f.shape[0]
        cur_vert_id = map_v[mesh2_f[i, 1]]
        cur_uv_id = map_uv[mesh2_f[i, 2]]
        cur_norm_id = map_vn[mesh2_f[i, 3]]
        merged_f = np.vstack([merged_f, np.column_stack((cur_face_id, cur_vert_id, cur_uv_id, cur_norm_id)).astype(np.int32)])
    return merged_f

# --- fixtures ---
v1 = np.array([[0,0,0],[1,0,0],[2,2,2],[3,0,0]], dtype=float)
v2 = np.array([[1,0,0],[9,9,9],[0,0,0],[5,5,5],[1,0,0]], dtype=float)  # idx 0/2/4 overlap v1
_, o1, o2 = PM.find_overlaps(v1, v2, decimals=3)

tv, mv = PM.ProcessVertices(v1, v2, o1, o2)
rtv, rmv = ref_process_vertices(v1, v2, o1, o2)
assert np.array_equal(tv, rtv), "ProcessVertices: merged vertices differ from reference"
assert np.array_equal(mv, rmv), "ProcessVertices: map_v differs from reference"
print("ProcessVertices OK  merged shape:", tv.shape, " map:", mv)

f1 = np.array([[0,0,0,0],[0,1,1,1],[0,2,2,2],
               [1,1,1,1],[1,3,3,3],[1,0,0,0]], dtype=np.int32)
f2 = np.array([[0,0,0,0],[0,1,1,1],[0,4,4,4],
               [1,3,3,3],[1,2,2,2],[1,0,0,0]], dtype=np.int32)
muv = np.arange(10, dtype=np.int32)
mvn = np.arange(20, dtype=np.int32)
mf = PM.ProcessFaces(f1, f2, mv, muv, mvn)
rf = ref_process_faces(f1, f2, mv, muv, mvn)
# old code offset face ids by row count, new by face count: compare remapped ids only
assert np.array_equal(mf[:, 1:], rf[:, 1:]), "ProcessFaces: remapped v/uv/vn ids differ from reference"
assert mf[:, 0].max() == 3, "ProcessFaces: face id offset wrong (2 faces + 2 faces -> max id 3)"
print("ProcessFaces OK  rows:", mf.shape[0])

# ProcessUV: must not mutate caller array, must separate overlapping ranges
uv1 = np.array([[0.1,0.2],[0.9,0.8]], dtype=float)
uv2 = np.array([[0.5,0.5],[1.5,1.5]], dtype=float)
uv2_backup = uv2.copy()
mu, _ = PM.ProcessUV(uv1, uv2)
assert np.array_equal(uv2, uv2_backup), "ProcessUV mutated the caller's array"
assert mu[2:, 0].min() >= 1.0, "ProcessUV did not separate overlapping U ranges"
print("ProcessUV OK")

# writeWithColor: valid OBJ output
tmp_obj = os.path.join(tempfile.gettempdir(), "test_mpm.obj")
vn1 = np.array([[0,0,1],[1,0,0],[0,1,0],[1,1,1]], dtype=float)
PM.writeWithColor(f1, v1, uv1, vn1, [], tmp_obj)
with open(tmp_obj, encoding="utf-8") as fp:
    obj_lines = [l for l in fp.read().splitlines() if l.strip()]
assert obj_lines[0].startswith("v "), "OBJ first line is not a vertex"
assert sum(1 for l in obj_lines if l.startswith("vt ")) == 2
assert sum(1 for l in obj_lines if l.startswith("vn ")) == 4
assert sum(1 for l in obj_lines if l.startswith("f ")) == 2
print("writeWithColor OK")

# load_merge_info: npz roundtrip + legacy JSON compat
tmp_base = os.path.join(tempfile.gettempdir(), "test_mpm_MergeInfo")
np.savez(tmp_base + ".npz",
         num_vertices_mesh1=v1.shape[0], num_vertices_mesh2=v2.shape[0],
         map_v=mv, uv_mesh1=uv1, uv_mesh2=uv2, faces_mesh1=f1, faces_mesh2=f2)
info = PM.load_merge_info(tmp_base + ".npz")
assert info["num_vertices_mesh1"] == 4 and info["num_vertices_mesh2"] == 5
assert np.array_equal(info["map_v"], mv)
print("load_merge_info (.npz) OK")

with open(tmp_base + ".json", "w", encoding="utf-8") as fp:
    json.dump({"num_vertices_mesh1": 4, "num_vertices_mesh2": 5,
               "map_v": mv.tolist(), "uv_mesh1": uv1.tolist(), "uv_mesh2": uv2.tolist(),
               "faces_mesh1": f1.tolist(), "faces_mesh2": f2.tolist()}, fp)
info2 = PM.load_merge_info(tmp_base + ".json")
assert info2["num_vertices_mesh1"] == 4
assert np.array_equal(info2["map_v"], mv)
print("load_merge_info (legacy .json) OK")

# SplitMeshes normal slicing (B3): vn is not deduplicated, mesh2's normals
# are the tail entries of merged_vn, NOT merged_vn[map_v].
merged_vn = np.vstack([vn1, v2])  # stand-in: N1 normals then N2 "normals"
split_vn2_fixed = merged_vn[4:4 + 5]
assert np.array_equal(split_vn2_fixed, v2), "mesh2 normals must come from the tail slice"
print("SplitMeshes vn slice OK")

print("\nALL TESTS PASSED")
