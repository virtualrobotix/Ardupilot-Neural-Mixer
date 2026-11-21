"""Read the Q8bot STEP assembly with OCP: one binary STL per unique part (part frame, mm) and a JSON with every
leaf instance (part, 4x4 world transform, colour)."""
import json, sys, struct
from pathlib import Path
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.Quantity import Quantity_Color
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

src, out = sys.argv[1], Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
rd = STEPCAFControl_Reader()
rd.SetNameMode(True); rd.SetColorMode(True)
assert rd.ReadFile(src) == 1
rd.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def name(lab):
    n = TDataStd_Name()
    return n.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), n) else "?"


def mat(loc):
    t = loc.Transformation()
    M = np.eye(4)
    for i in range(3):
        for j in range(4):
            M[i, j] = t.Value(i + 1, j + 1)
    return M


def colour(lab, shape):
    c = Quantity_Color()
    for typ in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if ct.GetColor_s(lab, typ, c):
            return [c.Red(), c.Green(), c.Blue()]
    # colour of the first coloured face / solid sub-shape
    subs = TDF_LabelSequence()
    st.GetSubShapes_s(lab, subs)
    for i in range(1, subs.Length() + 1):
        for typ in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if ct.GetColor_s(subs.Value(i), typ, c):
                return [c.Red(), c.Green(), c.Blue()]
    return None


def write_stl(shape, path, defl=0.08):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.35, True)
    tris = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            T = mat(loc)
            pts = np.array([[tri.Node(i).X(), tri.Node(i).Y(), tri.Node(i).Z(), 1.0] for i in range(1, tri.NbNodes() + 1)])
            pts = (pts @ T.T)[:, :3]
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if f.Orientation() == TopAbs_REVERSED:
                    a, b = b, a
                tris.append((pts[a - 1], pts[b - 1], pts[c - 1]))
        ex.Next()
    tris = np.array(tris, dtype=np.float32)
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0]); n = n / (np.linalg.norm(n) or 1)
            fh.write(struct.pack("<12fH", *n, *t.reshape(-1), 0))
    return len(tris)


parts, instances = {}, []


def walk(lab, T, path):
    ref = TDF_Label()
    loc = st.GetLocation_s(lab)
    T = T @ mat(loc)
    target = lab
    if st.IsReference_s(lab):
        st.GetReferredShape_s(lab, ref)
        target = ref
    nm = name(target)
    if st.IsAssembly_s(target):
        comps = TDF_LabelSequence()
        st.GetComponents_s(target, comps)
        for i in range(1, comps.Length() + 1):
            walk(comps.Value(i), T, path + "/" + nm)
        return
    key = nm
    if key not in parts:
        shape = st.GetShape_s(target)
        g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g)
        bb = Bnd_Box(); BRepBndLib.Add_s(shape, bb)
        fn = "".join(ch if ch.isalnum() else "_" for ch in nm) + ".stl"
        n = write_stl(shape, out / fn)
        c = g.CentreOfMass()
        parts[key] = {"file": fn, "tris": n, "volume_mm3": g.Mass(), "com": [c.X(), c.Y(), c.Z()],
                      "bbox": [bb.CornerMin().X(), bb.CornerMin().Y(), bb.CornerMin().Z(), bb.CornerMax().X(), bb.CornerMax().Y(), bb.CornerMax().Z()], "colour": colour(target, shape)}
    instances.append({"part": key, "path": path, "T": T.tolist(), "colour": colour(lab, None)})


free = TDF_LabelSequence()
st.GetFreeShapes(free)
for i in range(1, free.Length() + 1):
    walk(free.Value(i), np.eye(4), "")
json.dump({"parts": parts, "instances": instances}, open(out / "assembly.json", "w"), indent=1)
for k, p in parts.items():
    print(f"{k:42s} tris {p['tris']:7d} vol {p['volume_mm3'] / 1000:7.3f} cm3 bbox {np.round(p['bbox'], 1).tolist()} col {p['colour']}")
for ins in instances:
    T = np.array(ins["T"])
    print(f"{ins['part']:42s} pos {np.round(T[:3, 3], 2)} x {np.round(T[:3, 0], 2)} z {np.round(T[:3, 2], 2)}  {ins['path']}")
