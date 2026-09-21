"""
get_shelf_bbox.py

Loads each shelf's actual collision mesh (.DAE) and computes its real
axis-aligned bounding box in meters. Raw DAE vertex data is in
centimeters (per <unit meter="0.010000">), so we scale by 0.01 to get
true meters -- confirmed via grep on the file's own <asset><unit> tag.
"""

import numpy as np
from collada import Collada

UNIT_SCALE = 0.01  # centimeters -> meters, per this file set's <unit> tag

MODELS = {
    "ShelfD_01": "/home/eesha/amr_ws/src/bcr_bot/models/aws_robomaker_warehouse_ShelfD_01/meshes/aws_robomaker_warehouse_ShelfD_01_collision.DAE",
    "ShelfE_01": "/home/eesha/amr_ws/src/bcr_bot/models/aws_robomaker_warehouse_ShelfE_01/meshes/aws_robomaker_warehouse_ShelfE_01_collision.DAE",
    "ShelfF_01": "/home/eesha/amr_ws/src/bcr_bot/models/aws_robomaker_warehouse_ShelfF_01/meshes/aws_robomaker_warehouse_ShelfF_01_collision.DAE",
}

for name, path in MODELS.items():
    mesh = Collada(path)
    all_verts = []
    for geom in mesh.geometries:
        for prim in geom.primitives:
            all_verts.append(prim.vertex)
    verts = np.vstack(all_verts) * UNIT_SCALE
    mins = verts.min(axis=0)
    maxs = verts.max(axis=0)
    size = maxs - mins
    print(f"{name}:")
    print(f"  min: x={mins[0]:.3f} y={mins[1]:.3f} z={mins[2]:.3f}")
    print(f"  max: x={maxs[0]:.3f} y={maxs[1]:.3f} z={maxs[2]:.3f}")
    print(f"  SIZE (width x depth x height): {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} meters")
    print()
