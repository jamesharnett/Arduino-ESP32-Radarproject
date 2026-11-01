# Enclosures

## v1 parts (original author, print-ready)

Binary STL exports from Autodesk Fusion; bounding boxes measured from the files.
They fit the **Level 1** radar node and the GIGA display.

| File | Bounding box (mm) | Holds |
|---|---|---|
| `body display.stl` | 34 × 105 × 113 | GIGA R1 + Display Shield + 1160100 LiPo |
| `top display.stl` | 8 × 85 × 111 | display lid |
| `top radarmodule.stl` | 47 × 31 × 87 | XIAO + RD-03D + 602560 LiPo |
| `bottom radarmodule.stl` | 47 × 6 × 67 | radar node base plate |

Triangle counts are low (208–1088), so curved features print faceted. The GIGA
board is 101.5 mm long inside a 105 mm body: check the fit in your slicer.

## Level 2 sensor node (DRAFT, unverified)

`level2_sensor_node.scad` is a parametric OpenSCAD model for the radar + LIDAR
node: XIAO ESP32-S3, RD-03D lying horizontally behind a 1.2 mm front window,
MT3608, TP4056/BW4056, a 103450 pouch cell, toggle switch and LED indicator
inside a 78 × 100 × 32 mm box, with the LD19 bolted on top of the lid where it
has a clear 360° view. `level2_sensor_node_base.stl` and `level2_sensor_node_lid.stl`
are rendered from the defaults; `level2_sensor_node_preview.png` shows the assembly.

**It was designed from datasheet dimensions, not from measured parts, and has
never been printed.** Before you commit filament:

1. Measure your modules and put the numbers into the parameters at the top of
   the `.scad` file (`rd03d`, `xiao`, `mt3608`, `tp4056`, `cell`, `switch_d`).
2. The LD19 mounting-hole pattern (`ld19_holes`, four holes on a 22 mm square) is
   a **guess**. Measure the base of your LD19, or print the lid alone first and
   drill to fit.
3. The RD-03D needs a plastic window under 2 mm and no metal within 20 mm; the
   model thins the front wall to `rd03d_window_t` over the module. Keep the cell
   and the switch away from that face.
4. A ≥ 3000 mAh cell (recommended for Level 2 runtime) is larger than the
   103450 pocket; set `cell` and raise `box_d` accordingly.
5. Tape the XIAO's flex antenna under the lid in the marked slot, not next to
   the cell, and plug it in before closing the box.

Render with:

```bash
openscad -D 'part="base"' -o level2_sensor_node_base.stl level2_sensor_node.scad
openscad -D 'part="lid"'  -o level2_sensor_node_lid.stl  level2_sensor_node.scad
```

Open the file in OpenSCAD with `part="assembly"` to see the lid, the LD19
silhouette and the RD-03D position together.
