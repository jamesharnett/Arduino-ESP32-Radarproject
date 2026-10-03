// level2_sensor_node.scad — RadarLink Level 2 sensor node enclosure (DRAFT, unverified)
//
// Holds: Seeed XIAO ESP32-S3, Ai-Thinker RD-03D (horizontal, behind a thin front
// window), MT3608 boost module, TP4056/BW4056 USB-C charger, a pouch cell, a
// toggle switch and a 1S LED indicator, with the LDROBOT LD19 LIDAR mounted on
// top of the lid where it has a clear 360-degree view.
//
// THIS IS A DRAFT DESIGNED FROM DATASHEET NUMBERS, NOT FROM MEASURED PARTS.
// Before printing in earnest: measure every module you actually own and put the
// numbers into the parameters below, print only the lid first to check the LD19
// hole pattern (ld19_holes is a guess), and check the RD-03D window thickness
// (keep it under 2 mm, no metal within 20 mm of the module's face).
//
// Render:  openscad -D 'part="base"' -o level2_base.stl level2_sensor_node.scad
//          openscad -D 'part="lid"'  -o level2_lid.stl  level2_sensor_node.scad
// Preview: part="assembly"
//
// Axes: x = width (left/right), y = depth (front at -y), z = up. The RD-03D
// looks towards -y; mount the LD19 so its connector points to +y, then set the
// receivers' LIDAR_ANGLE_OFFSET_DEG from the calibration in docs/TESTING.md.
//
// SPDX-License-Identifier: MIT

part = "assembly";          // "base" | "lid" | "assembly"

/* ------------------------------------------------------------ parameters */
wall      = 2.0;            // side wall thickness
floor_t   = 2.0;
lid_t     = 2.5;
corner_r  = 3.0;
clear     = 0.4;            // clearance added around parts

// components (mm) — MEASURE YOURS
rd03d     = [44.0, 2.0, 15.0];  // PCB width, thickness (with antenna face), height; mounted horizontally
rd03d_window_t = 1.2;           // front window thickness in front of the radar
xiao      = [21.0, 17.8, 4.0];  // PCB x, y, height incl. components (USB-C adds 1.3 mm on one side)
mt3608    = [36.0, 17.0, 12.0]; // boost module incl. trimmer
tp4056    = [28.0, 17.0, 6.0];  // USB-C charger module
cell      = [34.0, 50.0, 10.5]; // pouch cell; 103450 = 2000 mAh. 3000 mAh (e.g. 606090: 60x90x6) needs box_d >= 110
switch_d  = 6.2;                // toggle switch hole
indicator = [9.5, 5.0];         // 1S 4-LED indicator window (w, h)
ld19_dia  = 38.6;               // LD19 body diameter
ld19_h    = 34.8;               // LD19 height above its base
ld19_holes = [[-11.0, -11.0], [11.0, -11.0], [-11.0, 11.0], [11.0, 11.0]];  // GUESS: measure the LD19 base
ld19_hole_d = 2.7;              // M2.5 clearance
ld19_cable_d = 8.0;

// layout
box_w = 74;                     // internal width
box_d = 96;                     // internal depth
box_h = 30;                     // internal height (components lie flat)
insert_d = 4.1;                 // heat-set insert hole for M3
insert_h = 6.0;
post     = 7.0;                 // corner post size

$fn = 48;

/* --------------------------------------------------------------- helpers */
module rrect(w, d, h, r) {
    hull() for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * (w / 2 - r), sy * (d / 2 - r), 0]) cylinder(r = r, h = h);
}

module pocket(size, z = floor_t, rail = 1.6, rail_h = 3.0) {
    // four short rails that locate a PCB of `size` on the floor without screws
    s = [size[0] + 2 * clear, size[1] + 2 * clear];
    for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * (s[0] / 2 + rail / 2), sy * (s[1] / 2 - 4), z])
            cube([rail, 8, rail_h], center = false);
}

ow = box_w + 2 * wall;
od = box_d + 2 * wall;
oh = box_h + floor_t;

/* ------------------------------------------------------------------ base */
module base() {
    difference() {
        union() {
            rrect(ow, od, oh, corner_r);
        }
        // cavity
        translate([0, 0, floor_t]) rrect(box_w, box_d, box_h + 1, corner_r - wall > 0.5 ? corner_r - wall : 0.5);
        // RD-03D window: thin the front wall to rd03d_window_t over the module face
        translate([-(rd03d[0] + 6) / 2, -od / 2 - 0.01, floor_t + 6])
            cube([rd03d[0] + 6, wall - rd03d_window_t + 0.01, rd03d[2] + 6]);
        // TP4056 USB-C on the right wall, near the back
        translate([ow / 2 - wall - 0.01, box_d / 2 - 12 - 10, floor_t + 2]) cube([wall + 0.02, 10, 4]);
        // XIAO USB-C on the right wall, near the front (programming)
        translate([ow / 2 - wall - 0.01, -box_d / 2 + 14, floor_t + 2]) cube([wall + 0.02, 10, 4]);
        // toggle switch on the left wall
        translate([-ow / 2 - 0.01, box_d / 2 - 20, floor_t + box_h / 2]) rotate([0, 90, 0]) cylinder(d = switch_d, h = wall + 0.02);
        // indicator window on the left wall
        translate([-ow / 2 - 0.01, 0, floor_t + box_h / 2 - indicator[1] / 2]) cube([wall + 0.02, indicator[0], indicator[1]]);
        // vents on the back wall
        for (i = [-2 : 2]) translate([i * 8 - 1, od / 2 - wall - 0.01, floor_t + 6]) cube([2, wall + 0.02, box_h - 12]);
    }
    // corner posts with heat-set insert holes
    for (sx = [-1, 1], sy = [-1, 1]) difference() {
        translate([sx * (box_w / 2 - post / 2), sy * (box_d / 2 - post / 2), floor_t]) cube([post, post, box_h], center = false);
        translate([sx * (box_w / 2 - post / 2) + post / 2, sy * (box_d / 2 - post / 2) + post / 2, floor_t + box_h - insert_h])
            cylinder(d = insert_d, h = insert_h + 0.01);
    }
    // RD-03D cradle: two slots the module slides into, face against the window
    for (sx = [-1, 1]) translate([sx * (rd03d[0] / 2 + 1.0) - 1.0, -box_d / 2, floor_t])
        difference() {
            cube([2.0, rd03d[1] + 2 * clear + 2, rd03d[2] + 8]);
            translate([-0.01, 1.0, 2]) cube([2.02, rd03d[1] + 2 * clear, rd03d[2] + 7]);
        }
    // component pockets (flat on the floor)
    translate([-box_w / 2 + cell[0] / 2 + 2, box_d / 2 - cell[1] / 2 - 2, 0]) pocket(cell);
    translate([box_w / 2 - mt3608[1] / 2 - 2, box_d / 2 - mt3608[0] / 2 - 2, 0]) rotate([0, 0, 90]) pocket(mt3608);
    translate([box_w / 2 - tp4056[1] / 2 - 2, 6, 0]) rotate([0, 0, 90]) pocket(tp4056);
    translate([box_w / 2 - xiao[0] / 2 - 2, -box_d / 2 + xiao[1] / 2 + 10, 0]) pocket(xiao);
}

/* ------------------------------------------------------------------- lid */
module lid() {
    difference() {
        union() {
            rrect(ow, od, lid_t, corner_r);
            // inner lip
            translate([0, 0, -2]) difference() {
                rrect(box_w - 0.6, box_d - 0.6, 2.01, corner_r - wall > 0.5 ? corner_r - wall : 0.5);
                translate([0, 0, -0.01]) rrect(box_w - 0.6 - 2 * 1.6, box_d - 0.6 - 2 * 1.6, 2.03, 1);
            }
            // LD19 seat ring
            translate([0, 10, lid_t - 0.01]) difference() {
                cylinder(d = ld19_dia + 6, h = 2);
                translate([0, 0, -0.01]) cylinder(d = ld19_dia + 2 * clear, h = 2.02);
            }
        }
        // lid screws into the corner posts
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx * (box_w / 2 - post / 2), sy * (box_d / 2 - post / 2), -3]) cylinder(d = 3.3, h = lid_t + 6);
        // LD19 mounting holes and cable pass-through
        for (h = ld19_holes) translate([h[0], 10 + h[1], -3]) cylinder(d = ld19_hole_d, h = lid_t + 6);
        translate([0, 10 + 8, -3]) cylinder(d = ld19_cable_d, h = lid_t + 6);
        // antenna slot: the XIAO's flex antenna is taped under the lid, front-right, away from the cell
        translate([box_w / 2 - 14, -box_d / 2 + 4, -0.01]) cube([12, 30, 0.6]);
    }
}

module ld19_ghost() {
    color("silver", 0.4) translate([0, 10, oh + lid_t + 2]) cylinder(d = ld19_dia, h = ld19_h);
}

/* --------------------------------------------------------------- output */
if (part == "base") base();
else if (part == "lid") translate([0, 0, 0]) lid();
else {
    base();
    translate([0, 0, oh + 0.01]) lid();
    ld19_ghost();
    color("green", 0.5) translate([-rd03d[0] / 2, -box_d / 2 + 1.5, floor_t + 2]) cube(rd03d);
}
