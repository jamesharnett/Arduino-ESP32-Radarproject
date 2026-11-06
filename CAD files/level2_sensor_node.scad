// level2_sensor_node.scad — RadarLink Level 2 sensor node enclosure
// Seeed XIAO ESP32-S3 + Ai-Thinker RD-03D radar + LDROBOT LD19 LIDAR, with an
// MT3608 boost module, a TP4056 USB-C charger (DW01A/FS8205A protection), a pouch
// cell, a toggle switch and a 1S 4-LED battery indicator.
//
// PARTS — set `part` below or pass -D 'part="..."' on the command line:
//   "base"           box body: radar cradle in the front wall, floor pockets,
//                    corner posts for heat-set inserts, rebate for the lid
//   "lid"            flush lid with the LD19 three-ear mount and cable slot
//   "assembly"       base + lid + translucent component ghosts (preview only)
//   "ld19_coupon"    60 x 60 x 2.5 test plate with exactly the lid's LD19 features
//   "rd03d_coupon"   slice of the front wall (window + cradle + floor stub) to
//                    test the radar slide-in and the 1.0 mm window print
//   "ld19_template"  2D drawing of the LD19 footprint; export with
//                    openscad -D 'part="ld19_template"' -o ld19_template.svg ...
//                    and print on paper at 100 % (1:1) to lay the sensor on.
//
// AXES: x = width (+x is right when looking at the front), y = depth (the FRONT
// is -y), z = up.  The origin is the centre of the cavity footprint on the
// outer floor face.  All dimensions in mm.
//
// ORIENTATION OF THE SENSORS
//  * The RD-03D stands VERTICALLY inside the front (-y) wall, 44 mm axis up,
//    antenna face toward -y, J1 connector end DOWN.  Its +-60 deg azimuth field
//    opens across the 15 mm width (x) and the +-30 deg pitch field along the
//    44 mm length (z) [RD-UM 9.1, posture (a)].  A horizontal board, as in the
//    previous draft, would have swapped the two fields.
//  * The LD19 sits on the lid on its three ears.  Its single REAR ear (the 0 deg
//    scan direction) points to -y, the radar's direction; its connector wall
//    faces +y (back).  The LD19 frame therefore equals the box frame (+Y of the
//    datasheet drawing = +y here) and the receivers' LIDAR_ANGLE_OFFSET_DEG
//    starts near 0.  Angles increase clockwise seen from above [LD19-DS].
//  * The lid top is the LD19 mounting plane.  The optical window band is
//    23-33 mm above it: nothing on the lid may rise above ~20 mm (only the
//    1.8 mm locating bosses and the M3 screw heads do).
//
// SOURCES (cited next to each number)
//  [LD19-DS]   LDROBOT_LD19_Datasheet_V1.0.pdf, installation drawing p.7 (±0.2 mm)
//              https://github.com/ldrobotSensorTeam/ld19/blob/master/datasheet/
//  [LD19-STEP] STL-19P_3D.stp, https://github.com/WE-Autopilot/sensor-setup/
//              releases/download/Drivers/STL-19P_3D.stp, cross-checked with
//              https://github.com/Myzhar/ldrobot-lidar-ros2/blob/main/3d_model/LD19.step
//              (LD19, Waveshare D300 kit and STL-19P/D500 are mechanically the same)
//  [RD-DS]     rd-03d_v1.0.0_specification.pdf, Table 1 and Figure 4 (size chart)
//              https://github.com/Doubletop12/LD2450-and-Rd-03d_Docs/blob/main/Rd-03d/
//  [RD-UM]     rd-03d_multi-target_trajectory_tracking_user_manual.pdf, Fig. 8-1, 9.1
//  [LD2450-DG] HLK-LD2450 manual, radome rules of the same ICLEGEND 24 GHz front end:
//              antenna-to-wall gap a multiple of 6.2 mm (lambda/2 in air) or
//              <= 1.25 mm (lambda/10); wall = lambda/2 in plastic (ABS 3.92 mm) or
//              <= lambda/8 (ABS/PETG 1.0, PC 0.89 mm); a 3.1 mm gap is the worst case.
//  [XIAO-KICAD] Seeed-Studio/OPL_Kicad_Library, ESP32S3.kicad_pcb: XIAO ESP32-S3 board
//              outline 17.78 x 21.135 R1.905, USB-C and battery-pad positions (the spec
//              table rounds to 21 x 17.8)
//  [NOPSCAD]   NopSCADlib measured MT3608 module: 37 x 17 x 1.2 mm PCB, R2 corners, no
//              mounting holes, four 1.5 mm pad holes 3.07 from the short ends / 5.1 from the long edges
//  [SELLERS]   seller listings: MT3608 36-37.2 x 17-17.2 x 14 mm, TP4056 USB-C charger
//              28 x 17.3 mm (29 incl. the connector), 1S 4-LED indicator 43.5 x 20 x 8.6 mm
//  [CELL-DS]   Jauch / DNK pouch-cell datasheets and listings (medium confidence)
//  [INSERT]    CNC Kitchen and Accu charts: common M3 brass insert 4.4-4.6 mm OD takes a
//              4.0 mm pilot hole; printed holes come out ~0.25 undersize, so 4.2 is modelled
//
// VERIFIED (datasheet or STEP): every number in the LD19 and RD-03D blocks.
// RESEARCHED BUT NOT MEASURED (medium confidence, marked MEASURE): the XIAO,
// MT3608, TP4056, indicator and pouch-cell sizes, the PCB thicknesses and USB-C
// port heights, the RD-03D PCB thickness (1.6 assumed) and J1 height (4.5
// allowed; a user measured 3.8).  Measure your modules before printing the base.
//
// Render:
//   openscad -D 'part="base"'          -o level2_sensor_node_base.stl level2_sensor_node.scad
//   openscad -D 'part="lid"'           -o level2_sensor_node_lid.stl  level2_sensor_node.scad
//   openscad -D 'part="ld19_coupon"'   -o level2_ld19_coupon.stl      level2_sensor_node.scad
//   openscad -D 'part="rd03d_coupon"'  -o level2_rd03d_coupon.stl     level2_sensor_node.scad
//   openscad -D 'part="ld19_template"' -o level2_ld19_template.svg    level2_sensor_node.scad
//
// SPDX-License-Identifier: MIT

part = "assembly";
shell_alpha = 0.55;  // transparency of base and lid in the "assembly" preview only

/* ====================================================================== shell */
wall        = 2.4;   // side walls; the outer `lid_skin` of it rises around the flush lid
floor_t     = 2.0;
lid_t       = 2.5;   // lid plate; its top face is the LD19 mounting plane
lid_skin    = 1.2;   // outer part of the wall that stays beside the lid (rebate)
lid_clear   = 0.15;  // lid plate to rebate, per side
corner_r    = 3.0;   // outer corner radius
clear       = 0.4;   // added around a part inside its corner guides
guide_t     = 1.2;   // corner guide thickness
gap         = 2.6;   // between neighbouring parts (room for two guides + 0.2)
edge_gap    = 1.0;   // part to wall
post        = 7.0;   // square corner posts for the lid screws
insert_d    = 4.2;   // heat-set insert pilot: M3 brass insert 4.4-4.6 OD wants 4.0, printed holes come out ~0.25 small [INSERT]
insert_h    = 6.0;
lid_screw_d = 3.3;   // M3 clearance through the lid
$fn = 48;

/* ============================================================== RD-03D radar */
rd03d_pcb       = [15.0, 44.0, 1.6]; // width, length [RD-DS Table 1, ±0.2]; thickness MEASURE (1.6 assumed)
rd03d_conn_h    = 4.5;   // J1 (1.25 mm wafer, pins 5V GND TX RX) behind the PCB back [RD-DS Fig. 4]; a user measured 3.8, allow 4.5 MEASURE
rd03d_corner_r  = 2.0;   // R2.0 at two diagonal corners, R1.1 notches at the other two [RD-DS Fig. 4]
rd03d_window_t  = 1.0;   // window wall <= lambda/8 in plastic: ABS/PETG 1.0, PC 0.89 [LD2450-DG]
rd03d_gap       = 0.8;   // antenna face to window inner face, <= 1.25 (lambda/10 in air) [LD2450-DG]
rd03d_window    = [24, 50]; // thinned pane (x, z), centred on the board; >= rd03d_clear
rd03d_clear     = [18, 34]; // zone around the antenna that no other wall feature may enter
rd03d_ant_frac  = 2/3;   // antenna patches: the two thirds of the FRONT face nearest J2 (top) [RD-DS Fig. 4]
rd03d_grip      = 1.0;   // groove depth into each long edge (the edge bands carry no parts)
rd03d_grip_h    = 26;    // groove length from the board bottom; the top 18 mm stay free so the
                         // J2 pads (seven 2.54 mm holes 1.51 mm from the long edge [RD-DS Fig. 4]) are not pinched
rd03d_rail_w    = 3.0;   // cradle rail width (x)
rd03d_back_lip  = 1.2;   // rail material behind the board back face, at the edges only
rd03d_z0        = 5.0;   // board bottom edge above the floor top: the J1 cable can turn below the board
rd03d_top_clear = 3.0;   // board top edge to lid underside
metal_keepout   = 20;    // project rule: no cell, switch body or screw within 20 mm of the antenna face

/* ================================================================= LD19 LIDAR */
ld19_body       = 38.59; // square base at the mounting plane, rotation axis at its centre [LD19-DS p.7, LD19-STEP]
ld19_body_top   = 37.08; // square at the top of the base (2 deg draft, widest at the bottom) [LD19-STEP]
ld19_h          = 34.8;  // total height above the mounting plane [LD19-DS]
ld19_cap_d      = 35.29; // rotating cap diameter [LD19-STEP]
ld19_cap_h      = 12.6;  // rotating cap height [LD19-STEP]
ld19_holes      = [[23.40, 8.52], [-23.40, 8.52], [0, -23.40]]; // ear holes, +y toward the connector wall [LD19-DS p.7]
ld19_hole_d_sensor = 2.50;   // through hole in each ear [LD19-DS p.7]
ld19_cbore      = [4.46, 2.0]; // counterbore on the UNDERSIDE of each ear, dia x depth [LD19-STEP]
ld19_ear_d      = 7.20;  // ear boss diameter [LD19-STEP]
ld19_ear_t      = 3.6;   // ear thickness [LD19-STEP]
ld19_ear_x      = 27.0;  // side ears reach x = +-27.0 (54.0 overall) [LD19-STEP]
ld19_ear_y_rear = -27.0; // rear ear tip; 46.29 overall from the connector wall [LD19-STEP]
ld19_conn_wall_y = 19.3; // connector wall = +y face of the base [LD19-DS]
ld19_conn_w     = 9.5;   // JST ZH1.5T-4P opening width, centred [LD19-STEP]
ld19_conn_z     = [2.9, 7.3]; // opening above the mounting plane; cable leaves horizontally [LD19-STEP]
ld19_window_z   = [23, 33];   // optical band above the mounting plane (beam plane ~27) [LD19-DS]
ld19_hole_d     = 2.7;   // M2.5 clearance through the lid
ld19_boss       = [4.2, 1.8]; // locating boss into the counterbore: 0.13 radial, 0.2 axial clearance
ld19_nut        = [5.0, 2.0]; // M2.5 nut (DIN 934): across flats, thickness
ld19_nut_clear  = 0.25;  // added to the across-flats of the nut pocket
ld19_cable_slot = [10, 7];    // x, y: through-slot just outside the connector wall
ld19_cable_slot_gap = 0.7;    // connector wall to slot
ld19_pos        = [0, 4];     // rotation axis on the lid (x, y); 4 mm toward +y keeps the rear ear's screw 20 mm from the antenna

/* ======================= other modules — researched, not yet measured (MEASURE) */
xiao        = [21.135, 17.78, 1.6]; // PCB x (USB-C at +x), y [XIAO-KICAD]; thickness MEASURE
xiao_r      = 1.905;              // PCB corner radius [XIAO-KICAD]
xiao_usb    = [9.0, 7.35, 3.3];   // USB-C receptacle: width across the board, depth, height (generic model) MEASURE
xiao_usb_overhang = 1.5;          // receptacle beyond the PCB edge [XIAO-KICAD] MEASURE
xiao_standoff = 2.0;              // the XIAO lies on its BACK: that side carries only the two battery pads
                                  // (2.03 x 1.02, 1.9 apart, 4.4 from one long edge, 8.3 / 10.2 from the USB
                                  // edge, BAT- nearer the USB) and test pads [XIAO-KICAD]; 2 mm for the wires
xiao_comp_h = 4.5;                // component height on the FRONT (U.FL and 30-pin connector opposite the USB-C)
mt3608      = [37.2, 17.2, 14.0]; // PCB x, y [SELLERS 36-37.2 x 17-17.2]; height incl. 10-turn trimmer and inductor
mt3608_pcb_t = 1.2;               // [NOPSCAD]; R2 corners, no mounting holes (held by corner guides)
tp4056      = [28.0, 17.3, 4.9];  // TP4056 + DW01A/FS8205A USB-C charger: PCB x (port at +x), y, height 4.3-4.9 [SELLERS]
tp4056_pcb_t = 1.6;               // MEASURE
tp4056_usb  = [9.0, 3.3];         // receptacle width, height (generic USB-C) MEASURE
tp4056_usb_overhang = 1.5;        // USB-C centred on one short end, 29 overall incl. the connector [SELLERS]
indicator_pcb    = [43.5, 20.0, 8.6]; // 1S 4-LED indicator board: x, z (against the back wall), height incl. LEDs + connector [SELLERS] MEASURE
indicator_pcb_t  = 1.6;               // MEASURE
indicator_pocket = [44, 21, 9];       // pocket behind the window: x, z, depth into the box
indicator_window = [22, 6];           // the four LEDs sit in a row along one long edge
indicator_led_offset = 4.0;           // LED row centre below the board's top edge MEASURE
indicator_standoff = 7.0;             // PCB component face to the inner wall face: LEDs and connector toward the wall MEASURE
cell_type   = "103450";           // "103450" | "105575" | "606090" | "906090" | "custom"  [CELL-DS]
cell_custom = [10, 34, 50];       // thickness, width, length for cell_type == "custom"
switch_d    = 6.2;                // toggle switch panel hole
switch_body = [13, 15, 10];       // metal body behind the panel: x, depth, z        MEASURE
usb_opening = [11, 5.5];          // USB-C wall openings (w, h): 1 mm around a 9.0 x 3.3 receptacle
vent        = [2.0, 24];          // vent slot width, height
part_z      = 3.0;                // corner guide height above a PCB's seat (cell guides are 6)

/* ============================================================ derived layout */
// Pouch cells, thickness x width x length [CELL-DS]: 103450 ~1800 mAh (some listings
// 2000), 105575 5000 mAh, 606090 4000 mAh, 906090 6000 mAh.  The LIDAR node wants
// 606090 or larger for a useful runtime; the box grows with the cell.
cell = cell_type == "103450" ? [10, 34, 50] :
       cell_type == "105575" ? [10, 55, 75] :
       cell_type == "606090" ? [6, 60, 90] :
       cell_type == "906090" ? [9, 60.5, 92] : cell_custom;
cell_fp = [cell[1], cell[2]];     // lying flat, long axis along y
cell_h  = cell[0];

// Front zone.  ant_off is the antenna face relative to the nominal inner face of
// the front wall (negative = out in the thinned pane); keep_y is the first y,
// measured from that face, where metal may sit.
ant_off   = rd03d_window_t + rd03d_gap - wall;
keep_y    = ant_off + metal_keepout;
right_w   = max(tp4056[0], xiao[0], mt3608[0]);
right_d   = tp4056[1] + gap + xiao[1] + gap + mt3608[1];
left_w    = cell_fp[0];
left_d    = cell_fp[1];
back_inset = post + edge_gap;     // the back corners hold the posts
wall_off  = 0.5;                  // PCBs with a USB-C port stop this far from the right wall

box_w = ceil(edge_gap + left_w + gap + right_w + wall_off);   // internal width
box_d = ceil(keep_y + max(left_d, right_d) + back_inset);     // internal depth
box_h = rd03d_z0 + rd03d_pcb[1] + rd03d_top_clear;            // internal height (floor top to lid underside)
ow = box_w + 2 * wall;  od = box_d + 2 * wall;  oh = floor_t + box_h + lid_t;   // outer size
xL = -box_w / 2;  xR = box_w / 2;  yF = -box_d / 2;  yB = box_d / 2;
z_lid = floor_t + box_h;          // lid underside / post tops / ledge

// Footprints as [x0, y0, x1, y1]
cell_rect = [xL + edge_gap, yB - back_inset - left_d, xL + edge_gap + left_w, yB - back_inset];
tp_rect   = [xR - wall_off - tp4056[0], yB - back_inset - tp4056[1], xR - wall_off, yB - back_inset];
xiao_rect = [xR - wall_off - xiao[0], tp_rect[1] - gap - xiao[1], xR - wall_off, tp_rect[1] - gap];
mt_rect   = [xR - wall_off - mt3608[0], xiao_rect[1] - gap - mt3608[1], xR - wall_off, xiao_rect[1] - gap];
parts_2d  = [["cell", cell_rect, cell_h], ["TP4056", tp_rect, tp4056[2]],
             ["XIAO", xiao_rect, xiao_standoff + xiao[2] + xiao_comp_h], ["MT3608", mt_rect, mt3608[2]]];
post_c    = [for (sx = [-1, 1], sy = [-1, 1]) [sx * (box_w / 2 - post / 2), sy * (box_d / 2 - post / 2)]];
post_rects = [for (c = post_c) [c[0] - post / 2, c[1] - post / 2, c[0] + post / 2, c[1] + post / 2]];

// Radar geometry (y positions are absolute)
y_win_in = -od / 2 + rd03d_window_t;      // inner face of the window pane
y_ant    = y_win_in + rd03d_gap;          // antenna face
y_back   = y_ant + rd03d_pcb[2];          // PCB back face
y_conn   = y_back + rd03d_conn_h;         // back of the J1 wafer
zb       = floor_t + rd03d_z0;            // board bottom edge
zt       = zb + rd03d_pcb[1];             // board top edge
ant_z    = [zt - rd03d_ant_frac * rd03d_pcb[1], zt];
win_z    = [zb + rd03d_pcb[1] / 2 - rd03d_window[1] / 2, zb + rd03d_pcb[1] / 2 + rd03d_window[1] / 2];
clear_zc = min((ant_z[0] + ant_z[1]) / 2, win_z[1] - rd03d_clear[1] / 2);
clear_z  = [clear_zc - rd03d_clear[1] / 2, clear_zc + rd03d_clear[1] / 2];
rail_x_in  = rd03d_pcb[0] / 2 + 0.15 - rd03d_grip;   // groove bottom (board edge sits 0.15 off it)
rail_x_out = rail_x_in + rd03d_rail_w;
rail_y1    = y_back + 0.15 + rd03d_back_lip;
groove_y   = [y_ant - 0.15, y_back + 0.15];

// Back wall: indicator board in a grooved pocket on the inside (LEDs toward the
// window), toggle switch below it, vents to the right of the pocket.  Side walls:
// vents (left, above the cell) and the two USB-C openings (right).
ind_grip    = 1.0;                                         // groove depth into each short edge of the board
ind_rail_w  = 2.5;
ind_xin     = indicator_pcb[0] / 2 + 0.15 - ind_grip;      // rail inner face from the pocket centre
ind_x       = xL + post + edge_gap + ind_xin + ind_rail_w; // pocket centre
ind_z1      = z_lid - 3;                                   // pocket top: the board slides in from above
ind_z0      = ind_z1 - indicator_pocket[1];                // groove bottom: the board rests here
ind_groove_y = [yB - indicator_standoff - indicator_pcb_t - 0.2, yB - indicator_standoff + 0.1];
indicator_x = ind_x;
indicator_z = ind_z0 + indicator_pcb[1] - indicator_led_offset;   // window centre
switch_x    = ind_x - 8;
switch_z    = floor_t + cell_h + 2 + switch_body[2] / 2;   // above the cell (which is inset 8 mm from this wall)
switch_box  = [switch_x - switch_body[0] / 2, yB - switch_body[1], switch_z - switch_body[2] / 2,
               switch_x + switch_body[0] / 2, yB, switch_z + switch_body[2] / 2];   // x0 y0 z0 x1 y1 z1
vent_z0     = floor_t + 16;
back_vent_x = [for (x = [ind_x + ind_xin + ind_rail_w + 3 : 4.5 : xR - post - 2]) x];
left_vent_y = [for (y = [cell_rect[1] + 8 : 6 : yB - back_inset - 2]) y];
right_vent_y = concat([for (y = [mt_rect[1] + 2 : 6 : mt_rect[3] - 2]) y],
                      [for (y = [tp_rect[1] + 2 : 6 : tp_rect[3] - 2]) y]);
// USB-C openings in the right wall: [y centre, z centre]
usb_ports = [[(xiao_rect[1] + xiao_rect[3]) / 2, floor_t + xiao_standoff + xiao[2] + xiao_usb[2] / 2],
             [(tp_rect[1] + tp_rect[3]) / 2, floor_t + tp4056_pcb_t + tp4056_usb[1] / 2]];

// Lid
lid_w = box_w + 2 * (wall - lid_skin) - 2 * lid_clear;
lid_d = box_d + 2 * (wall - lid_skin) - 2 * lid_clear;
lid_r = corner_r - lid_skin - lid_clear;
ld19_cx = ld19_pos[0];  ld19_cy = ld19_pos[1];
slot_rect = [ld19_cx - ld19_cable_slot[0] / 2, ld19_cy + ld19_conn_wall_y + ld19_cable_slot_gap,
             ld19_cx + ld19_cable_slot[0] / 2, ld19_cy + ld19_conn_wall_y + ld19_cable_slot_gap + ld19_cable_slot[1]];
ld19_foot = [ld19_cx - ld19_ear_x, ld19_cy + ld19_ear_y_rear, ld19_cx + ld19_ear_x, slot_rect[3]]; // ears + slot
ant_recess = [xR - 16, yF + keep_y, xR - 3, yF + keep_y + 26];   // XIAO flex antenna taped under the lid, 0.6 deep
nut_pocket_r = (ld19_nut[0] + ld19_nut_clear) / cos(30) / 2;     // circumscribed radius of the hex pocket

/* =========================================================== geometry helpers */
function rect_inside(a, b, m = 0) = a[0] >= b[0] + m && a[1] >= b[1] + m && a[2] <= b[2] - m && a[3] <= b[3] - m;
function rects_overlap(a, b) = a[0] < b[2] && b[0] < a[2] && a[1] < b[3] && b[1] < a[3];
function circle_rect_hit(c, r, q) =
    let(dx = max(q[0] - c[0], 0, c[0] - q[2]), dy = max(q[1] - c[1], 0, c[1] - q[3])) dx * dx + dy * dy < r * r;
function box3(r, z0, z1) = [r[0], r[1], z0, r[2], r[3], z1];           // [x0 y0 z0 x1 y1 z1]
// distance from a 3D box to the antenna rectangle (plane y = y_ant, |x| <= 7.5, z in ant_z)
function ant_dist(b) = let(
    dx = max(0, b[0] - rd03d_pcb[0] / 2, -rd03d_pcb[0] / 2 - b[3]),
    dy = max(0, b[1] - y_ant, y_ant - b[4]),
    dz = max(0, b[2] - ant_z[1], ant_z[0] - b[5])) sqrt(dx * dx + dy * dy + dz * dz);
function cbox(c, r, z0, z1) = [c[0] - r, c[1] - r, z0, c[0] + r, c[1] + r, z1];  // box around a circle

/* ================================================================== checks */
// Every assert() aborts the render with a message; fix the parameter it names.
module checks() {
    assert(rd03d_window_t <= 1.3, str("rd03d_window_t = ", rd03d_window_t, " > 1.3 mm (lambda/8 in plastic)"));
    assert(rd03d_gap <= 1.25, str("rd03d_gap = ", rd03d_gap, " > 1.25 mm (lambda/10 in air)"));
    assert(rd03d_window[0] >= rd03d_clear[0] && win_z[0] <= clear_z[0] + 1e-6 && win_z[1] + 1e-6 >= clear_z[1],
           "radar window pane smaller than the clear zone rd03d_clear");
    assert(win_z[0] >= floor_t && win_z[1] <= z_lid + 1e-6, "radar window pane runs into the floor or the lid ledge");
    assert(zt + rd03d_top_clear <= z_lid + 1e-6, "RD-03D board does not fit under the lid: raise box_h");
    assert(rd03d_window[0] / 2 < box_w / 2 - post - 2, "radar window reaches the front corner posts");
    assert(rail_x_out + 2 < box_w / 2 - post, "radar cradle rails reach the front corner posts");
    cavity = [xL, yF, xR, yB];
    for (p = parts_2d) {
        assert(rect_inside(p[1], cavity, clear), str(p[0], " pocket does not fit inside the box"));
        for (q = post_rects) assert(!rects_overlap(p[1], q), str(p[0], " pocket overlaps a corner post"));
        for (o = parts_2d) assert(o[0] == p[0] || !rects_overlap(p[1], o[1]), str(p[0], " pocket overlaps ", o[0]));
        assert(ant_dist(box3(p[1], floor_t, floor_t + p[2])) + 1e-6 >= metal_keepout,
               str(p[0], " is inside the ", metal_keepout, " mm keep-out in front of the radar antenna"));
    }
    assert(ant_dist(switch_box) + 1e-6 >= metal_keepout, "switch body is inside the radar keep-out");
    assert(switch_box[0] > xL + post && switch_box[3] < xR - post, "switch body hits a back corner post");
    assert(switch_box[2] > floor_t + cell_h + 1, "switch body hits the cell");
    assert(switch_box[5] < ind_z0 - 3, "switch body hits the indicator pocket: lower the switch or raise the pocket");
    assert(ind_x - ind_xin - ind_rail_w >= xL + post && ind_x + ind_xin + ind_rail_w <= xR - post,
           "indicator pocket hits a back corner post");
    assert(ind_z0 - 2 > floor_t + max(cell_h, tp4056[2]) + 1 && ind_z1 <= z_lid - 1,
           "indicator pocket collides with the cell / TP4056 below it or with the lid");
    assert(indicator_pcb[2] <= indicator_pocket[2] && indicator_pcb[0] <= indicator_pocket[0] && indicator_pcb[1] <= indicator_pocket[1],
           "indicator board larger than its pocket");
    assert(slot_rect[3] < yB - indicator_pocket[2] - 1.5, "LD19 cable slot drops onto the indicator pocket");
    assert(len(back_vent_x) > 0 && back_vent_x[len(back_vent_x) - 1] + vent[0] / 2 < xR - post, "back vents reach a post");
    assert(usb_ports[0][1] + usb_opening[1] / 2 < vent_z0 && usb_ports[1][1] + usb_opening[1] / 2 < vent_z0, "USB-C openings run into the vents");
    // lid and LD19
    lid_rect = [-lid_w / 2, -lid_d / 2, lid_w / 2, lid_d / 2];
    assert(rect_inside(ld19_foot, lid_rect, 1.0), "LD19 ear footprint (54 x 46.3 plus cable slot) does not fit on the lid");
    assert(ld19_boss[0] < ld19_cbore[0] && ld19_boss[1] < ld19_cbore[1], "locating boss does not fit the LD19 counterbore");
    assert(lid_t - ld19_nut[1] + ld19_boss[1] >= 1.5, "nut pocket leaves less than 1.5 mm under the boss");
    for (h = ld19_holes) {
        hc = [ld19_cx + h[0], ld19_cy + h[1]];
        assert(!circle_rect_hit(hc, ld19_hole_d / 2 + 1.0, slot_rect), "LD19 cable slot intersects a mounting hole");
        assert(!circle_rect_hit(hc, nut_pocket_r + 1.0, ant_recess), "antenna recess intersects an LD19 nut pocket");
        for (c = post_c) assert(norm(hc - c) > nut_pocket_r + lid_screw_d / 2 + 1.0, "LD19 hole too close to a lid screw");
        assert(ant_dist(cbox(hc, ld19_ear_d / 2, z_lid - ld19_nut[1], z_lid + 20)) + 1e-6 >= metal_keepout,
               "an LD19 screw/nut is inside the radar keep-out: move ld19_pos toward +y");
    }
    for (c = post_c) {
        assert(!circle_rect_hit(c, lid_screw_d / 2 + 1.0, slot_rect), "LD19 cable slot intersects a lid screw hole");
        assert(!circle_rect_hit(c, lid_screw_d / 2 + 1.0, ant_recess), "antenna recess intersects a lid screw hole");
        assert(ant_dist(cbox(c, insert_d / 2, z_lid - insert_h, z_lid + 20)) + 1e-6 >= metal_keepout,
               "a lid screw / insert is inside the radar keep-out");
    }
    assert(ant_recess[2] < lid_w / 2 - lid_skin - lid_clear - 1 && ant_recess[1] > -lid_d / 2 + 3,
           "antenna recess reaches the lid rim");
    assert(ld19_boss[1] + 3.0 < 20 && ld19_boss[1] + 3.0 < ld19_window_z[0], "something on the lid rises above 20 mm toward the LD19 optical band");  // 3.0 = M3 screw head
}

/* ============================================================ shape helpers */
module rrect(w, d, h, r) {
    hull() for (sx = [-1, 1], sy = [-1, 1]) translate([sx * (w / 2 - r), sy * (d / 2 - r), 0]) cylinder(r = r, h = h);
}
module rrect2d(w, d, r) {
    hull() for (sx = [-1, 1], sy = [-1, 1]) translate([sx * (w / 2 - r), sy * (d / 2 - r)]) circle(r = r);
}
module rect2d(q, r = 0) {   // 2D rounded rectangle from [x0 y0 x1 y1]
    translate([(q[0] + q[2]) / 2, (q[1] + q[3]) / 2])
        if (r > 0) rrect2d(q[2] - q[0], q[3] - q[1], r); else square([q[2] - q[0], q[3] - q[1]], center = true);
}
module frame2d(w, d, r, t) { difference() { rrect2d(w, d, r); rrect2d(w - 2 * t, d - 2 * t, max(r - t, 0.05)); } }
module ring2d(r, t) { difference() { circle(r = r + t); circle(r = r); } }
module bar2d(p, q, t) { hull() { translate(p) circle(d = t, $fn = 12); translate(q) circle(d = t, $fn = 12); } }

// L-shaped corner guides around a footprint q = [x0 y0 x1 y1].  wallside = "x1"
// drops the bars parallel to a wall at x1, so a USB-C opening there stays free.
module guides(q, h, leg = 6, wallside = "") {
    x0 = q[0] - clear;  y0 = q[1] - clear;  x1 = q[2] + clear;  y1 = q[3] + clear;
    for (c = [[x0, y0, 0, 0], [x1, y0, 1, 0], [x0, y1, 0, 1], [x1, y1, 1, 1]])
        translate([c[0], c[1], floor_t - 0.01]) mirror([c[2], 0, 0]) mirror([0, c[3], 0]) {
            translate([-guide_t, -guide_t, 0]) cube([leg + guide_t, guide_t, h + 0.01]);
            if (!(wallside == "x1" && c[2] == 1))
                translate([-guide_t, -guide_t, 0]) cube([guide_t, leg + guide_t, h + 0.01]);
        }
}

/* ===================================================================== base */
module base() {
    difference() {
        rrect(ow, od, oh, corner_r);
        // cavity
        translate([0, 0, floor_t]) rrect(box_w, box_d, box_h + lid_t + 1, max(corner_r - wall, 0.5));
        // rebate for the flush lid
        translate([0, 0, z_lid]) rrect(lid_w + 2 * lid_clear, lid_d + 2 * lid_clear, lid_t + 1, max(lid_r + lid_clear, 0.5));
        // radar window: thin the front wall to rd03d_window_t over the pane
        translate([-rd03d_window[0] / 2, y_win_in, win_z[0]])
            cube([rd03d_window[0], wall - rd03d_window_t + 0.01, win_z[1] - win_z[0]]);
        // USB-C openings in the right wall: XIAO (programming) and TP4056 (charging)
        for (p = usb_ports) translate([xR - 0.01, p[0] - usb_opening[0] / 2, p[1] - usb_opening[1] / 2])
            cube([wall + 0.02, usb_opening[0], usb_opening[1]]);
        // toggle switch and indicator window in the back wall
        translate([switch_x, yB - 0.01, switch_z]) rotate([-90, 0, 0]) cylinder(d = switch_d, h = wall + 0.02);
        translate([indicator_x - indicator_window[0] / 2, yB - 0.01, indicator_z - indicator_window[1] / 2])
            cube([indicator_window[0], wall + 0.02, indicator_window[1]]);
        // vents: back wall (right half), left wall (above the cell), right wall (above the boards)
        for (x = back_vent_x) translate([x - vent[0] / 2, yB - 0.01, vent_z0]) cube([vent[0], wall + 0.02, vent[1]]);
        for (y = left_vent_y) translate([xL - wall - 0.01, y - vent[0] / 2, vent_z0]) cube([wall + 0.02, vent[0], vent[1]]);
        for (y = right_vent_y) translate([xR - 0.01, y - vent[0] / 2, vent_z0]) cube([wall + 0.02, vent[0], vent[1]]);
    }
    // corner posts with heat-set insert holes for the lid screws
    for (c = post_c) difference() {
        translate([c[0] - post / 2, c[1] - post / 2, floor_t - 0.01]) cube([post, post, box_h + 0.01]);
        translate([c[0], c[1], z_lid - insert_h]) cylinder(d = insert_d, h = insert_h + 1);
    }
    // RD-03D cradle: two rails on the inside of the window pane, each with a
    // groove that takes rd03d_grip of the long edge over the lower rd03d_grip_h;
    // the board slides in from the top (lid off) and rests on the groove bottoms.
    // The gap between the rails below the board is the slot for the J1 cable.
    for (m = [0, 1]) mirror([m, 0, 0]) difference() {
        translate([rail_x_in, y_win_in - 0.01, floor_t - 0.01])
            cube([rd03d_rail_w, rail_y1 - y_win_in + 0.01, zb + rd03d_grip_h - floor_t + 0.01]);
        translate([rail_x_in - 0.01, groove_y[0], zb])
            cube([rd03d_grip + 0.02, groove_y[1] - groove_y[0], rd03d_grip_h + 1]);
    }
    // floor pockets: corner guides, no screws (none of the modules has mounting holes)
    guides(cell_rect, 6, 8);
    guides(tp_rect, part_z, 6, "x1");
    guides(xiao_rect, xiao_standoff + part_z, 6, "x1");
    guides(mt_rect, part_z, 6, "x1");
    // the XIAO lies on its back on two strips under its long edges: the battery wires
    // soldered to the back pads (4.4 mm from one long edge) have xiao_standoff of room
    for (y = [xiao_rect[1] + 0.8, xiao_rect[3] - 0.8])
        translate([xiao_rect[0] + 1, y - 0.8, floor_t - 0.01]) cube([xiao[0] - 2, 1.6, xiao_standoff + 0.01]);
    // indicator pocket: two grooved rails on the inside of the back wall; the board
    // slides in from above (lid off), LEDs toward the window, and rests at ind_z0
    for (m = [0, 1]) translate([ind_x, 0, 0]) mirror([m, 0, 0]) difference() {
        translate([ind_xin, yB - indicator_pocket[2] - 0.5, ind_z0 - 2])
            cube([ind_rail_w, indicator_pocket[2] + 0.5 + 0.01, ind_z1 - ind_z0 + 2]);
        translate([ind_xin - 0.01, ind_groove_y[0], ind_z0])
            cube([ind_grip + 0.02, ind_groove_y[1] - ind_groove_y[0], indicator_pocket[1] + 1]);
    }
}

/* ================================================================== ghosts */
module rd03d_outline2d() {   // board frame: u across the 15 mm width, v along the 44 mm length
    w = rd03d_pcb[0];  l = rd03d_pcb[1];  r = rd03d_corner_r;
    difference() {
        hull() {
            translate([-w / 2 + r, l / 2 - r]) circle(r = r);
            translate([w / 2 - r, -l / 2 + r]) circle(r = r);
            translate([w / 2 - 0.02, l / 2 - 0.02]) square(0.02);
            translate([-w / 2, -l / 2]) square(0.02);
        }
        translate([w / 2, l / 2]) circle(r = 1.1);      // R1.1 notches at the other two corners
        translate([-w / 2, -l / 2]) circle(r = 1.1);
    }
}
module rd03d_ghost() {
    // rotate([90,0,0]) maps (u, v, extrusion) to (x, -y, z): the extrusion fills y_ant..y_back
    color("darkgreen", 0.75) translate([0, y_back, zb + rd03d_pcb[1] / 2]) rotate([90, 0, 0])
        linear_extrude(rd03d_pcb[2]) rd03d_outline2d();
    color("gold", 0.9) translate([-6, y_ant - 0.1, ant_z[0]]) cube([12, 0.1, ant_z[1] - ant_z[0]]);   // antenna patches, schematic
    color("ivory", 0.9) translate([-4, y_back, zb + 1.5]) cube([8, rd03d_conn_h, 5.5]);              // J1 wafer
}
module ld19_ghost() {
    translate([ld19_cx, ld19_cy, oh]) {
        color("dimgray", 0.55) hull() {
            linear_extrude(0.01) square(ld19_body, center = true);
            translate([0, 0, ld19_h - ld19_cap_h]) linear_extrude(0.01) square(ld19_body_top, center = true);
        }
        color("black", 0.6) translate([0, 0, ld19_h - ld19_cap_h]) cylinder(d = ld19_cap_d, h = ld19_cap_h);
        color("red", 0.3) translate([0, 0, ld19_window_z[0]]) difference() {       // optical band
            cylinder(d = ld19_cap_d + 1.0, h = ld19_window_z[1] - ld19_window_z[0]);
            translate([0, 0, -1]) cylinder(d = ld19_cap_d - 1.0, h = 20);
        }
        for (h = ld19_holes) color("dimgray", 0.55) difference() {
            hull() {
                translate([h[0], h[1], 0]) cylinder(d = ld19_ear_d, h = ld19_ear_t);
                translate([h[0] * 0.72, h[1] * 0.72, 0]) cylinder(d = ld19_ear_d, h = ld19_ear_t);
            }
            translate([h[0], h[1], -1]) cylinder(d = ld19_hole_d_sensor, h = 10);
            translate([h[0], h[1], -0.01]) cylinder(d = ld19_cbore[0], h = ld19_cbore[1]);
        }
        color("white", 0.9) translate([-ld19_conn_w / 2, ld19_conn_wall_y - 1.5, ld19_conn_z[0]])
            cube([ld19_conn_w, 1.5, ld19_conn_z[1] - ld19_conn_z[0]]);
        color("red", 0.9) translate([0, -ld19_body / 2 - 4, 0.5]) linear_extrude(0.5)    // 0 deg marker
            polygon([[-2.5, 0], [2.5, 0], [0, -4]]);
    }
}
module parts_ghost() {
    color("royalblue", 0.5) translate([cell_rect[0], cell_rect[1], floor_t]) cube([cell_fp[0], cell_fp[1], cell_h]);
    color("green", 0.6) translate([tp_rect[0], tp_rect[1], floor_t]) cube([tp4056[0], tp4056[1], tp4056[2]]);
    color("green", 0.6) translate([xiao_rect[0], xiao_rect[1], floor_t + xiao_standoff]) cube([xiao[0], xiao[1], xiao[2]]);
    color("gray", 0.7) translate([xiao_rect[2] - xiao_usb[1] + xiao_usb_overhang, usb_ports[0][0] - xiao_usb[0] / 2, floor_t + xiao_standoff + xiao[2]])
        cube([xiao_usb[1], xiao_usb[0], xiao_usb[2]]);
    color("gray", 0.7) translate([tp_rect[2] - 7.35 + tp4056_usb_overhang, usb_ports[1][0] - tp4056_usb[0] / 2, floor_t + tp4056_pcb_t])
        cube([7.35, tp4056_usb[0], tp4056_usb[1]]);
    color("green", 0.6) translate([mt_rect[0], mt_rect[1], floor_t]) cube([mt3608[0], mt3608[1], mt3608[2]]);
    color("silver", 0.6) translate([switch_box[0], switch_box[1], switch_box[2]]) cube([switch_body[0], switch_body[1], switch_body[2]]);
    color("silver", 0.8) translate([switch_x, yB + wall, switch_z]) rotate([-90, 0, 0]) cylinder(d = 5.5, h = 9);
    color("orange", 0.6) translate([ind_x - indicator_pcb[0] / 2, ind_groove_y[0] + 0.2, ind_z0])
        cube([indicator_pcb[0], indicator_pcb_t, indicator_pcb[1]]);                       // indicator PCB
    for (i = [-1.5 : 1 : 1.5]) color("lime", 0.9)                                           // its four LEDs
        translate([indicator_x + i * 5 - 1, yB - 2.5, indicator_z - 1]) cube([2, 2.5, 2]);
}

/* ============================================================== LD19 mount */
// Both modules are shared by the lid and the test coupon, so the coupon has
// exactly the lid's hole pattern, bosses, nut pockets and cable slot.
module ld19_mount_add(plate_t) {      // Ø4.2 x 1.8 locating bosses on the top face
    for (h = ld19_holes) translate([ld19_cx + h[0], ld19_cy + h[1], plate_t - 0.01])
        cylinder(d = ld19_boss[0], h = ld19_boss[1] + 0.01);
}
module ld19_mount_cut(plate_t) {      // Ø2.7 through holes, M2.5 nut pockets from below, cable slot
    for (h = ld19_holes) translate([ld19_cx + h[0], ld19_cy + h[1], 0]) {
        translate([0, 0, -1]) cylinder(d = ld19_hole_d, h = plate_t + ld19_boss[1] + 2);
        translate([0, 0, -0.01]) rotate([0, 0, 30]) cylinder(r = nut_pocket_r, h = ld19_nut[1] + 0.01, $fn = 6);
    }
    translate([0, 0, -1]) linear_extrude(plate_t + 2) rect2d(slot_rect, 1.5);
}

/* ====================================================================== lid */
// Flush lid: sits in the rebate of the base on the four posts and the 1.2 mm
// ledge, held by four M3 screws into heat-set inserts.  Print it top face up:
// the underside carries only cut-ins (nut pockets, antenna recess).
module lid() {
    difference() {
        union() {
            rrect(lid_w, lid_d, lid_t, lid_r);
            ld19_mount_add(lid_t);
        }
        ld19_mount_cut(lid_t);
        for (c = post_c) translate([c[0], c[1], -1]) cylinder(d = lid_screw_d, h = lid_t + 2);
        // 0.6 mm recess under the lid for the XIAO's flex antenna (taped), away from the cell
        translate([ant_recess[0], ant_recess[1], -0.01])
            cube([ant_recess[2] - ant_recess[0], ant_recess[3] - ant_recess[1], 0.6 + 0.01]);
    }
}

/* ================================================================== coupons */
coupon = [60, 60, 2.5];
module ld19_coupon() {
    difference() {
        union() {
            rrect(coupon[0], coupon[1], coupon[2], 3);
            translate([-ld19_cx, -ld19_cy, 0]) ld19_mount_add(coupon[2]);
        }
        translate([-ld19_cx, -ld19_cy, 0]) ld19_mount_cut(coupon[2]);
        // engraved 0 deg arrow: points to -y, the sensor's rear ear goes here
        translate([0, -coupon[1] / 2 + 2.5, coupon[2] - 0.6]) linear_extrude(1) polygon([[-2, 2.5], [2, 2.5], [0, -1.5]]);
    }
}
module rd03d_coupon() {   // 30 mm wide slice of the front wall with the pane, the rails and a floor stub
    intersection() {
        base();
        translate([-15, -od / 2 - 1, -1]) cube([30, 13, oh + 2]);
    }
}

/* ============================================================ 2D template */
// Export with -o level2_ld19_template.svg and print at 100 %.  Lid outline and
// lid screws are drawn too, so the sheet also shows where the sensor sits.
module ld19_template() {
    t = 0.3;
    frame2d(lid_w, lid_d, lid_r, t);
    for (c = post_c) translate(c) ring2d(lid_screw_d / 2, t);
    difference() { rect2d(slot_rect, 1.5); offset(delta = -t) rect2d(slot_rect, 1.5); }
    translate(ld19_pos) {
        frame2d(ld19_body, ld19_body, 0.5, t);                       // base at the mounting plane
        for (h = ld19_holes) {
            translate(h) ring2d(ld19_hole_d_sensor / 2, t);          // Ø2.50 hole edge
            translate(h) for (a = [0 : 90 : 270]) rotate(a) translate([2.0, -t / 2]) square([2.8, t]);
            difference() {                                           // ear outline
                hull() { translate(h) circle(d = ld19_ear_d); translate(h * 0.72) circle(d = ld19_ear_d); }
                hull() { translate(h) circle(d = ld19_ear_d - 2 * t); translate(h * 0.72) circle(d = ld19_ear_d - 2 * t); }
            }
        }
        translate([-ld19_conn_w / 2, ld19_conn_wall_y - 1.0]) square([ld19_conn_w, 1.0]);   // connector opening
        bar2d([0, 0], [0, -17.5], 0.6);                              // 0 deg arrow through the rear ear,
        bar2d([0, -29.3], [0, -31], 0.6);                            // interrupted at the hole's crosshair
        polygon([[-2.5, -30.5], [2.5, -30.5], [0, -35]]);
        translate([3, -34.5]) text("0° = front, radar side", size = 2.8, font = "DejaVu Sans");
        translate([-ld19_conn_w / 2 - 27, ld19_conn_wall_y + 1.2]) text("connector + cable", size = 2.5, font = "DejaVu Sans");
    }
    translate([-lid_w / 2 + 3, lid_d / 2 - 6]) text("LD19 footprint 1:1", size = 3.2, font = "DejaVu Sans");
    translate([-lid_w / 2 + 3, lid_d / 2 - 10]) text("print at 100 %, check the bar", size = 2.4, font = "DejaVu Sans");
    translate([lid_w / 2 - 25, -lid_d / 2 + 3]) {                   // 20 mm scale bar
        square([20, t]);  square([t, 2]);  translate([20 - t, 0]) square([t, 2]);
        translate([5, 1.2]) text("20 mm", size = 2.4, font = "DejaVu Sans");
    }
}

/* =================================================================== output */
checks();
echo(str("internal ", box_w, " x ", box_d, " x ", box_h, " mm, outer ", ow, " x ", od, " x ", oh,
         " mm, with LD19 ", oh + ld19_h, " mm"));
echo(str("antenna face y=", y_ant, "  board z=", zb, "..", zt, "  keep-out line y=", yF + keep_y));
echo(str("cell ", cell_rect, " TP4056 ", tp_rect, " XIAO ", xiao_rect, " MT3608 ", mt_rect));
echo(str("LD19 holes on the lid ", [for (h = ld19_holes) [ld19_cx + h[0], ld19_cy + h[1]]], " cable slot ", slot_rect));

if (part == "base") base();
else if (part == "lid") lid();
else if (part == "ld19_coupon") ld19_coupon();
else if (part == "rd03d_coupon") rd03d_coupon();
else if (part == "ld19_template") ld19_template();
else if (part == "assembly") {
    color("khaki", shell_alpha) base();
    color("gold", shell_alpha) translate([0, 0, z_lid]) lid();
    ld19_ghost();
    rd03d_ghost();
    parts_ghost();
}
else assert(false, str("unknown part \"", part, "\""));
