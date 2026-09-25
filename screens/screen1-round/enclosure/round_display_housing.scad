// =============================================================================
//  Dash housing for the round display (screen 1)
//  ESP32-2424S012: ESP32-C3 + 1.28" 240x240 GC9A01 + CST816S touch, USB-C
// -----------------------------------------------------------------------------
//  A bevelled puck floating on a slim stalk, like a gauge pod. Parts:
//
//    cup     - the puck's body: one piece, a 45 degree bevel round the face, no
//              bezel ring and no screws on the front. The board goes in from
//              the BACK, glass first, against the front lip and a foam ring.
//              The USB-C plug leaves through a slot in the side wall, the two
//              side buttons through two holes.
//    cover   - the flat back: three posts press the board forward, three M2
//              countersunk screws into the wall, a shallow recess that the
//              stand's dock drops into, two magnet pockets and vents.
//    stand   - the mount: a round foot with a VHB pad pocket, a stalk that
//              rises and bends back to lie in the puck's plane, and a small
//              dock at its top with two magnets that holds the puck's back.
//              Nothing shows behind the puck. The USB-C plug hangs from the
//              puck's bottom with a right-angle lead going sideways.
//    wedge   - the alternative, lying down, for a dash you look down at.
//    testring- a 3 mm slice of the cavity with the USB slot: print it first
//              (a minute), drop the board in, adjust the numbers below.
//
//  Render one part:  openscad -o cup.stl -D 'part="cup"' round_display_housing.scad
//  Parts: "cup" "cover" "stand" "wedge" "testring"
//         "assembly" (the puck on the stand) "exploded" (the parts apart)
//
//  Everything in millimetres. Measure your board before printing (README).
// =============================================================================

part = "assembly";

/* ───────────── the board (measure these, defaults are the 2424S012) ───────── */
board_d      = 38.5;   // PCB diameter (the 38.5 of "38.5 x 37")
glass_d      = 36.0;   // cover glass outer diameter (sits on the PCB)
active_d     = 32.4;   // visible display area; the front lip must clear it
pcb_t        = 1.6;    // PCB thickness
front_stack  = 3.0;    // glass top above the PCB front face (LCD + touch glass)
back_stack   = 5.0;    // tallest thing on the back (JST battery socket, ESP module)

usb_angle    = 270;    // where the USB-C socket points (0 = right, 90 = up, 270 = down)
usb_w        = 13.0;   // slot width for the plug body (right-angle plug: ~12.5)
usb_h        = 7.5;    // slot height
usb_below    = 1.5;    // socket centre below the PCB back face

btn_angles   = [200, 240]; // side buttons (BOOT / RESET), degrees like usb_angle. [] = none
btn_d        = 4.0;    // finger-nail / paperclip hole
btn_below    = 1.5;    // switch plunger centre below the PCB back face

/* ───────────── printing and fit ─────────────────────────────────────────── */
clr_r        = 0.4;    // radial clearance around the PCB
clr_z        = 0.3;    // axial play
wall         = 4.6;    // cup wall (the cover screws bite into it)
cover_t      = 3.6;    // the back cover (dock recess + magnet pockets + floor)
lip_overlap  = 1.2;    // front lip over the glass edge
lip_t        = 1.6;    // front lip thickness
gasket_t     = 1.0;    // foam ring between lip and glass (takes up tolerance)
bevel        = 3.5;    // 45 deg bevel round the front edge: the lens look
post_angles  = [30, 150, 330]; // cover posts that press on the PCB edge (clear of USB and buttons)
post_w       = 3.0;    // post width along the wall
post_in      = 1.5;    // post reach into the cavity (under the PCB edge only)
screw_angles = [90, 220, 340]; // cover screws, M2 x 8 countersunk self-tapping, from the back.
                       // Unevenly spaced on purpose: the cover only fits one way round, so the
                       // dock recess is level when the USB slot is at the bottom
screw_hole   = 1.8;    // pilot hole in the cup wall
screw_head_d = 4.0;    // countersink in the cover
vent_angles  = [45, 75, 105, 135]; // vent slots in the cover, upper half (the stalk covers the lower)
magnet_d     = 10.0;   // two Ø10 x 2 mm disc magnets in cover and dock (0 = none)
magnet_t     = 2.0;
magnet_gap   = 16.0;   // centre-to-centre

/* ───────────── the stand (upright, on the dash top) ─────────────────────── */
face_tilt    = 12;     // the puck leans back from vertical by this much
plug_gap     = 9;      // clear height under the puck's bottom edge: a right-angle USB-C plug
stem_w       = 14;     // the stalk's section at the foot ...
stem_d       = 8;
stem_d_top   = 6;      // ... and its depth where it meets the dock (the dock is as deep)
stem_run     = 18;     // the straight run in the puck's plane before the dock
dock_h       = 14;     // the dock: a stadium magnet_gap + dock_h wide, dock_h tall
dock_t       = stem_d_top; // behind the puck's back plane, flush with the stalk ...
dock_boss    = 0.7;    // ... and this much into the cover's recess (locates the puck)
dock_clr     = 0.2;    // recess clearance round the boss
foot_d       = 40;     // round foot (a Ø34 VHB disc fits underneath)
foot_t       = 3.5;

/* ───────────── the wedge (alternative, lying down) ──────────────────────── */
tilt_deg     = 20;     // face tilts up towards the driver by this much
base_w       = 52;     // footprint (a 50 x 50 VHB pad fits underneath)
base_l       = 50;
base_min_t   = 4;      // thinnest edge of the wedge
recess_depth = 1.5;    // locating recess for the cup on top of the wedge

/* ───────────── derived ──────────────────────────────────────────────────── */
$fn = 120;
cav_d   = board_d + 2 * clr_r;                     // cavity diameter
outer_d = cav_d + 2 * wall;                        // puck diameter
z_pcb   = cover_t + clr_z + back_stack;            // PCB back face, from the puck's back
cup_h   = z_pcb + pcb_t + front_stack + gasket_t + lip_t;   // puck thickness
lip_in  = glass_d - 2 * lip_overlap;               // the opening
r_screw = cav_d / 2 + 2.0;                         // screw circle, in the wall
dock_w  = magnet_gap + dock_h;                     // the magnets sit in the stadium's round ends
eps = 0.01;

assert(lip_in >= active_d + 0.6, "the front lip would cover the display: reduce lip_overlap");
assert(r_screw - screw_hole / 2 - cav_d / 2 >= 1.0, "cover screws too close to the cavity");
assert(outer_d / 2 - r_screw - screw_head_d / 2 >= 0.4, "countersinks break out of the cover's edge");
assert(cover_t - dock_boss - 0.1 - magnet_t - 0.2 >= 0.6, "cover too thin for recess plus magnet pockets");
assert(dock_h >= magnet_d + 0.4 + 3, "dock too small round the magnets");
assert(dock_t + dock_boss - magnet_t - 0.2 >= 1.0, "dock too thin for the magnet pockets");

// The stand: the puck's centre sits here, so its bottom edge is plug_gap above the foot.
z_c = foot_t + plug_gap + (outer_d / 2) * cos(face_tilt);
/** The puck's frame on the stand: local z points at the driver (world -y, a
 *  little up), local y is up (leaning back by face_tilt). The cup's own z
 *  axis is this local z, so cup() and cover() drop straight in. */
module puck_frame() { translate([0, 0, z_c]) rotate([90 - face_tilt, 0, 0]) children(); }
/** A point of the puck frame in world coordinates. */
function pf(x, y, z) = [x, y * sin(face_tilt) - z * cos(face_tilt),
                        z_c + y * cos(face_tilt) + z * sin(face_tilt)];

/* ───────────── helpers ──────────────────────────────────────────────────── */
// A rounded slot through the cup wall at an angle: w wide, h tall, centred at
// height z. open_back runs it out through the cup's back, so the cover closes
// it and, printed front-down, there is nothing to bridge.
module wall_slot(ang, w, h, z, open_back = false) {
    r = min(2, h / 2 - 0.1);
    rotate([0, 0, ang])
        translate([cav_d / 2 - 1, 0, z])
            rotate([0, 90, 0])
                linear_extrude(wall + 2)
                    hull() {
                        offset(r = r) square([h - 2 * r, w - 2 * r], center = true);
                        if (open_back)
                            translate([h + 2, 0]) offset(r = r) square([h - 2 * r, w - 2 * r], center = true);
                    }
}
module round_hole(ang, d, z) {
    rotate([0, 0, ang])
        translate([cav_d / 2 - 1, 0, z])
            rotate([0, 90, 0])
                linear_extrude(wall + 2) circle(d = d);
}
module magnet_pockets(z0, depth) {
    if (magnet_d > 0)
        for (s = [-1, 1])
            translate([s * magnet_gap / 2, 0, z0])
                cylinder(d = magnet_d + 0.4, h = depth + eps);
}
module rounded_rect(w, l, r) { offset(r = r) offset(delta = -r) square([w, l], center = true); }
module stadium(w, h) { hull() for (s = [-1, 1]) translate([s * (w - h) / 2, 0]) circle(d = h); }

/* ───────────── the cup ──────────────────────────────────────────────────── */
module cup() {
    difference() {
        // the body: from the cover up to the face, bevelled at the front
        rotate_extrude()
            polygon([[0, cover_t], [outer_d / 2, cover_t], [outer_d / 2, cup_h - bevel],
                     [outer_d / 2 - bevel, cup_h], [0, cup_h]]);
        // cavity, open at the back, closed by the front lip
        translate([0, 0, cover_t - 1]) cylinder(d = cav_d, h = cup_h - lip_t - cover_t + 1);
        // the opening in the lip, with a soft edge
        translate([0, 0, cup_h - lip_t - eps]) cylinder(d = lip_in, h = lip_t + 2 * eps);
        translate([0, 0, cup_h - 0.6]) cylinder(d1 = lip_in, d2 = lip_in + 1.2, h = 0.6 + eps);
        // USB-C plug through the side wall, out through the back
        wall_slot(usb_angle, usb_w, usb_h, z_pcb - usb_below, open_back = true);
        // side buttons
        for (a = btn_angles) round_hole(a, btn_d, z_pcb - btn_below);
        // cover screw pilot holes, up into the wall from the back
        for (a = screw_angles)
            rotate([0, 0, a]) translate([r_screw, 0, cover_t - eps]) cylinder(d = screw_hole, h = 6);
    }
}

/* ───────────── the back cover ───────────────────────────────────────────── */
module cover() {
    difference() {
        union() {
            // the disc, with a soft outer edge on the back
            rotate_extrude()
                polygon([[0, 0], [outer_d / 2 - 0.5, 0], [outer_d / 2, 0.5],
                         [outer_d / 2, cover_t], [0, cover_t]]);
            // posts: the PCB edge rests on these; nothing touches the components
            for (a = post_angles)
                rotate([0, 0, a])
                    translate([cav_d / 2 - post_in - 0.3, -post_w / 2, cover_t - eps])
                        cube([post_in, post_w, clr_z + back_stack + eps]);
        }
        // screws: through hole + countersunk head on the back
        for (a = screw_angles)
            rotate([0, 0, a])
                translate([r_screw, 0, 0]) {
                    translate([0, 0, -eps]) cylinder(d = 2.3, h = cover_t + 2 * eps);
                    translate([0, 0, -eps]) cylinder(d1 = screw_head_d, d2 = 2.3, h = 1.0 + eps);
                }
        // the dock's recess, then the magnets in its floor
        translate([0, 0, -eps]) linear_extrude(dock_boss + 0.1 + eps)
            stadium(dock_w + 2 * dock_clr, dock_h + 2 * dock_clr);
        magnet_pockets(dock_boss + 0.1 - eps, magnet_t + 0.2);
        // vents
        for (a = vent_angles)
            rotate([0, 0, a])
                translate([cav_d / 2 - 3.5, 0, -eps])
                    hull() {
                        translate([0,  3, 0]) cylinder(d = 1.6, h = cover_t + 2 * eps);
                        translate([0, -3, 0]) cylinder(d = 1.6, h = cover_t + 2 * eps);
                    }
    }
}

/* ───────────── the stand ────────────────────────────────────────────────── */
// The stalk is a quadratic curve: straight up out of the foot, bending back
// until it runs in the puck's plane for the last stem_run millimetres and
// ends in the dock. A chain of thin rounded slabs square to the curve, hulled
// pairwise; the section widens from stem_w to the dock's width once it is
// hidden behind the puck. Nothing leans more than face_tilt from vertical, so
// it prints foot-down with no support.
stem_p2 = pf(0, 0, -dock_t / 2);
stem_p1 = pf(0, -stem_run, -dock_t / 2);
stem_p0 = [0, stem_p1[1], foot_t - 0.5];
function bez(u)  = (1 - u) * (1 - u) * stem_p0 + 2 * u * (1 - u) * stem_p1 + u * u * stem_p2;
function bezd(u) = 2 * (1 - u) * (stem_p1 - stem_p0) + 2 * u * (stem_p2 - stem_p1);
function sstep(x) = let(t = min(max(x, 0), 1)) t * t * (3 - 2 * t);
function stem_wid(u) = stem_w + (dock_w - stem_w) * sstep((u - 0.3) / 0.55);
function stem_dep(u) = stem_d + (stem_d_top - stem_d) * u;
module stem_slab(u) {
    p = bez(u); d = bezd(u);
    translate(p) rotate([atan2(-d[1], d[2]), 0, 0])
        linear_extrude(0.6, center = true) rounded_rect(stem_wid(u), stem_dep(u), 2.5);
}
module stand() {
    n = 24;
    difference() {
        union() {
            // the foot, chamfered
            rotate_extrude()
                polygon([[0, 0], [foot_d / 2, 0], [foot_d / 2, foot_t - 1.5],
                         [foot_d / 2 - 1.5, foot_t], [0, foot_t]]);
            // the stalk, cut flat where the puck's back sits
            difference() {
                union() {
                    hull() {   // a small flare into the foot
                        translate([0, stem_p0[1], foot_t - 0.3])
                            linear_extrude(0.6, center = true) rounded_rect(stem_w + 6, stem_d + 3, 4);
                        stem_slab(0.12);
                    }
                    for (i = [0 : n - 1]) hull() { stem_slab(i / n); stem_slab((i + 1) / n); }
                }
                puck_frame() translate([0, 0, 50]) cube(100, center = true);
            }
            // the dock, with its boss into the cover's recess
            puck_frame() translate([0, 0, -dock_t])
                linear_extrude(dock_t + dock_boss) stadium(dock_w, dock_h);
        }
        // magnets in the boss face
        puck_frame() translate([0, 0, dock_boss - magnet_t - 0.2]) magnet_pockets(0, magnet_t + 0.2);
        // shallow pocket underneath to locate the VHB disc
        translate([0, 0, -eps]) cylinder(d = foot_d - 6, h = 0.5);
        // nothing below the foot
        translate([0, 0, -50 - eps]) cube([200, 200, 100], center = true);
    }
}

/* ───────────── the wedge (alternative) ──────────────────────────────────── */
module wedge() {
    module on_face() {   // the wedge's tilted top face, origin at the recess centre
        translate([0, -base_l / 2 + 3, base_min_t])
            rotate([tilt_deg, 0, 0])
                translate([0, (base_l - 6) / 2, 0]) children();
    }
    difference() {
        union() {
            difference() {
                hull() {
                    for (sx = [-1, 1], sy = [-1, 1])
                        translate([sx * (base_w / 2 - 3), sy * (base_l / 2 - 3), 0])
                            cylinder(r = 3, h = base_min_t + (sy > 0 ? (base_l - 6) * tan(tilt_deg) : 0));
                }
                on_face() translate([0, 0, -recess_depth]) cylinder(d = outer_d + 0.6, h = 20);
                translate([0, 0, -eps]) linear_extrude(0.6) rounded_rect(base_w - 8, base_l - 8, 2);
            }
            // the same boss as the stand's dock, on the recess floor
            on_face() translate([0, 0, -recess_depth - eps])
                linear_extrude(dock_boss + eps) stadium(dock_w, dock_h);
        }
        on_face() translate([0, 0, -recess_depth + dock_boss - magnet_t - 0.2])
            magnet_pockets(0, magnet_t + 0.2);
    }
}

/* ───────────── test-fit ring ────────────────────────────────────────────── */
module testring() {
    difference() {
        cylinder(d = outer_d, h = 3);
        translate([0, 0, -eps]) cylinder(d = cav_d, h = 4);
        rotate([0, 0, usb_angle]) translate([cav_d / 2 - 1, -usb_w / 2, -eps]) cube([wall + 2, usb_w, 4]);
    }
}

/* ───────────── a mock board, for the assembly views only ────────────────── */
module board_mock() {
    color("darkgreen") translate([0, 0, z_pcb]) cylinder(d = board_d, h = pcb_t);
    color("black", 0.9) translate([0, 0, z_pcb + pcb_t]) cylinder(d = glass_d, h = front_stack);
    color("dimgray") translate([0, 0, z_pcb + pcb_t + front_stack - 0.2]) cylinder(d = active_d, h = 0.3);
}

/* ───────────── output ───────────────────────────────────────────────────── */
if (part == "cup")      cup();
if (part == "cover")    cover();
if (part == "stand")    stand();
if (part == "wedge")    wedge();
if (part == "testring") testring();
if (part == "assembly") {                 // the puck on the dash
    color("#3d3d3d") stand();
    puck_frame() {
        color("#2b2b2b") cup();
        color("#2b2b2b") cover();
        board_mock();
    }
}
if (part == "exploded") {                 // the parts pulled forward off the stand
    color("#3d3d3d") stand();
    puck_frame() {
        color("#2b2b2b") translate([0, 0, 16]) cover();
        color("#2b2b2b") translate([0, 0, 30]) cup();
        translate([0, 0, 30]) board_mock();
    }
}
