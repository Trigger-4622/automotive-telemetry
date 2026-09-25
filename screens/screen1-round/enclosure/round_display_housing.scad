// =============================================================================
//  Dash housing for the round display (screen 1)
//  ESP32-2424S012: ESP32-C3 + 1.28" 240x240 GC9A01 + CST816S touch, USB-C
// -----------------------------------------------------------------------------
//  Three printed parts, plus a test-fit ring:
//
//    cup     - the body. The board drops in glass-first from the front and
//              rests on pads; the USB-C cable leaves through the side wall;
//              the side buttons are reachable through two holes; the back is
//              flat, with pockets for two magnets (or tape it straight on).
//    bezel   - a ring screwed onto the cup rim with three M2 self-tapping
//              screws. Its lip holds the glass down on a 1 mm foam ring.
//    base    - a wedge for the dash: VHB tape underneath, a shallow locating
//              recess and two magnet pockets on top, tilted towards the driver.
//    testring- a 3 mm slice of the cup's cavity with the USB slot: print it
//              first (a minute), drop the board in, adjust the numbers below.
//
//  Render one part:  openscad -o cup.stl -D 'part="cup"' round_display_housing.scad
//  Parts: "cup" "bezel" "base" "testring" "assembly" (exploded, for a look)
//
//  Everything in millimetres. Measure your board before printing (README).
// =============================================================================

part = "assembly";

/* ───────────── the board (measure these, defaults are the 2424S012) ───────── */
board_d      = 38.5;   // PCB diameter (the 38.5 of "38.5 x 37")
glass_d      = 36.0;   // cover glass outer diameter (sits on the PCB)
active_d     = 32.4;   // visible display area; the bezel must clear it
pcb_t        = 1.6;    // PCB thickness
front_stack  = 3.0;    // glass top above the PCB front face (LCD + touch glass)
back_stack   = 5.0;    // tallest thing on the back (JST battery socket, ESP module)

usb_angle    = 270;    // where the USB-C socket points (0 = right, 90 = up, 270 = down)
usb_w        = 13.0;   // slot width for the plug body (right-angle plug: ~12.5)
usb_h        = 7.5;    // slot height
usb_below    = 1.7;    // socket centre below the PCB back face

btn_angles   = [200, 240]; // side buttons (BOOT / RESET), degrees like usb_angle. [] = none
btn_d        = 4.0;    // finger-nail / paperclip hole
btn_below    = 1.5;    // switch plunger centre below the PCB back face

/* ───────────── printing and fit ─────────────────────────────────────────── */
clr_r        = 0.4;    // radial clearance around the PCB
clr_z        = 0.3;    // axial play
wall         = 3.6;    // cup wall (thick enough for the bezel screws)
back_t       = 3.0;    // cup back (holds the magnet pockets)
lip_overlap  = 1.2;    // bezel lip over the glass edge
lip_t        = 1.6;    // bezel lip thickness
lip_proud    = 0.8;    // bezel stands this much above the glass (scratch guard)
gasket_t     = 1.0;    // foam ring between lip and glass (takes up tolerance)
pad_angles   = [30, 150, 330]; // board support pads (keep clear of USB and buttons)
pad_w        = 3.0;    // pad width along the wall
pad_in       = 1.5;    // pad reach into the cavity (under the PCB edge only)
screw_n      = 3;      // bezel screws, M2 x 6 self-tapping
screw_hole   = 1.7;    // pilot hole in the cup
screw_head_d = 4.2;    // countersink in the bezel
screw_angle0 = 60;     // first screw here; the three at 120 deg avoid the USB slot and buttons
vents        = true;   // slots in the cup back
magnet_d     = 10.0;   // two Ø10 x 2 mm disc magnets in cup and base (0 = none)
magnet_t     = 2.0;
magnet_gap   = 16.0;   // centre-to-centre

/* ───────────── the dash base ────────────────────────────────────────────── */
tilt_deg     = 20;     // face tilts up towards the driver by this much
base_w       = 52;     // footprint (a 50 x 50 VHB pad fits underneath)
base_l       = 50;
base_min_t   = 4;      // thinnest edge of the wedge
recess_depth = 1.5;    // locating recess for the cup on top of the wedge

/* ───────────── derived ──────────────────────────────────────────────────── */
$fn = 120;
cav_d   = board_d + 2 * clr_r;           // cavity diameter
outer_d = cav_d + 2 * wall;              // cup outer diameter
z_pads  = back_t + back_stack + clr_z;   // top of the pads = PCB back face
z_glass = z_pads + pcb_t + front_stack;  // glass top
cup_h   = z_glass;                       // rim level with the glass top
bezel_in = glass_d - 2 * lip_overlap;
bezel_h  = gasket_t + lip_t + lip_proud; // ring height above the cup rim
r_screw  = (cav_d + outer_d) / 4;        // screw circle: middle of the wall
eps = 0.01;

assert(bezel_in >= active_d + 0.6, "the bezel opening would cover the display: reduce lip_overlap");
assert(wall - screw_hole >= 1.6, "wall too thin around the bezel screws");

/* ───────────── helpers ──────────────────────────────────────────────────── */
// A rounded slot through the cup wall at an angle: w wide, h tall, centred at
// height z. The flat top is a short bridge (under 10 mm) when the cup prints
// back-down; a 45 deg roof would eat the height a USB-C plug needs.
module wall_slot(ang, w, h, z) {
    r = min(2, h / 2 - 0.1);
    rotate([0, 0, ang])
        translate([cav_d / 2 - 1, 0, z])
            rotate([0, 90, 0])          // extrude outward along +x; 2D x runs down world z
                linear_extrude(wall + 2)
                    offset(r = r) square([h - 2 * r, w - 2 * r], center = true);
}
module round_hole(ang, d, z) {
    rotate([0, 0, ang])
        translate([cav_d / 2 - 1, 0, z])
            rotate([0, 90, 0])
                linear_extrude(wall + 2)
                    hull() {                // teardrop: printable lying down
                        circle(d = d);
                        translate([-d / 2, 0]) square([d / 2, d * 0.15], center = true);
                    }
}
module magnet_pockets(z0, depth) {
    if (magnet_d > 0)
        for (s = [-1, 1])
            translate([s * magnet_gap / 2, 0, z0])
                cylinder(d = magnet_d + 0.4, h = depth + eps);
}

/* ───────────── the cup ──────────────────────────────────────────────────── */
module cup() {
    difference() {
        union() {
            cylinder(d = outer_d, h = cup_h);
        }
        // cavity
        translate([0, 0, back_t]) cylinder(d = cav_d, h = cup_h);
        // USB-C plug through the side wall
        wall_slot(usb_angle, usb_w, usb_h, z_pads - usb_below);
        // side buttons
        for (a = btn_angles) round_hole(a, btn_d, z_pads - btn_below);
        // bezel screw pilot holes
        for (i = [0 : screw_n - 1])
            rotate([0, 0, screw_angle0 + i * 360 / screw_n])
                translate([r_screw, 0, cup_h - 6.5]) cylinder(d = screw_hole, h = 7);
        // magnets in the back
        magnet_pockets(-eps, magnet_t + 0.2);
        // vents
        if (vents)
            for (a = [0 : 60 : 359])
                rotate([0, 0, a + 30])
                    translate([cav_d / 2 - 3.5, 0, -eps])
                        hull() {
                            translate([0,  3, 0]) cylinder(d = 1.6, h = back_t + 2 * eps);
                            translate([0, -3, 0]) cylinder(d = 1.6, h = back_t + 2 * eps);
                        }
    }
    // board support pads: the PCB edge rests on these, components hang between
    for (a = pad_angles)
        rotate([0, 0, a])
            translate([cav_d / 2 - pad_in, -pad_w / 2, back_t - eps])
                cube([pad_in + 0.5, pad_w, z_pads - back_t + eps]);
}

/* ───────────── the bezel ────────────────────────────────────────────────── */
module bezel() {
    difference() {
        cylinder(d = outer_d, h = bezel_h);
        // the opening
        translate([0, 0, -eps]) cylinder(d = bezel_in, h = bezel_h + 2 * eps);
        // recess for the glass edge + foam ring (underside)
        translate([0, 0, -eps]) cylinder(d = glass_d + 1.0, h = gasket_t + eps);
        // screws: through hole + countersunk head
        for (i = [0 : screw_n - 1])
            rotate([0, 0, screw_angle0 + i * 360 / screw_n])
                translate([r_screw, 0, 0]) {
                    translate([0, 0, -eps]) cylinder(d = 2.3, h = bezel_h + 2 * eps);
                    translate([0, 0, bezel_h - 1.4]) cylinder(d1 = 2.3, d2 = screw_head_d, h = 1.4 + eps);
                }
        // a soft edge on the outside
        translate([0, 0, bezel_h]) rotate_extrude() translate([outer_d / 2, 0]) circle(r = 0.8);
    }
}

/* ───────────── the dash base ────────────────────────────────────────────── */
module base() {
    // wedge: flat underneath, top face tilted by tilt_deg about the x axis
    top_t = base_min_t + base_l * tan(tilt_deg);
    difference() {
        hull() {
            for (sx = [-1, 1], sy = [-1, 1])
                translate([sx * (base_w / 2 - 3), sy * (base_l / 2 - 3), 0])
                    cylinder(r = 3, h = base_min_t + (sy > 0 ? (base_l - 6) * tan(tilt_deg) : 0));
        }
        // locating recess + magnet pockets, cut into the tilted face
        translate([0, -base_l / 2 + 3, base_min_t])
            rotate([tilt_deg, 0, 0])
                translate([0, (base_l - 6) / 2, 0]) {
                    translate([0, 0, -recess_depth]) cylinder(d = outer_d + 0.6, h = 20);
                    translate([0, 0, -recess_depth - magnet_t - 0.2]) magnet_pockets(0, magnet_t + 0.2);
                }
        // shallow pocket underneath to locate the VHB pad
        translate([0, 0, -eps]) linear_extrude(0.6)
            offset(r = 2) offset(delta = -2) square([base_w - 8, base_l - 8], center = true);
    }
}

/* ───────────── test-fit ring ────────────────────────────────────────────── */
module testring() {
    difference() {
        cylinder(d = outer_d, h = 3);
        translate([0, 0, -eps]) cylinder(d = cav_d, h = 4);
        rotate([0, 0, usb_angle]) translate([cav_d / 2 - 1, -usb_w / 2, -eps]) cube([wall + 2, usb_w, 4]);
    }
    for (a = pad_angles)
        rotate([0, 0, a]) translate([cav_d / 2 - pad_in, -pad_w / 2, 0]) cube([pad_in + 0.5, pad_w, 1]);
}

/* ───────────── a mock board, for the assembly view only ─────────────────── */
module board_mock() {
    color("darkgreen") translate([0, 0, z_pads]) cylinder(d = board_d, h = pcb_t);
    color("black", 0.9) translate([0, 0, z_pads + pcb_t]) cylinder(d = glass_d, h = front_stack);
    color("dimgray") translate([0, 0, z_pads + pcb_t + front_stack - 0.2]) cylinder(d = active_d, h = 0.3);
    color("silver") rotate([0, 0, usb_angle]) translate([board_d / 2 - 7, -4.5, z_pads - 3.3]) cube([7.5, 9, 3.2]);
}

/* ───────────── output ───────────────────────────────────────────────────── */
if (part == "cup")      cup();
if (part == "bezel")    bezel();
if (part == "base")     base();
if (part == "testring") testring();
if (part == "assembly") {
    color("#2b2b2b") cup();
    board_mock();
    color("#3a3a3a") translate([0, 0, cup_h + 8]) bezel();
    color("#444") translate([0, 0, -45]) base();
}
