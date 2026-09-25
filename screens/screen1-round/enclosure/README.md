# Dash housing for the round display

A printed puck that holds the ESP32-2424S012 board (ESP32-C3, 1.28" GC9A01,
CST816S touch — the board screen 1 runs), floating on a slim stalk on the
dash top like a gauge pod. Everything is one parametric OpenSCAD file,
`round_display_housing.scad`; the STLs here are that file rendered with the
default numbers. `preview/` has renders of every part and of the assembly.

| File | What it is | Print |
|---|---|---|
| `testring.stl` | A 3 mm slice of the cup's cavity with the USB slot. **Print this first** (a minute). If the board drops in with a whisker of play and the USB plug clears the slot, the numbers are right. | flat |
| `cup.stl` | The puck: one piece, a 45° bevel round the face, no bezel ring, nothing on the front but the glass. The board goes in **from the back**, glass first, against the front lip and a foam ring. The USB-C plug leaves through a slot in the bottom of the wall, the two side buttons through two holes. | face down, no supports |
| `cover.stl` | The flat back. Three posts press the board forward onto the lip; three M2 countersunk screws into the wall; a shallow recess the stand's dock drops into, with two magnet pockets in its floor; vents in the upper half. | back down, no supports |
| `stand.stl` | The dash mount. A Ø40 foot with a VHB pocket underneath, a stalk that rises and bends back to lie in the puck's plane, and at its top a small dock with two magnets and a raised boss. The puck sits on the dock leaning back 12°; from the driver's seat nothing shows behind it. | foot down, no supports |
| `wedge.stl` | The alternative, lying down: a 20° wedge with a locating recess and the same dock boss, for a dash you look down at. | flat |

Default sizes: puck Ø48.5 × 16.1 mm. Standing, its centre is 36 mm above
the dash, its top 63 mm; the bottom edge is 9 mm clear of the foot for the
plug. Foot Ø40 × 3.5 mm, stalk 14 × 8 mm at the foot. The USB-C plug hangs
from the bottom of the puck with a **right-angle lead** whose cable goes
sideways past the stalk and back along the dash. About 30 g of filament in
all.

## Measure before you print the cup

The defaults come from the board's published outline (38.5 × 37 mm, Ø32.4
active area) and typical 1.28" display stacks — not from your board. Five
caliper measurements pin it down; each is a variable at the top of the file:

| Variable | Measure | Default |
|---|---|---|
| `board_d` | PCB diameter (the widest point) | 38.5 |
| `glass_d` | cover glass outer diameter | 36.0 |
| `front_stack` | glass top above the PCB's front face | 3.0 |
| `back_stack` | tallest part on the back (usually the battery socket or the radio module) | 5.0 |
| `usb_angle`, `btn_angles` | where the USB-C socket and the buttons sit. Hold the board the way the gauges read (the firmware does not rotate the picture, so the board's own "up" is up); angles run counter-clockwise from 3 o'clock, so 270° is straight down | 270, [200, 240] |
| `post_angles` | where the cover's three posts land. They press on the back face of the PCB, 1.4 mm in from its edge, so put them where that edge is bare | [30, 150, 330] |

If the board has flat edges, a round cavity still fits it (the flats just
leave a gap). Change the numbers, re-run the test ring, then export:

```bash
openscad -o cup.stl      -D 'part="cup"'      round_display_housing.scad
openscad -o cover.stl    -D 'part="cover"'    round_display_housing.scad
openscad -o stand.stl    -D 'part="stand"'    round_display_housing.scad
openscad -o wedge.stl    -D 'part="wedge"'    round_display_housing.scad
openscad -o testring.stl -D 'part="testring"' round_display_housing.scad
```

`part="assembly"` shows the puck on the stand; `part="exploded"` the parts
pulled forward off it. The stand's own knobs: `face_tilt` (how far it leans
back, 12°), `plug_gap` (clear height under the puck for the plug, 9 mm),
`stem_w`/`stem_d` (the stalk's section), `foot_d`. The puck's: `bevel` (the
face bevel, 3.5 mm) and `wall` (4.6 mm; the cover screws bite into it, so
not thinner).

## Printing

- **Not PLA.** A dashboard in the sun passes 70 °C and PLA slumps. PETG is
  the minimum; ASA or ABS is better. Matte black or dark grey, so the puck
  does not reflect in the windscreen.
- 0.2 mm layers, 3 perimeters, 20–30 % infill, no supports in the
  orientations above:
  - Cup face down: the bevel is a 45° overhang, the cavity opens upward, the
    USB slot runs out through the top of the print and the screw pilot holes
    are blind holes from the top, so nothing bridges.
  - Cover back down: the posts stand up; the dock recess, countersinks,
    magnet pockets and vents are holes in the first layers.
  - Stand foot down: the stalk never leans more than the 12° face tilt and
    widens gently behind the puck; the dock's boss is a 0.7 mm step.
- Screws: 3 × M2 × 8 countersunk self-tapping (ST2.2 × 8 flat head). The
  pilot holes are Ø1.8; run a screw in and out once before the board is in.

## Assembly

1. Two Ø10 × 2 mm disc magnets in the pockets at the bottom of the cover's
   recess, two in the dock's face, epoxy or thick CA, flush. **Check the
   polarity before the glue sets**: the pair in the dock must attract the
   pair in the cover. N42SH or ferrite if your summers are hot; ordinary N35
   loses strength above about 80 °C.
2. Stick a 1 mm foam ring (self-adhesive EVA, or three dabs of hot glue) on
   the glass edge, outside the display area. It takes up the tolerance in
   the stack height so the lip cannot rattle or crush the glass.
3. Board in from the back, glass first, USB-C socket lined up with the slot
   and the buttons with their holes. The glass lands on the foam against the
   lip.
4. Cover on. Its three screw holes are unevenly spaced, so it only lines up
   one way: with the USB slot at the bottom the dock recess is level. The
   posts land on the bare PCB edge; nothing touches the components. Three
   screws, snug, not tight.
5. Stand on the dash: clean the spot with isopropyl alcohol, a Ø34 disc (or
   a 30 × 30 mm square) of 3M VHB 5952 in the pocket under the foot, press
   hard for a minute, leave it 24 hours before hanging the puck on it. On a
   curved dash top, warm the tape with a hairdryer.
6. Puck onto the dock: the boss drops into the recess in the cover and the
   magnets pull it home. The boss takes the sideways loads (touching the
   screen, bumps), the magnets only have to hold it on. Right-angle USB-C
   plug in from below, cable out to the side past the stalk and back along
   the dash.

The puck lifts straight off the dock for flashing.

## What the defaults assume

- The USB-C socket points straight out of the PCB edge (side entry) and
  sits at the bottom when the picture is upright. If yours is elsewhere,
  set `usb_angle`; keep `screw_angles` and `post_angles` clear of it.
- The two buttons are side-actuated tactile switches at the PCB edge. If
  they are on the back face instead, set `btn_angles = []` and press them
  through a vent slot with a pin, or add pockets in `cover()`.
- The board's radio antenna is on the module itself; 3.6 mm of PETG behind
  it costs nothing noticeable. If yours has an IPEX pigtail antenna, tuck
  it into the gap between the components and the cover before closing.
- The lip covers 1.2 mm of glass edge and stays 0.6 mm clear of the active
  area. Touch works through the opening as before.
