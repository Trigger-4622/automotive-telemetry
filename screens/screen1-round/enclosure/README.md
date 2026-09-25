# Dash housing for the round display

A printed puck that holds the ESP32-2424S012 board (ESP32-C3, 1.28" GC9A01,
CST816S touch — the board screen 1 runs), standing upright on the dash top
like a gauge pod. Everything is one parametric OpenSCAD file,
`round_display_housing.scad`; the STLs here are that file rendered with the
default numbers.

| File | What it is | Print |
|---|---|---|
| `testring.stl` | A 3 mm slice of the cup's cavity with the USB slot. **Print this first** (a minute). If the board drops in with a whisker of play and the USB plug clears the slot, the numbers are right. | flat |
| `cup.stl` | The puck's body. Board goes in glass-first from the front, rests on three pads; USB-C leaves through the side wall; two side holes reach the buttons; flat back with two magnet pockets and vents. | back down, no supports |
| `bezel.stl` | Ring screwed to the cup rim with 3 × M2 × 6 self-tapping screws; its lip holds the glass down on a 1 mm foam ring. | front (flat face) down |
| `stand.stl` | The dash mount. The puck stands leaning back 12°, held by two magnets on a plate behind it and resting its bottom edge on two small lips; a neck down to a flat foot with a VHB pad underneath. | foot down, no supports |
| `wedge.stl` | The alternative, lying down: a 20° wedge with a locating recess and magnet pockets, for a dash you look down at. | flat |

Default sizes: puck Ø46.5 × 12.9 mm; standing, its centre is 36 mm above
the dash and its top 58 mm; foot 52 × 46 mm. The USB-C plug hangs from the
bottom of the puck, 9 mm clear of the foot, with a **right-angle lead** whose
cable goes sideways round the neck and back along the dash. About 30 g of
filament in all.

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
| `usb_angle`, `btn_angles` | hold the board the way the gauges read (the firmware does not rotate the picture, so the board's own "up" is up); angles run counter-clockwise from 3 o'clock, so 270° is straight down. Where the USB-C socket and the buttons sit | 270, [200, 240] |

If the board has flat edges, a round cavity still fits it (the flats just
leave a gap). Change the numbers, re-run the test ring, then export:

```bash
openscad -o cup.stl      -D 'part="cup"'      round_display_housing.scad
openscad -o bezel.stl    -D 'part="bezel"'    round_display_housing.scad
openscad -o stand.stl    -D 'part="stand"'    round_display_housing.scad
openscad -o testring.stl -D 'part="testring"' round_display_housing.scad
```

`part="assembly"` shows the puck standing on the dash; `part="exploded"`
the parts apart. The stand's own knobs: `face_tilt` (how far it leans back,
12°), `plug_gap` (clear height under the puck for the plug, 9 mm),
`lip_angles` (where the two lips under the puck are — keep them away from
`usb_angle`), `foot_w`/`foot_l`.

## Printing

- **Not PLA.** A dashboard in the sun passes 70 °C and PLA slumps. PETG is
  the minimum; ASA or ABS is better. Matte black or dark grey, so the puck
  does not reflect in the windscreen.
- 0.2 mm layers, 3 perimeters, 20–30 % infill, no supports in the
  orientations above. The stand's neck leans back with the puck, so nothing
  on it overhangs more than the 12° tilt; the two lips are 4 mm cantilevers,
  which print fine.
- The cup's back is the bed face: the magnet pockets and vents print as
  holes in the first layers, which works on any printer.

## Assembly

1. Two Ø10 × 2 mm disc magnets in the cup's back pockets, two in the
   stand's plate, epoxy or thick CA. **Check the polarity before the glue
   sets**: the pair in the plate must attract the pair in the cup. N42SH or
   ferrite if your summers are hot; ordinary N35 loses strength above about
   80 °C.
2. Stick a 1 mm foam ring (self-adhesive EVA, or three dabs of hot glue) on
   the glass edge, outside the display area. It takes up the tolerance in
   the stack height so the bezel cannot rattle or crush the glass.
3. Board in glass-first, USB-C socket lined up with the slot. It rests on
   the three pads at the cavity wall; nothing touches the components.
4. Bezel on, three M2 × 6 self-tapping screws. Snug, not tight.
5. Stand on the dash: clean the spot with isopropyl alcohol, a 45 × 40 mm
   piece of 3M VHB 5952 (or 4991) in the shallow pocket under the foot,
   press hard for a minute, leave it 24 hours before hanging the puck on
   it. On a curved dash top, warm the tape with a hairdryer and use two
   narrower strips instead of one square.
6. Puck onto the plate: the magnets pull it home and its bottom edge sits
   on the two lips. Right-angle USB-C plug in from below, cable out to the
   side and back along the dash.

The puck lifts straight off the stand for flashing.

## What the defaults assume

- The USB-C socket points straight out of the PCB edge (side entry) and
  sits at the bottom when the picture is upright. If yours is elsewhere,
  set `usb_angle`; if it ends up near a lip, move `lip_angles`.
- The two buttons are side-actuated tactile switches at the PCB edge. If
  they are on the back face instead, set `btn_angles = []` and press them
  through a vent slot with a pin, or add pockets in `cup()`.
- The board's radio antenna is on the module itself; 3.6 mm of PETG in front
  of it costs nothing noticeable. If yours has an IPEX pigtail antenna, tuck
  it into the gap between the components and the back wall before closing.
- The bezel lip covers 1.2 mm of glass edge and stays 0.6 mm clear of the
  active area. Touch works through the opening as before.
