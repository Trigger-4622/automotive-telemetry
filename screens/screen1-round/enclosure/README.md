# Dash housing for the round display

A printed puck that holds the ESP32-2424S012 board (ESP32-C3, 1.28" GC9A01,
CST816S touch — the board screen 1 runs) and sticks to the dash on a tilted
base. Everything is one parametric OpenSCAD file, `round_display_housing.scad`;
the STLs here are that file rendered with the default numbers.

| File | What it is | Print |
|---|---|---|
| `testring.stl` | A 3 mm slice of the cup's cavity with the USB slot. **Print this first** (a minute). If the board drops in with a whisker of play and the USB plug clears the slot, the numbers are right. | flat |
| `cup.stl` | The body. Board goes in glass-first from the front, rests on three pads; USB-C leaves through the side wall; two side holes reach the buttons; flat back with two magnet pockets and vents. | back down, no supports |
| `bezel.stl` | Ring screwed to the cup rim with 3 × M2 × 6 self-tapping screws; its lip holds the glass down on a 1 mm foam ring. | front (flat face) down |
| `base.stl` | Wedge for the dash: VHB pad underneath, a locating recess and two magnet pockets on the tilted top. | flat bottom down |

Default sizes: cup Ø46.5 × 12.9 mm, bezel opening Ø33.6 (the display's active
area is Ø32.4), base 52 × 50 mm tilted 20°. About 25 g of filament in all.

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
openscad -o base.stl     -D 'part="base"'     round_display_housing.scad
openscad -o testring.stl -D 'part="testring"' round_display_housing.scad
```

`part="assembly"` shows an exploded view in the OpenSCAD window.

## Printing

- **Not PLA.** A dashboard in the sun passes 70 °C and PLA slumps. PETG is
  the minimum; ASA or ABS is better. Matte black or dark grey, so the puck
  does not reflect in the windscreen.
- 0.2 mm layers, 3 perimeters, 20–30 % infill, no supports in the
  orientations above. The USB slot and button holes have pointed roofs so
  they print lying down.
- The cup's back is the bed face: the magnet pockets and vents print as
  holes in the first layers, which works on any printer.

## Assembly

1. Two Ø10 × 2 mm disc magnets in the cup's back pockets, two in the base,
   epoxy or thick CA. **Check the polarity before the glue sets**: the pair
   in the base must attract the pair in the cup. N42SH or ferrite if your
   summers are hot; ordinary N35 loses strength above about 80 °C.
2. Stick a 1 mm foam ring (self-adhesive EVA, or three dabs of hot glue) on
   the glass edge, outside the display area. It takes up the tolerance in
   the stack height so the bezel cannot rattle or crush the glass.
3. Board in glass-first, USB-C socket lined up with the slot. It rests on
   the three pads at the cavity wall; nothing touches the components.
4. Bezel on, three M2 × 6 self-tapping screws. Snug, not tight.
5. Base on the dash: clean the spot with isopropyl alcohol, a 50 × 50 mm
   piece of 3M VHB 5952 (or 4991) in the shallow pocket underneath, press
   hard for a minute, leave it 24 hours before hanging the puck on it.
   On a strongly curved dash, warm the tape with a hairdryer and use two
   narrower strips instead of one square.

The puck lifts off the base for flashing. The cable is a right-angle USB-C
lead; it leaves through the bottom of the puck and runs down the dash.

## What the defaults assume

- The USB-C socket points straight out of the PCB edge (side entry). If on
  your board it is recessed or on the back face, widen `usb_h` or move the
  slot down with `usb_below`.
- The two buttons are side-actuated tactile switches at the PCB edge. If
  they are on the back face instead, set `btn_angles = []` and press them
  through a vent slot with a pin, or add pockets in `cup()`.
- The board's radio antenna is on the module itself; 3.6 mm of PETG in front
  of it costs nothing noticeable. If yours has an IPEX pigtail antenna, tuck
  it into the gap between the components and the back wall before closing.
- The bezel lip covers 1.2 mm of glass edge and stays 0.6 mm clear of the
  active area. Touch works through the opening as before.
