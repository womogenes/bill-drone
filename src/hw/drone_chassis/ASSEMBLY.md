## Flush leg crossbar — 2026-10-13

Leg crossbars now align with the existing frame bosses: top Z −2 (raised 2 mm), side edges X ±29 (1 mm wider each side), matching square top corners. Screw centers, 8 mm mounting thickness, roots and foot tie are preserved. Only the two leg bodies changed; live regeneration and collision checks passed. Solid PLA equivalent is 46.31 g per frame, not sliced usage.

Both `stl/reinforced_landing_legs_pair.3mf` and `stl/legs_and_motor_cradles.3mf` are refreshed. The combined project now includes current M2 nut-recess cradles with organic tree supports. Both projects are unsliced.

## Horn screw direction and captive nuts — 2026-10-13

The four M2 horn screws now enter from the servo side and thread into attachment-side hex nuts. Recesses are 4.5 mm across flats and 2.2 mm deep; clearance holes remain 2.4 mm. Center M3 socket-head screw direction and retaining shoulder are unchanged. Fit the outer screws to the horn before mounting it to the servo when rear access is obstructed. Install the center screw before closing the opposite journal/retainer.

Use `../end_attachment/stl/motor_cradles_m2_nut_recesses_pair.3mf` for two attachments only. Organic tree supports enabled, build-plate-only. Unsliced; inspect pocket support cleanup and actual print fit. The earlier combined legs project has the previous cradle geometry.

## Landing root revision — 2026-10-13

Following the photographed root fracture, the unprinted replacement frames now have extended tapered roots (inner junctions X ±8, Z −24) and 8 mm inside fillets. The bottom tie, 8 mm frame thickness, mounting holes, and rear electronics-deck interface remain unchanged. Only the two leg parts changed in the live model; regeneration and assembly interference checks passed.

Use `stl/reinforced_landing_legs_pair.3mf`: two frames flat, 0.20 mm layers, six walls, 35% gyroid, five top/bottom layers, 6 mm brim, supports off. The project is unsliced and has not been checked in the slicer GUI. Roots do not have separate solid-infill modifiers. Solid-PLA equivalent is 44.71 g each (+5.17 g versus the previous tied design), not a sliced mass estimate. Test physically with propellers removed before powered tests; geometry checks do not establish landing-impact strength.

# Drone assembly

Updated 2026-10-12. CAD source snapshots: `chassis.fs` and
`../end_attachment/servo.fs`. Live document is Drone, assembly tab Drone assembly:
https://cad.onshape.com/documents/d7e760af3cc2197cafa5a123/w/0a0eede3b0e261a3b55f7eb1/e/d6979667eed205f63b0dfaf2

The assembly retains two linked instances of the servo-mount Part Studio.
Re-export modified parts before slicing; old STLs do not update themselves.

## M2 horn interface and reinforced landing frames — 2026-10-13

Reprint only the two Motor cradles and two landing frames for this revision.
The servo frames, pivot supports, electronics deck (including rear braces),
side frames and existing landing-frame attachment holes remain unchanged.

The cradle now has four 2.4 mm clearance holes on the existing 14 mm cardinal
horn pattern, with 4.4 mm head counterbores for M2 socket screws. Do not enlarge
the horn to M3. The M2 head seats leave 3.8 mm of printed material to the horn
face. If the horn is threaded M2, an M2 x 6 screw projects 2.2 mm into it; check
actual usable thread depth and servo-case clearance. If it uses through holes
and nuts, M2 x 8 leaves about 1.9 mm beyond the measured 2.3 mm horn for a nut.
Choose the applicable arrangement for the real horn, not both.

The center hole is 3.5 mm with a 6 mm counterbore for an M3 socket head. A 2 mm
thick shoulder between the head seat and hub relief retains the head. The
counterbore is 2 mm deep; a standard 3 mm tall socket head protrudes 1 mm into
the open cradle. Select center-screw length from actual servo thread engagement;
the new seat is 4 mm outward from the horn contact face. Do not assume the
original center screw remains long enough. Install the center screw before the
journal nut, pivot retainer and support obstruct the axial driver path.

Landing-frame root width at the crossbar is increased from roughly 8 to 16 mm
by moving the inner root corners from x=+/-20 to +/-12. Root fillets increase
from 3 to 5 mm. A 7 mm tall, 8 mm deep ground-level tie joins the feet of each
pair, resisting splay. Overall footprint, height, mounting-hole centers and
front/rear interchangeability are retained. This does not certify crash strength.
Test progressively with the assembled vehicle before another powered test.

Replacement projects:
- stl/reinforced_landing_legs_pair.3mf: two identical frames, flat, supports off.
- ../end_attachment/stl/motor_cradles_m2_pair.3mf: two cradles, motor-plate face
  down, normal automatic build-plate supports enabled for overhangs.
Both: 0.20 mm layers, 4 walls, 25% gyroid, 5 top/bottom layers and 6 mm brims.
These are unsliced projects; inspect the sliced preview before printing.
Solid-PLA equivalents are 16.84 g per cradle and 39.54 g per landing frame;
actual sliced use will differ. The leg changes add 13.38 g per frame by solid
volume. CAD regeneration, collision checks and all 12 horn hardware/access
checks passed. The central head-retention check intentionally confirms that a
head pushed through its shoulder collides with the retaining material.

## Direct-mount electronics platform and camera fixture (current)

The current Electronics deck replaces the IMU tube cap and mounts directly to
its original beam-bridge interface. Do not stack it on the old cap. The bridge,
side frames and landing legs are unchanged. Historical cap/deck/IMU instructions
below are superseded by this section.

The 80 x 129.5 mm platform now includes two integral rear-leg mounting tabs
and triangular ribs. Its overall height is 31.3 mm; the original platform
underside remains at z=5.3 mm between the tabs. Two M3 clamp
centers at x=0, y=+/-10 mm and head seats at z=9.3 mm retain the original M3 x 20
screws and captive bridge nuts. Two rear tabs share the existing landing-leg
holes at x=+/-24, z=-12 and their captive nuts. The 5.2 mm tab thickness
keeps M3 x 20 tips 0.2 mm short of the existing frame bore end. Only the deck needs reprinting;
the existing rear leg frame, side frames, beam bridge and camera parts remain
usable. No separate IMU bosses or deck-to-cap screws.
A 90 x 70 mm perfboard mounts with 90 mm front-to-back and 70 mm across the beam, using four M2
clearance holes on a 66 x 86 mm pattern (x=+/-33, y=-8/78). The 2 mm corner-hole
inset is retained as an assumption from the previous board. Its front edge stays
at y=-10 to clear the camera, so its center moves 15 mm rearward versus the
60 x 40 board. Recheck balance after fitting the larger board and electronics. Four 5 mm standoffs provide side-entry
4.5 mm across-flats, 2 mm high M2 nut pockets. Board thickness is assumed 1.6 mm;
check M2 x 8 screw engagement against the actual board. The regulator and IMU
layout on that perfboard are not specified or mechanically verified.

The camera stands vertically with its antenna end DOWN. Only the two upper
corners are clamped, using the measured 2.5 x 2.5 mm component-free regions on
both PCB faces. Fixed front contact patches are at most 2.2 x 2.0 mm and the
rear tab pads are 2.0 x 2.0 mm. Upper/outer edge stops allow 0.3 mm clearance.
A single removable rear retaining bar uses two M2 x 10 screws and side-loaded
M2 nuts. The screws sit above/outside the PCB and never touch it. The CAD shows
the bar loose; it closes 1.1 mm to the nominal 1.2 mm board thickness. Tighten
gently and evenly until both pads contact the clear PCB corners.

Exposed post and PCB-bar top corners have 1 mm chamfers; foot corners use
1.2 mm at the rear and 0.7 mm at the front. Lens-keeper outline corners use
0.8 mm chamfers. Outer post front edges have a 0.5 mm edge break. Lens contact
surfaces, PCB pads, mounting holes and nut pockets retain their dimensions.

The camera module is also supported by a shallow lower barrel fork and removable
upper keeper. The 8.9 mm nominal split opening accommodates the 8 mm fixed barrel
and +/-0.3 mm lens-center measurement error. It is deliberately loose before
clamping. Two M2 x 10 screws pull the keeper down until it gently holds the barrel;
do not tighten to a prescribed torque or force the split closed. Nominal CAD
contact occurs after about 0.9 mm keeper travel, with 0.1 mm split remaining.
The loose assembly shown in CAD is not the final tightened position.

The barrel contact band is 2.8 mm deep (y=-47.7 to -44.9), behind the nominal
lens tip and 0.3 mm ahead of the square camera housing. The clamp grips the fixed
barrel, never the glass. Screw lugs sit beside the housing and clear the modeled
PCB, but unmodeled front components and actual connectors need a physical fit
check. Stop if clamping shifts focus, strains the ribbon, or presses on components.
The antenna end remains open. Tape wires to the deck for strain relief.

Assembly order:
1. Load four M2 nuts into the platform standoffs and all four M2 nuts into the
   camera frame. The two new lens-clamp nuts slide in from behind, before the
   camera is installed; the PCB-retainer nuts enter from the outside edges.
2. Fit the camera frame with four M3 x 10 screws from below and four M3 nuts
   above the feet, before placing the platform on the bridge.
3. Remove both rear landing-leg screws, supporting the frame while they are
   out. Lower the new deck into place; start the existing two M3 x 20 central
   clamp screws loosely. Install these before fitting the perfboard.
4. Secure the new deck tabs through the rear landing legs using two M3 x 20
   screws in place of the previous M3 x 16 screws. Reuse the existing captive
   nuts. Start all deck/leg screws loosely, then tighten evenly without pulling a misaligned part
   into place. The two front landing-leg screws stay unchanged.
5. Attach the perfboard with four M2 x 8 screws.
6. With both camera retainers removed, place the CAM from behind, antenna down,
   upper corners against their pads, and fixed barrel in the lower fork.
7. Fit the PCB retaining bar using two M2 x 10 screws; tighten gently/evenly.
8. Fit the lens keeper using two additional M2 x 10 screws from above. Alternate
   small adjustments until the camera module is steady. Do not crush the barrel.

Print camera_mount_with_lens_keeper.3mf: THREE pieces, frame and lens keeper
front-face down, PCB retaining bar rear-face down. Historical filenames
camera_mount_simple.3mf and camera_fixture_with_tabs.3mf are refreshed with the
same three pieces. Do not print obsolete camera_tab_left/right files.
The camera foot hole pattern and beam-bridge interface are unchanged.

The braced electronics_deck.3mf is now inverted: perfboard standoff ends face
the bed, braces point upward. Normal automatic supports from the build plate
are enabled beneath the elevated platform. Do not use the old bottom-down,
supports-off deck orientation. The combined electronics_deck_and_camera.3mf is
also refreshed, with supports enabled for the deck and disabled on camera parts.
Deck solid-PLA equivalent is about 29.89 g, excluding support/brim material.
Settings: 0.20 mm layers, 3 walls, 15% gyroid, 4 top/bottom layers, 6 mm outer
brim, supports disabled. Small transverse screw holes and nut-pocket roofs bridge.
Projects are unsliced and have not been GUI-verified. No print was launched.

The geometry/access checks and solid-volume mass estimates are recorded in
stl/camera_fixture.txt. These checks do not establish physical grip or vibration
resistance; test with the real board and camera before operation.

## Interchangeable side frames

Exposed joint-block, landing-mount and hatch-mount outline corners have 1 mm
chamfers through the frame thickness. Bridge interfaces and symmetry are retained.

Both side frames are now symmetric front-to-back. Export either updated side
frame and print two copies; rotate one 180 degrees about the vertical axis for
the opposite side. Nut entries face outward. The hanger shoulders are symmetric,
and hatch locating sockets are provided at both y = -9 and +9 mm. The existing
hatch uses one socket on each frame; the extra sockets permit interchangeability.

The already-printed beam bridge remains reusable. The current direct-mount deck
replaces the old IMU tube cap; retain that cap only for the previous deck.
Bridge tongue/socket dimensions, joint screw positions, and landing mounts are
unchanged. Front and rear landing legs are already identical; print two copies.
CAD union checks confirm front/back reflection and left/right interchange after
rotation to within 0.000001 mm³. Battery release (27 positions), frame insertion
(14 positions), and nut/driver access (18 checks) still pass. Physical fit and
load testing remain necessary.

## Modular chassis revision

The chassis is now three printed modules: **Beam bridge**, **Left side frame**,
and **Right side frame**. Each side frame includes its battery hanger,
landing rails, braces and hatch attachment. Four interlocking tongues join the
side frames to the bridge. Their lower faces seat on pocket floors; four
M3 × 12 screws clamp the tongues down and prevent withdrawal. The joint does
not depend on screw-shaft friction alone.

Tongues are 8 mm wide and 6.7 mm thick, with 8.5 mm engagement into each side
frame. Sockets provide 0.3 mm clearance on each side and at the end, and 0.3 mm
above the tongue. The underside seats directly on the socket floor. Tongue
nose corners have 0.8 mm lead-ins. Joint nut pockets are 6.2 mm across flats
and 3.2 mm deep, with a 2.5 mm supporting wall beneath the tongue seat.
The M3 × 12 head seats at z = -5.3 mm; its tip is at -17.3 mm, beyond the
nominal nut bottom at -16.9 mm. No washers are assumed in these recessed seats.

**Print orientation:** lay each side frame on its broad, flat INNER face
(the face that faces the battery); bosses project upward. Lay the beam bridge
on its broad bottom face with the carbon-tube channel up. The side-frame rails
and diagonal braces then lie in the bed plane. This avoids the tall upright
rail-support arrangement. The small socket roofs and transverse screw holes
still need sliced-preview inspection for bridging; no slicer verification or
print-time claim has been made. Use 0.20 mm layers, five walls and 25% gyroid
as a starting point. The joints and narrow braces will be predominantly walls.

The three modules total 47.142 cm³ of CAD solid volume: 5.6% more than the
last one-piece lightweight body, but 43.4% less than the original 83.245 cm³
cage. The added material is concentrated at the modular joints. Actual filament
and time require slicing. The battery hatch, centred battery, electronics
mounting arrangement and landing-foot positions are unchanged.

For the original modular conversion, re-export all three modules. For the
subsequent symmetry revision, only re-export the side frames. The old one-piece chassis cannot substitute for
one of them. This revision adds four M3 × 12 screws and four standard M3 nuts.
The obsolete monolithic assembly instance is preserved but suppressed; the
three new instances replace it in the assembly group.

## Battery changes — no straps

The printed **Battery hatch** supports and locates the pack. It uses two
**M3 × 16 socket-head screws** (ordinary ~5.5 mm diameter heads) and two standard
M3 nuts. Do not add washers to these two screws: their heads must pass through
the 6.6 mm release holes. There are now nine printed parts in the central chassis set, including the hatch.

Slide the nuts sideways into the two low chassis bosses. Each nut sits above a
6 mm solid floor, so a bottom-up screw pulls it into supporting material.
Install the screws through the hatch's SMALL keyhole ends. The two locating pegs
must seat in the matching chassis recesses; then tighten snugly. The battery
rests on the hatch's raised rails, with printed stops at both ends.

For a swap:

1. Disconnect power and support the hatch and battery. Lift/support the drone
   to leave hand clearance underneath.
2. Loosen both screws approximately **four turns** (2 mm for standard M3 × 0.5
   threads). Do not unscrew them completely.
3. Lower the hatch 2 mm to disengage its locating pegs, then slide it **7 mm
   toward the camera**. The enlarged keyholes now align with the screw heads.
4. Lower hatch and battery together. Put the replacement on the hatch and
   reverse the motion, checking both locating pegs seat before tightening.

The screws and nuts remain in the chassis. Neither the electronics deck nor
landing gear is removed. This is a quick mechanical swap, not electrically
uninterrupted hot swapping. Keep the connector accessible outside the bay and
leave enough lead slack for the short release motion before lowering the pack.

The end stops now locate the battery centrally with about 1 mm total length
clearance, rather than relying on straps to hold an adjustable pack position.
The open space between the landing frames clears the release movement. For final
fore/aft balance, adjust the taped electronics placement and verify the real
centre of mass with all wiring installed. Side-to-side battery placement is
centred and repeatable. The design assumes the measured 36 × 35 × 74 mm pack;
it is not a universal adjustable battery holder.

## Where the nut pockets are

| Joint | Where to look | Nut entry |
|---|---|---|
| Electronics deck | Two diagonal corners of the central IMU/tube cap, at x = ±26 mm and y = ±10 mm; the other corners have locating pegs | Sideways through the narrow horizontal slots |
| Landing frames | Small mounting blocks behind the front/rear frames, beside the battery bay | Sideways from the outside |
| Central tube clamp | Roof of the empty battery bay, beneath the carbon tube | Upward into two hexagonal recesses; remove battery/hatch for access |
| Battery hatch | The two low outside bosses, one on each side of the bay | Sideways into slots above the bottom screw holes |

The top-visible round holes are screw passages. The nut pockets are often
hidden behind a mating part or reached through a rectangular side opening.
Camera and IMU screws use PLA pilot holes rather than nuts.

## Screw list

Lengths below are under-head lengths for ordinary M3 socket-head screws and
standard M3 hex nuts (~5.5 mm across flats, ~2.4 mm thick). Nut pockets are
6.2 mm across flats. Additional washers change the available engagement.

| Joint | Quantity | Screw | Insertion and retention |
|---|---:|---|---|
| Bridge to side frames | 4 | M3 × 12 | Down through the large access holes onto the tongue seats; side-loaded nuts bear against 2.5 mm supporting walls. Assemble before fitting the cap. |
| Battery hatch | 2 | M3 × 16 | Up from the keyhole hatch; side-loaded nuts bear downward on 6 mm solid floors. No washers. |
| Central tube clamp | 2 | M3 × 20 | Down through the IMU cap; nuts enter from below the empty battery opening and bear upward against a solid roof. |
| Electronics deck | 2 | M3 × 10 | Down from the deck into the cap; nuts enter sideways and bear against a 2.5 mm roof. |
| Landing frames | 4 total | M3 × 16 | From the outer face of each frame toward the chassis; nuts enter sideways and bear against a 3 mm wall. Screw tips remain outside the battery envelope. |
| Camera cap | 2 | M3 × 8 | Into 2.6 mm PLA pilots; no nuts. Tighten gently. |
| IMU | 2 | M3 × 6 | Into 2.6 mm PLA pilots, assuming ~1.6 mm PCB. Check engagement if board or spacer thickness differs. |
| Servo-mount tube saddle | 4 per motor | M3 × 20 | Up from the saddle; nuts load from the TOP of the frame and bear downward on solid pocket floors. |
| Servo bare ears | 4 per servo | M3 × 12 | Through ~2.5 mm bare ears and 6 mm frame, with nuts on the inside. This rigid mount does not assume rubber grommets/sleeves remain installed. |
| Horn to cradle | 4 per motor | M3 × 10 | Heads in cradle's shallow inside recesses; nuts behind the measured 2.3 mm horn. The four 14 mm-spaced cardinal horn holes must clear the M3 screws. |
| Metal motor X bracket to cradle | 4 per motor | M3 × 8 | Down through the modeled 1.5 mm bracket and cradle; nuts in underside pockets. Use the motor's correct original fasteners for bracket-to-motor attachment. |
| Removable pivot support foot | 2 per motor | M3 × 10 | Down through the slots, without added washers; bottom-loaded nuts bear upward against solid roofs. Longer screws may be needed with washers. |
| Pivot retainer | 1 per motor | M3 × 20 | From the outside, through the retainer and journal, into a nut loaded from the cradle's inner face. |
| Servo horn centre | 1 per servo | Supplied servo screw | Retain its original thread and length. Do not substitute an arbitrary M3 screw. |

M3 × 12 horn bolts can reach the servo case in the measured configuration;
use M3 × 10. All other lengths are based on nominal CAD and the stated hardware
thicknesses. Confirm screw tips and nut engagement during the first assembly.

## Whole-drone fastener totals

Includes the central chassis and both complete servo/motor mounts; no spares.

| M3 socket-head screw | Quantity |
|---|---:|
| M3 × 6 | 2 |
| M3 × 8 | 10 |
| M3 × 10 | 14 |
| M3 × 12 | 12 |
| M3 × 16 | 6 |
| M3 × 20 | 12 |
| **Total** | **56** |

Also: **52 standard M3 hex nuts**, plus the two supplied servo-centre screws
and the motors' original bracket-to-motor screws and propeller hardware.
Camera and IMU screws account for the four M3 screws without nuts.
Use standard M3 × 0.5 threads. Do not substitute taller nyloc nuts in the
printed pockets. The 6.2 mm pocket width provides 0.7 mm total across-flats
clearance around a nominal 5.5 mm nut. The shallow servo pivot-foot, motor-bracket,
and journal pockets now have 3.1 mm usable depth for a nominal 2.4 mm nut. Side-loaded chassis pockets have at least 3.0 mm
axial clearance (3.2 mm slot depth); the central tube pockets are deeper.

## Assembly order

1. Clean only print burrs and test-fit standard nuts. Do not hammer nuts into
   pockets or force bolts to compensate for misaligned parts. The intended
   nut-loading side is not necessarily the screw-head side.
2. Before fitting the cap or landing frames, load the four module-joint nuts
   sideways into the side frames. Slide each side frame inward onto the two
   bridge tongues. Fit the four M3 × 12 screws from above through the large
   access bores and tighten evenly onto the seated tongues. Do not force
   tongues into their sockets; remove print burrs first. Then, with the
   battery absent, load the two central clamp nuts through the open
   bottom. Put the chassis and cap around the tube (10.5 mm horizontal × 10 mm
   vertical), fit the two opposed top-down screws, and tighten evenly. The cap must
   grip the tube before its closing gap disappears. Seat screw heads in the shallow cap counterbores. Fit the IMU later to preserve driver access.
3. Slide the deck and landing-frame nuts into their SIDE openings. Fit the two
   landing frames from their outside faces. Load the two hatch nuts through their side slots while everything
   is accessible. A small piece of removable tape can retain loose nuts before
   their screws engage; it carries no structural load.
4. For each end mount, load the tube-clamp nuts from above, put the frame and
   saddle around the tube, then install the bottom-up screws. Load the two
   pivot-foot nuts from below before installing the motor cradle.
5. Bolt the servo's bare ears to its frame. On the bench, attach the supplied
   horn to the separate motor cradle with the four M3 × 10 bolts and nuts.
   Use a 2.5 mm L-shaped hex key from the open underside of the cradle for the
   inside screw heads; hold the nuts before the horn is fitted to the servo.
6. Centre the servo with the propeller absent, disconnect power, then fit the
   horn/cradle to the spline. Install the supplied CENTRE screw now. Its driver
   enters through the EMPTY opposite journal: the revised bore is 4.5 mm and
   the checked straight driver-shaft envelope is 4 mm diameter. Use a suitably
   slim, long driver with the correct tip. The journal nut, retainer and pivot
   support must not obstruct this path yet.
7. Insert the journal nut from inside the cradle. Slip the removable pivot
   support over the journal and install the retainer. Tightening the retainer
   should clamp it to the journal end, not squeeze the stationary support.
   Position the support in its foot slots with clearance at both rotating
   faces (nominally 1.0 mm inboard and 0.8 mm outboard), then secure its foot.
   Verify rotation before adding the propeller.
8. Attach the metal motor bracket using the underside cradle nuts and top-down
   screws. Keep screw tips out of motor internals. The checked top-down driver
   paths clear the modeled motor envelope. Leave flexible wire loops near the
   tilt axis and check their clearance through the actual tilt travel.
9. Fit the electronics deck and seat both locating pegs and tighten the two diagonal screws BEFORE taping down the
   perfboard. The raised tape rails put its underside 4 mm above the deck,
   clearing ordinary ~3 mm-high socket heads. Removing the deck later may
   require lifting the taped board; battery swaps do not. Fit the IMU and camera
   cap, and route their wires without obstructing the battery connector or hatch access.
10. Fit the battery and hatch from below using the reverse of the release
    sequence. Ensure both locating pegs are seated and both screws are tight.
    Check that the pack cannot rattle or escape. Check balance with the actual
    electronics and complete motor assemblies installed.

## Edge finishing

The tiny 0.25–0.30 mm chamfers have been removed from both chassis and servo
mounts at the user's request. Retained chamfers are 0.40 mm at the
hatch support pads/end stops and deck perimeter, and 0.60 mm at the
landing-foot toe corners. Both landing crossbars now have 2 mm chamfers on
their two exposed top outside corners. Existing structural root fillets and
larger servo-frame corner chamfers remain. Nut pocket widths and enlarged
depths are unchanged. Both studios regenerated successfully after cleanup.

The two central clamp screws sit on opposite sides of the tube. The electronics
deck now rests on the central cap, secured with two diagonal screws and registered by
two pegs. This revision saves four screws and four nuts without merging parts
that need to separate for printing or maintenance. Do not apply the reduction
by simply omitting screws from older parts: export the revised geometry.

## Verification and limits

The revised chassis regenerated without notices and its closed-position solids
were checked for interferences. Twenty-seven intermediate positions of the hatch release motion were checked
against the stationary chassis and hardware envelopes, with no interferences. Earlier servo-mount
checks covered nut and screwdriver access; that source was unchanged in this
hatch revision.

These are nominal geometry/assembly checks, not a strength rating or physical
fit test. Actual print shrinkage, screw heads, battery covering/lead placement,
and the chosen driver still need checking on the first assembled unit. Print
the hatch flat on its broad bottom face; do not scale it to correct hole fit.


## Nut-fit audit, 2026-10-11

The four landing-frame nut recesses now match their access slots in depth,
removing the 0.1 mm internal step. Small entrance chamfers were subsequently
removed as requested; the enlarged pockets remain. Hex pockets remain 6.2 mm across flats throughout both part studios.

Temporary CAD checks swept a 5.7 mm across-flats hex envelope through all
21 captive-nut insertion paths (10 chassis and 11 per servo-mount design).
Side-entry envelopes were 2.6 mm thick; axial-entry envelopes covered the
continuous loading path to near the pocket shoulder. All cleared their host
parts. Load central nuts before the battery, and servo nuts in the assembly
order above. Both studios regenerated without errors and had no solid
interferences. These are nominal CAD checks, not a guarantee against printed
burrs, sagging slot roofs, or printer dimensional error. Existing exported
STLs must be replaced to include these changes. Screw count and lengths stay
the same.


## Earlier one-piece revision checks

The earlier one-piece revision regenerated as 19 solids without errors or nominal part
interferences. The assembly mate group remains OK. All ten chassis/cap nut
insertion paths pass the swept-envelope check, including the relocated deck
nuts. All 27 sampled hatch-release positions clear the revised stationary
parts. The camera position, centred battery, servo source and carbon-tube size
are unchanged; the IMU mounting plane is now z = 15 mm. Landing-foot positions
are retained to preserve the support footprint. No physical landing-load test
or printed fit test has been performed.


## Modular validation

The final Part Studio contains 21 solids (nine printed parts and twelve hardware
reference bodies), with no nominal solid interference. Fourteen sampled
side-frame insertion positions clear the bridge. Eighteen nut/driver checks
pass, including all four new screw-access bores before installing the cap.
All 27 sampled battery-release positions remain clear. This is geometric
validation and a load-path review, not a measured strength or fatigue rating.
The joints add fasteners and must be inspected for looseness after initial
assembly and gentle landing/load tests before powered flight.
