# brief: flight replay

## the product
a place to watch a flight again and understand it: what the drone did, what it was told, and where it went wrong. first reader is the person who just flew it and the agent helping them; the same viewer later plays sim runs, flights flown by hand in the sim, and RL rollouts.

## the hero
the replay: the drone in 3D, moving as it moved, with its two rotors tilting and their thrust visible. everything else serves it: the timeline under it says when, the strips beside it say how much. nothing is hoisted above it (no summary cards on top).

## direction: braun functionalism with tufte's data-ink
calm off-white and graphite, hairlines, one signal color. engineering artifacts (the drone, its trail, the plots) are drawn as precise line work, not rendered as glossy objects. charts are small multiples with labels on the lines instead of legends, no boxes, no gridline noise, one shared cursor.

why not the attractors: the current page is the generic dark devtool (graphite, blue lines, cards); stock shadcn gray adds nothing; warm paper is a social look. a flight viewer is an instrument: rams' devices (one signal color, precise controls, nothing extra) and tufte's plots (maximum data, minimum chrome) are the right primary sources.

## constants for this surface
- signal color (`--signal`, an orange) means "now and what needs you": the playhead, the drone's thrust, and events that ended or endangered a flight (a stop, a recovery, a tip cut). nothing else is colored.
- data is ink and gray; the target is a lighter dash of the same ink, never a second hue.
- every number is tabular; units ride on the axis label, once.
- the timeline marks events where they happened; hovering shows what; quiet events (mode changes) stay gray.
- every state is designed: no flight chosen, loading (invisible for 200 ms, then a placeholder shaped like the viewer), unreadable recording, a recording without the newer fields (no position: the drone replays in place).
- nothing shifts: the viewer's frame is fixed before data arrives; switching flights swaps content inside it.
- motion: playback is the only animation; ui transitions 160 ms on ease-out; reduced motion keeps playback (it's the content) and drops the ui transitions.

## modules (reusable, nothing knows where data came from)
- `lib/trajectory.ts`: the format every producer emits (time, position, attitude quaternion, rotor thrust and tilt, named channels, events).
- `components/replay/`: `scene` (three.js: ground, trail, drone), `drone` (the bicopter drawn from its geometry), `timeline` (scrubber, events, play, speed), `strips` (canvas small multiples with the shared cursor), `use-playback` (the clock), `replay` (puts them together).
- producers: `lib/flight-report.ts` turns a recording into a trajectory; sims and rollouts write the same JSON.
