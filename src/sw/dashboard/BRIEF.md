# brief: flight replay

## the product
a place to watch a flight again and understand it: what the drone did, what it was told, and where it went wrong. first reader is the person who just flew it and the agent helping them; the same viewer later plays sim runs, flights flown by hand in the sim, and RL rollouts.

## the hero
the replay: the drone in 3D, moving as it moved, with its two rotors tilting and their thrust visible. everything else serves it: the timeline under it says when, the strips beside it say how much. nothing is hoisted above it (no summary cards on top).

## direction: braun functionalism with tufte's data-ink
calm surfaces, hairlines, one signal color. engineering artifacts (the drone, its trail, the plots) are drawn as precise line work, not rendered as glossy objects. charts are small multiples with labels on the lines instead of legends, no boxes, no gridline noise, one shared cursor.

why not the attractors: the old page was the generic dark devtool (graphite, blue lines, cards). a flight viewer is an instrument: rams' devices (one signal color, precise controls, nothing extra) and tufte's plots (maximum data, minimum chrome) are the primary sources. the look comes from restraint and line work, not new colors: the dashboard's AGENTS.md keeps to the shadcn theme's tokens and Tailwind's palette, so light and dark both follow the system as the rest of the dashboard does.

## constants for this surface
- colors: background, foreground, muted-foreground and border from the shadcn theme; Tailwind orange (600 light, 500 dark) is the signal, meaning "now and what needs you": the playhead, the rotors' thrust, and events that ended or endangered a flight (a stop, a recovery, a tip cut). nothing else is colored; errors are red-700.
- data is foreground and grays; the target is a muted dash, never a second hue.
- every number is tabular; units ride on the strip's label, once.
- the timeline marks events where they happened; hovering shows what; quiet events (mode changes) stay gray.
- every state is designed: no flight chosen, loading, unreadable recording, no WebGL (the hero says so, the rest works), a recording without position (the drone replays in place).
- nothing shifts: the viewer's frame is fixed before data arrives; switching flights swaps content inside it.
- motion: playback is the only animation; controls use shadcn's own transitions.
- the strips are canvas, not shadcn charts: 13 series at 50 Hz with a cursor redrawn every frame of playback is more than an SVG chart library keeps smooth.

## modules (reusable, nothing knows where data came from)
- `lib/trajectory.ts`: the format every producer emits (time, position, attitude quaternion, rotor thrust and tilt, named channels, events).
- `components/replay/`: `scene` (three.js: ground, trail, drone), `drone` (the bicopter drawn from its geometry), `timeline` (scrubber, events, play, speed), `strips` (canvas small multiples with the shared cursor), `use-playback` (the clock), `replay` (puts them together).
- producers: `lib/flight-report.ts` turns a recording into a trajectory; a sim or rollout would write the same JSON.
