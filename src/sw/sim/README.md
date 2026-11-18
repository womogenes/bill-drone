## Drone simulation

Host simulations of drone 1 on the firmware's own C (fw/drone/main, compiled here through ctypes on first use), against fitted plants. Run from src: `uv run -m sw.sim.<module> --help`. Heavy runs go to drone-v0. The tools removed on 2026-11-15 (the Python controllers that predated the firmware port, one-off replays and analyses, the GPS/EKF replay set) are on `william/archive-2026-11-15-indi`. models/NOTES.md says which plants are current and what the sims show.

**The firmware's mirrors**: flight.py (flight_math.c: the attitude estimate, the controller, INDI), hover.py (hover.c), guard.py (guard.c), ekf.py (ekf.c), flight_many.c (the library flight.py loads: flight_math.c and indi.c, with Command's offsets). Their structures mirror the headers; a change to a firmware structure needs its mirror changed.

**Plants**: rigid.py and rigid_step.c (the full-attitude rigid body, numpy and C with parity tests in test_rigid.py), airframe.py (the fitted terms), disturbance.py; sweep.py's lean 6-DOF plant for the hover sims.

**Burst and recovery**: flip_firmware.py flies flight_rt.c's tick order (estimate, burst, recovery or tip cut, guards, hold, controller, actuators) from the GPS hold on rigid.py; flip_esc.py the bursts per rotor_momentum band on the 2026-11-14 ESCs (--sag for the pack); recover_firmware.py the tumble scenarios and Level; tune_flip.py CMA-ES over the burst's parameters.

**Hover**: sweep.py (one flight, nominal with no arguments), outdoor.py (GPS-held hovers in wind), landing_test.py (landings, the battery tray, replays of recorded landings), takeoff_test.py (takeoffs from tilted rests).

**Fitting and checking plants**: fit_plant.py (the one plant, fitted on the scorecard), scorecard.py and scorecard_closed.py (every flight's prediction score and closed-loop rerun), burst_closed.py (a recorded burst flown again by the firmware that flew it), plant_response.py (the measured and the plant's servo-to-pitch frequency response), authority.py (the firmware's one airframe model fitted to the flights).
