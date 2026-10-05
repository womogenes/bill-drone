# Drone

We're building a bicopter!

Still in the very early stages of this, but hope to get results quickly.

## Links

- Live main doc (components, etc). Plese read this to get a sense of what components we are using, before claiming you don't have enough information on our components list. Doc: https://docs.google.com/document/d/1pgYeX1OccuEGhHezcuSxMdpLn0v6e36mq_4py6jGJlA/edit?tab=t.0

## Notes

- We are willing to do a lot of integration and coding. Do not say "this is impossible / potentially possible" --- if it's possible with engineering work, it's possible. Don't ask for photos of things if you can figure it out with enough thinking/research.
- We are bold. A 3.7V battery will definitely power an ESP32-CAM unless experiments show otherwise.
- When you give `uv run` commands, assume we're in `src/`. So, for example, `uv run --project src python src/sw/sensors/camera_readings.py` is too verbose. Instead, just say `uv run sw/sensors/camera_readings.py`.
- Do not keep tests checked in.
- We are lean. We are scrappy. We should keep the absolute minimum amount possible in this repo that is required to do a thing. Do not maintain backwards compatability. Do not write defensive code; prefer to fail loudly. For example, the guard clause here:

```py
def get_frame():
"""
Return a single cv2 frame from the camera
"""
ok, frame = camera.read()
if not ok:
    raise RuntimeError("Could not read a frame from go2rtc")
return frame
```

is a bad pattern and must be avoided. We should let it fail loudly.

## Code style

- Python docstrings should have a new line after the first """ and before the last """

## Project structure

- `src/` contains software and firmware.
- `src/lib/` contains external software and firmware. This includes things like ESC control etc. These should largely be unmodified.
- `src/sw` contains software and firmware.
- `src/hw` is empty right now but will contain things like PCB schematics etc.

## Local setup

Static laptop is streaming go2rtc over port 8555. On my tailnet, that's `http://100.64.0.6:1984/`.
