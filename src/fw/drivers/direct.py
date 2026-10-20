"""
Requests over the native ESP-NOW link (native/runtime/radio.c) that need the application: file updates and reads. The native runtime handles everything else.
"""

import asyncio
import radio
from drivers import update


def handle(request):
    if request[0] in (ord("F"), ord("P"), ord("M"), ord("Y"), ord("L"), ord("K")):
        return update.handle(request)
    raise ValueError(f"Unknown request {chr(request[0])}")


async def requests():
    while True:
        request = radio.request()
        if request is None:
            await asyncio.sleep_ms(10)
            continue
        try:
            radio.reply(b"\x00" + handle(request))
        except Exception as error:
            radio.reply(b"\x01" + repr(error).encode()[:200])
