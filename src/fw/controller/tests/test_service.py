"""Run with: uv run --with aiohttp --with pyserial python -m unittest discover -s src/fw/controller/tests."""

import asyncio
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import AsyncMock, patch
from aiohttp import web

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge import ClientSender, Service
from wire import SerialDecoder, pack, unpack


class Socket:
    def __init__(self):
        self.closed = False
        self.started = asyncio.Event()
        self.release = asyncio.Event()
        self.messages = []

    async def send_json(self, message):
        self.started.set()
        await self.release.wait()
        if self.closed:
            raise ConnectionResetError()
        self.messages.append(message)

    async def close(self):
        self.closed = True


class Port:
    def __init__(self):
        self.packets = []
        self.decoder = SerialDecoder()

    def write(self, data):
        self.packets.extend(self.decoder.feed(data))
        return len(data)


class ServiceTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.port = Port()
        self.service = Service(self.port, Path(self.directory.name))
        self.service.session = 1
        self.service.last_status = time.monotonic()
        self.tasks = []

    async def asyncTearDown(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.service.recorder.close("Test ended")
        self.directory.cleanup()

    async def test_slow_browser_retains_only_latest_reading(self):
        ws = Socket()
        sender = ClientSender(ws)
        self.tasks.append(sender.task)
        sender.push({"type": "reading", "time": 0})
        await ws.started.wait()
        for stamp in range(1, 10001):
            sender.push({"type": "reading", "time": stamp})
        sender.push({"type": "result", "request": "first"})
        sender.push({"type": "result", "request": "second"})
        self.assertEqual(len(sender.replies), 2)
        self.assertEqual(sender.reading["time"], 10000)
        ws.release.set()
        for attempt in range(20):
            if len(ws.messages) == 4:
                break
            await asyncio.sleep(0)
        self.assertEqual([message.get("time", message.get("request")) for message in ws.messages],
                         [0, "first", "second", 10000])
        await sender.close()

    async def test_socket_closing_during_send_is_clean(self):
        ws = Socket()
        sender = ClientSender(ws)
        sender.push({"type": "reading"})
        await ws.started.wait()
        ws.closed = True
        ws.release.set()
        await sender.task
        self.assertIsNone(sender.task.exception())

    async def test_slow_socket_close_has_a_deadline(self):
        close_started, close_cancelled = asyncio.Event(), asyncio.Event()

        class SlowCloseSocket(Socket):
            async def close(self):
                close_started.set()
                try:
                    await asyncio.Event().wait()
                finally:
                    close_cancelled.set()

        sender = ClientSender(SlowCloseSocket())
        self.tasks.append(sender.task)
        # Let the sender enter its try/finally before cancelling it.
        await asyncio.sleep(0)
        await asyncio.wait_for(sender.close(), 1)
        self.assertTrue(close_started.is_set())
        self.assertTrue(close_cancelled.is_set())
        self.assertTrue(sender.task.done())

    async def test_dashboard_disconnect_cancels_transactions_but_keeps_stop(self):
        disconnected = asyncio.Event()

        class DashboardSocket(Socket):
            def __aiter__(self):
                return self

            async def __anext__(self):
                await disconnected.wait()
                raise StopAsyncIteration

        ws = DashboardSocket()
        self.service.dashboard = ws
        self.service.senders[ws] = ClientSender(ws)
        self.service.transactions[ws] = set()
        with patch.object(self.service, "connect", new=AsyncMock(return_value=ws)):
            handler = asyncio.create_task(self.service.dashboard_socket(None))
            self.tasks.append(handler)
            self.service.dashboard_message(ws, {"type": "arm"})
            self.service.dashboard_message(ws, {"type": "tune", "settings": {"roll_p": 20}})
            self.service.dashboard_message(ws, {"type": "stop"})
            pending = tuple(self.service.transactions[ws])
            for attempt in range(20):
                if self.service.stop_reply is not None:
                    break
                await asyncio.sleep(0)
            stop = self.service.stopping
            self.tasks.append(stop)
            self.assertIsNotNone(self.service.stop_reply)
            self.service.send(1, b"queued target")
            disconnected.set()
            await asyncio.wait_for(handler, 1)

        self.assertTrue(all(task.cancelled() for task in pending))
        self.assertIsNone(self.service.dashboard)
        self.assertIsNone(self.service.target)
        self.assertEqual(len(self.service.outgoing), 0)
        self.assertNotIn(ws, self.service.transactions)
        self.assertNotIn(ws, self.service.senders)
        self.assertFalse(stop.done())
        self.assertFalse(self.service.stop_reply.cancelled())
        # The original stop still accepts its acknowledgement after the owner leaves.
        identifier = next(key for key, reply in self.service.waiting.items() if reply is self.service.stop_reply)
        self.service.receive(pack(6, 1, 99, 0, identifier + b"\x00"))
        await asyncio.wait_for(stop, 1)
        self.assertFalse(stop.cancelled())

    async def test_stop_preempts_targets_and_requests(self):
        self.service.send(6, b"req1S")
        self.service.send(1, b"old target")
        self.service.send(1, b"new target")
        self.service.send(6, b"stopX")
        writer = asyncio.create_task(self.service.write())
        self.tasks.append(writer)
        await asyncio.sleep(0)
        self.assertEqual([unpack(packet)[-1] for packet in self.port.packets],
                         [b"stopX", b"new target", b"req1S"])

    async def test_record_retries_write_once_and_ack_after_write(self):
        payload = b"LGS1" + struct.pack("<II3f2fB", 5, 0, 0, 0, 0, 0, 0, 1)
        packet = pack(2, 1, 10, 0, payload)
        self.service.receive(packet)
        self.service.receive(packet)
        self.assertEqual(self.service.record_queue.qsize(), 1)
        self.assertEqual(len(self.service.outgoing), 0)
        self.tasks.append(asyncio.create_task(self.service.record()))
        await self.service.record_queue.join()
        self.assertEqual(len(list(Path(self.directory.name).glob("*.bin"))), 1)
        self.assertEqual(self.service.delivered[2], 10)
        self.assertEqual(len(self.service.outgoing), 1)
        self.service.receive(packet)
        self.assertEqual(self.service.record_queue.qsize(), 0)
        self.assertEqual(len(self.service.outgoing), 2)

    async def test_slow_disk_does_not_block_control_writer(self):
        self.service.recorder.begin(5, {})
        entered, release = threading.Event(), threading.Event()

        def slow_fsync(fd):
            entered.set()
            release.wait(2)

        self.tasks.append(asyncio.create_task(self.service.record()))
        self.tasks.append(asyncio.create_task(self.service.write()))
        with patch("os.fsync", side_effect=slow_fsync):
            self.service.queue_record("check", False)
            try:
                self.assertTrue(await asyncio.to_thread(entered.wait, 1))
                self.service.send(1, b"target while disk is blocked")
                await asyncio.sleep(0)
                self.assertEqual(unpack(self.port.packets[0])[-1], b"target while disk is blocked")
                self.assertFalse(release.is_set())
            finally:
                release.set()
            await self.service.record_queue.join()

    async def test_disk_error_fails_service_without_acknowledging_record(self):
        payload = b"LGS1" + struct.pack("<II3f2fB", 5, 0, 0, 0, 0, 0, 0, 1)
        self.tasks.append(asyncio.create_task(self.service.record()))
        with patch.object(self.service.recorder, "receive", side_effect=OSError("Injected disk failure")):
            self.service.receive(pack(2, 1, 10, 0, payload))
            await asyncio.wait_for(self.service.record_queue.join(), 1)
        self.assertTrue(self.service.failed.is_set())
        self.assertEqual(self.service.recorder.error, "Injected disk failure")
        self.assertNotIn(2, self.service.delivered)
        self.assertEqual(len(self.service.outgoing), 0)

    async def test_failed_settings_refresh_is_retried(self):
        async def unavailable(*args):
            raise TimeoutError("Injected missing reply")

        self.service.request = unavailable
        await self.service.refresh_settings(4)
        self.assertIsNone(self.service.settings_version)
        self.assertGreater(self.service.settings_retry, time.monotonic())

    async def test_old_arm_cannot_retry_after_drone_restart(self):
        request = asyncio.create_task(self.service.request(b"A"))
        await asyncio.sleep(0)
        self.assertEqual(len(self.service.outgoing), 1)
        self.service.disconnect("Drone restarted")
        self.service.session = 2
        with self.assertRaisesRegex(RuntimeError, "Drone restarted|link changed"):
            await request
        self.assertEqual(len(self.service.outgoing), 0)

    async def test_cancelled_arm_is_removed_before_serial_write(self):
        request = asyncio.create_task(self.service.request(b"A"))
        await asyncio.sleep(0)
        self.assertEqual(len(self.service.outgoing), 1)
        request.cancel()
        await asyncio.gather(request, return_exceptions=True)
        self.assertEqual(len(self.service.outgoing), 0)

    async def test_stop_cancels_unacknowledged_arm_retry(self):
        request = asyncio.create_task(self.service.request(b"A"))
        await asyncio.sleep(0)
        self.assertEqual(len(self.service.outgoing), 1)
        self.tasks.append(self.service.stop())
        with self.assertRaisesRegex(RuntimeError, "Stopped"):
            await request
        self.assertEqual(len(self.service.outgoing), 0)
        self.assertFalse(self.service.arm_requests)

    async def test_stop_cancels_arm_before_transaction_starts(self):
        ws = Socket()
        self.service.transactions[ws] = set()
        self.service.dashboard_message(ws, {"type": "arm"})
        arm = next(iter(self.service.arm_tasks))
        self.tasks.append(self.service.stop())
        await asyncio.gather(arm, return_exceptions=True)
        self.assertTrue(arm.cancelled())
        self.assertEqual(len(self.service.outgoing), 0)

    async def test_timeout_preserves_recording_deduplication(self):
        payload = b"LGS1" + struct.pack("<II3f2fB", 5, 0, 0, 0, 0, 0, 0, 1)
        packet = pack(2, 1, 10, 0, payload)
        self.tasks.append(asyncio.create_task(self.service.record()))
        self.service.receive(packet)
        await self.service.record_queue.join()
        self.service.disconnect("Transient timeout")
        self.service.receive(packet)
        await self.service.record_queue.join()
        self.assertEqual(len(list(Path(self.directory.name).glob("*.bin"))), 1)

    async def test_resumed_recording_clears_temporary_stall_error(self):
        recorder = self.service.recorder
        recorder.begin(5, {})
        recorder.last_message = time.monotonic() - 2
        recorder.check(True)
        self.assertEqual(recorder.error, "Flight log stalled")
        recorder.receive(3, b"{}")
        self.assertIsNone(recorder.error)
        self.assertFalse(recorder.stalled)

    async def test_two_dashboards_cannot_acquire_same_slot(self):
        preparing, release = asyncio.Event(), asyncio.Event()

        class PreparingSocket(Socket):
            async def prepare(self, request):
                preparing.set()
                await release.wait()

        with patch("bridge.web.WebSocketResponse", side_effect=lambda **kwargs: PreparingSocket()):
            first = asyncio.create_task(self.service.connect(None, "dashboard"))
            await preparing.wait()
            with self.assertRaises(web.HTTPServiceUnavailable):
                await self.service.connect(None, "dashboard")
            release.set()
            ws = await first
            await self.service.senders.pop(ws).close()


if __name__ == "__main__":
    unittest.main()
