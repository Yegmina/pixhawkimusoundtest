#!/usr/bin/env python3
"""Serve a live Pixhawk6C IMU / compass dashboard without web dependencies."""

from __future__ import annotations

import json
import os
import queue
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).parent
SERIAL_PORT = os.environ.get("PIXHAWK_PORT", "/dev/ttyACM0")
BAUD = int(os.environ.get("PIXHAWK_BAUD", "115200"))
STREAM_HZ = max(1, int(os.environ.get("PIXHAWK_STREAM_HZ", "20")))


class TelemetryReader(threading.Thread):
    """One MAVLink consumer, fan-out to all connected browser clients."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.clients: set[queue.Queue] = set()
        self.clients_lock = threading.Lock()
        self.status = {"connected": False, "port": SERIAL_PORT, "error": "Starting…"}
        self.status_lock = threading.Lock()

    def subscribe(self) -> queue.Queue:
        client: queue.Queue = queue.Queue(maxsize=100)
        with self.clients_lock:
            self.clients.add(client)
        return client

    def unsubscribe(self, client: queue.Queue) -> None:
        with self.clients_lock:
            self.clients.discard(client)

    def publish(self, packet: dict) -> None:
        with self.clients_lock:
            clients = tuple(self.clients)
        for client in clients:
            try:
                client.put_nowait(packet)
            except queue.Full:
                try:
                    client.get_nowait()
                    client.put_nowait(packet)
                except queue.Empty:
                    pass

    def set_status(self, **values: object) -> None:
        with self.status_lock:
            self.status.update(values)

    def snapshot_status(self) -> dict:
        with self.status_lock:
            return dict(self.status)

    def request_streams(self, link: mavutil.mavfile) -> None:
        # Requesting stream intervals is temporary and does not modify flight parameters.
        for message_id in (26, 27, 105, 116, 129):
            link.mav.command_long_send(
                link.target_system,
                link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                int(1_000_000 / STREAM_HZ),
                0, 0, 0, 0, 0,
            )

    @staticmethod
    def to_packet(message: object) -> dict | None:
        kind = message.get_type()
        # RAW_IMU is the unambiguous primary stream. SCALED_IMU and HIGHRES_IMU
        # contain duplicate samples and made the graphs visually misleading.
        if kind not in {"RAW_IMU", "SCALED_IMU2", "SCALED_IMU3"}:
            return None
        data = message.to_dict()
        # RAW/SCALED values are milligravity, milliradian/s, and milligauss.
        # HIGHRES values are already SI except PX4's magnetometer representation,
        # so RAW_IMU / SCALED_IMU are used as the primary live series.
        packet = {
            "type": kind,
            "time": time.time() * 1000,
            "accelerometer": [data.get("xacc"), data.get("yacc"), data.get("zacc")] if kind == "RAW_IMU" else None,
            "gyroscope": [data.get("xgyro"), data.get("ygyro"), data.get("zgyro")] if kind == "RAW_IMU" else None,
            "temperature": data.get("temperature"),
            "sensorId": data.get("id", 0),
        }
        magnetic = [data.get("xmag"), data.get("ymag"), data.get("zmag")]
        # PX4 exposes additional magnetic sensor instances through SCALED_IMU2/3.
        # MAVLink does not include the physical device name, so this is the CAN1
        # RM3100 stream selected for this vehicle.
        packet["rm3100"] = (
            magnetic if kind in {"SCALED_IMU2", "SCALED_IMU3"} and any(magnetic) else None
        )
        packet["magnetometer"] = magnetic if kind == "RAW_IMU" else None
        return packet

    def run(self) -> None:
        while True:
            link = None
            try:
                self.set_status(connected=False, error=f"Opening {SERIAL_PORT}…")
                link = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD, autoreconnect=False)
                heartbeat = link.wait_heartbeat(timeout=8)
                if heartbeat is None:
                    raise TimeoutError("No MAVLink heartbeat")
                self.request_streams(link)
                self.set_status(
                    connected=True,
                    error=None,
                    system=heartbeat.get_srcSystem(),
                    component=heartbeat.get_srcComponent(),
                )
                while True:
                    message = link.recv_match(blocking=True, timeout=2)
                    if message is None:
                        continue
                    packet = self.to_packet(message)
                    if packet:
                        self.publish(packet)
            except Exception as error:
                self.set_status(connected=False, error=str(error))
                time.sleep(2)
            finally:
                if link:
                    link.close()


reader = TelemetryReader()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        print(f"{self.address_string()} - {format % args}")

    def send_json(self, data: dict) -> None:
        raw = json.dumps(data).encode()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self) -> None:
        if self.path == "/api/status":
            self.send_json(reader.snapshot_status())
            return
        if self.path == "/events":
            self.stream_events()
            return
        if self.path not in {"/", "/index.html"}:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        body = (ROOT / "index.html").read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream_events(self) -> None:
        client = reader.subscribe()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                try:
                    packet = client.get(timeout=10)
                    self.wfile.write(f"data: {json.dumps(packet)}\n\n".encode())
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            reader.unsubscribe(client)


if __name__ == "__main__":
    reader.start()
    port = int(os.environ.get("PORT", "8000"))
    print(f"Dashboard: http://localhost:{port}  |  Pixhawk: {SERIAL_PORT} @ {BAUD}")
    ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()
