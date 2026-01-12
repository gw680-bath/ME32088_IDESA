"""UDP comms layer for the parallel IDESA stack."""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Callable, Optional

from IDESA_State_2 import IDESAStateStore2
from IDESA_Types_2 import NavCommand2

NAV_PACKET_STRUCT = struct.Struct("<I3f")
ACK_PACKET_STRUCT = struct.Struct("<I")


class SendHistory2:
    """Keeps track of when each sequence was transmitted."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sent_at: dict[int, float] = {}

    def record(self, seq: int, timestamp: float) -> None:
        with self._lock:
            self._sent_at[seq] = timestamp

    def pop(self, seq: int) -> Optional[float]:
        with self._lock:
            return self._sent_at.pop(seq, None)

    def oldest_age(self, now: float) -> float:
        with self._lock:
            if not self._sent_at:
                return 0.0
            oldest = min(self._sent_at.values())
            return max(0.0, now - oldest)


class UDPSender2:
    """Periodically pushes NavCommand2 messages to Simulink."""

    def __init__(
        self,
        simulink_ip: str,
        state_store: IDESAStateStore2,
        command_supplier: Callable[[], NavCommand2],
        send_history: SendHistory2,
        port: int = 50001,
        send_hz: float = 30.0,
        ack_timeout_s: float = 0.5,
    ) -> None:
        if not simulink_ip:
            raise ValueError("simulink_ip must be provided")
        self._addr = (simulink_ip, int(port))
        self._state_store = state_store
        self._command_supplier = command_supplier
        self._send_history = send_history
        self._send_period = 1.0 / max(float(send_hz), 1.0)
        self._ack_timeout_s = max(float(ack_timeout_s), 0.1)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._seq = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="UDPSender2", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._thread:
            return
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_event.clear()

    def close(self) -> None:
        self.stop()
        try:
            self._sock.close()
        except OSError:
            pass

    def _run(self) -> None:
        next_send = time.time()
        while not self._stop_event.is_set():
            command = self._command_supplier()
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFFFFFF

            packet = NAV_PACKET_STRUCT.pack(
                seq,
                float(command.distance_error_mm),
                float(command.angle_error_deg),
                float(command.enable),
            )

            sent_at = time.time()
            try:
                self._sock.sendto(packet, self._addr)
            except OSError:
                # Keep trying until socket recovers.
                pass
            else:
                self._send_history.record(seq, sent_at)
                self._state_store.update(last_udp_send_time=sent_at, last_sent_seq=seq)
                self._evaluate_ack_timeout(sent_at)

            next_send += self._send_period
            sleep_time = max(0.0, next_send - time.time())
            if sleep_time == 0.0:
                next_send = time.time() + self._send_period
                sleep_time = self._send_period
            self._stop_event.wait(sleep_time)

    def _evaluate_ack_timeout(self, now: float) -> None:
        age = self._send_history.oldest_age(now)
        if age > self._ack_timeout_s:
            self._state_store.update(comms_lost=True)


class UDPAckReceiver2:
    """Listens for ACK packets coming back from Simulink."""

    def __init__(
        self,
        state_store: IDESAStateStore2,
        send_history: SendHistory2,
        bind_ip: str = "172.26.46.132",
        port: int = 50002,
        recv_buf: int = 1024,
    ) -> None:
        self._state_store = state_store
        self._send_history = send_history
        self._bind = (bind_ip, int(port))
        self._recv_buf = recv_buf

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.2)

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        try:
            self._sock.bind(self._bind)
        except OSError as exc:
            raise RuntimeError(f"Failed to bind UDP ACK socket on {self._bind}: {exc}")
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="UDPAckReceiver2", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._thread:
            return
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_event.clear()
        try:
            self._sock.close()
        except OSError:
            pass

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, _ = self._sock.recvfrom(self._recv_buf)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < ACK_PACKET_STRUCT.size:
                continue
            seq = ACK_PACKET_STRUCT.unpack_from(data)[0]
            received = time.time()
            sent_at = self._send_history.pop(seq)
            rtt_ms = (received - sent_at) * 1000.0 if sent_at else 0.0
            self._state_store.update(
                last_ack_seq=seq,
                last_ack_time=received,
                last_rtt_ms=rtt_ms,
                comms_lost=False,
            )

    def __del__(self) -> None:  # pragma: no cover - best-effort cleanup
        self.stop()
