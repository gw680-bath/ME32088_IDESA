"""Entrypoint for the parallel IDESA navigation stack."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from typing import Sequence, Tuple

from IDESA_Comms_2 import SendHistory2, UDPAckReceiver2, UDPSender2
from IDESA_GUI_mission_control_2 import create_gui
from IDESA_Mission_2 import TargetQueueController2
from IDESA_Navigation_2 import NavigationController2
from IDESA_State_2 import IDESAStateStore2
from IDESA_Vision_2 import VisionSystem2


@dataclass
class ParallelConfig2:
    simulink_ip: str = "138.38.226.147"
    bind_ip: str = "172.26.46.132"
    send_port: int = 50001
    ack_port: int = 50002
    send_hz: float = 30.0
    ack_timeout_s: float = 0.5
    angle_tol_deg: float = 5.0
    dist_tol_mm: float = 100.0
    camera_index: int = 1
    available_target_ids: Tuple[int, ...] = (2, 3, 4, 5, 6, 7)
    default_target_ids: Tuple[int, ...] = (2, 3)


class ParallelRuntime2:
    """Wires every subsystem together."""

    def __init__(self, config: ParallelConfig2) -> None:
        self.config = config
        self.state_store = IDESAStateStore2()

        self.mission = TargetQueueController2(
            self.state_store,
            switch_radius_mm=config.dist_tol_mm,
            available_target_ids=config.available_target_ids,
        )
        if config.default_target_ids:
            self.mission.set_target_ids(config.default_target_ids)

        self.navigation = NavigationController2(
            self.state_store,
            angle_tol_deg=config.angle_tol_deg,
            dist_tol_mm=config.dist_tol_mm,
        )

        self.vision = VisionSystem2(
            self.state_store,
            camera_index=config.camera_index,
            target_ids=config.default_target_ids,
        )

        self.send_history = SendHistory2()
        self.sender = UDPSender2(
            config.simulink_ip,
            self.state_store,
            self.navigation.get_command,
            self.send_history,
            port=config.send_port,
            send_hz=config.send_hz,
            ack_timeout_s=config.ack_timeout_s,
        )
        self.ack_receiver = UDPAckReceiver2(
            self.state_store,
            self.send_history,
            bind_ip=config.bind_ip,
            port=config.ack_port,
        )

        self._started = False

    def start(self) -> None:
        self.navigation.release_robot_stop()
        if self._started:
            return
        self.mission.start()
        self.navigation.start()
        self.ack_receiver.start()
        self.sender.start()
        self.vision.start_camera()
        self.vision.start_tracking()
        self._started = True

    def stop(self) -> None:
        self.navigation.engage_robot_stop()
        if not self._started:
            return
        self.sender.close()
        self.ack_receiver.stop()
        self.navigation.stop()
        self.mission.stop()
        self.vision.stop()
        self._started = False

    def robot_stop(self) -> None:
        self.navigation.engage_robot_stop()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Parallel IDESA mission controller")
    parser.add_argument("--simulink-ip", default="138.38.226.147", help="Destination IP for Simulink UDP (port 50001)")
    parser.add_argument("--bind-ip", default="172.26.46.132", help="Local IP for receiving ACKs (port 50002)")
    parser.add_argument("--send-hz", type=float, default=30.0, help="Command send rate")
    parser.add_argument("--ack-timeout", type=float, default=0.5, help="Seconds before comms_lost is asserted")
    parser.add_argument("--angle-tol", type=float, default=5.0, help="ALIGNING tolerance in degrees")
    parser.add_argument("--dist-tol", type=float, default=100.0, help="TRAVELING tolerance in millimetres")
    parser.add_argument("--camera-index", type=int, default=1, help="OpenCV camera index")
    parser.add_argument("--targets", type=int, nargs="*", default=[2, 3], help="Default target IDs")
    parser.add_argument(
        "--available-targets",
        type=int,
        nargs="*",
        default=[2, 3, 4, 5, 6, 7],
        help="IDs that should be available for selection in the GUI",
    )
    parser.add_argument("--no-gui", action="store_true", help="Run without the Tk dashboard")
    return parser.parse_args()


def _build_config(args: argparse.Namespace) -> ParallelConfig2:
    targets = tuple(int(t) for t in args.targets) if args.targets else (2, 3)
    available = tuple(int(t) for t in args.available_targets) if args.available_targets else (2, 3, 4, 5, 6, 7)
    return ParallelConfig2(
        simulink_ip=args.simulink_ip,
        bind_ip=args.bind_ip,
        send_port=50001,
        ack_port=50002,
        send_hz=args.send_hz,
        ack_timeout_s=args.ack_timeout,
        angle_tol_deg=args.angle_tol,
        dist_tol_mm=args.dist_tol,
        camera_index=args.camera_index,
        available_target_ids=available,
        default_target_ids=targets,
    )


def main() -> None:
    args = _parse_args()
    config = _build_config(args)
    runtime = ParallelRuntime2(config)

    def _shutdown(signum=None, frame=None):  # pragma: no cover - signal hook
        runtime.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        runtime.start()
    except Exception:
        runtime.stop()
        raise

    if args.no_gui:
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            runtime.stop()
            return

    gui = create_gui(runtime.state_store, runtime.mission, runtime.start, runtime.robot_stop, runtime.stop)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
