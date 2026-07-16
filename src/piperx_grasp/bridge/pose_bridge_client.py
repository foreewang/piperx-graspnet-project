from __future__ import annotations

import json
import math
import socket
import time
import uuid
from dataclasses import dataclass
from threading import Event
from typing import Any

import numpy as np

from piperx_grasp.calibration.transforms import (
    load_matrix,
    matrix_to_quaternion_xyzw,
    xyz_rpy_to_matrix,
)
from piperx_grasp.robot.piperx_client import PiperXClient


PROTOCOL_NAME = "piperx_pose_bridge"
PROTOCOL_VERSION = 1
MAX_MESSAGE_BYTES = 65_536


class PoseProviderError(RuntimeError):
    code = "INTERNAL_ERROR"
    retryable = False


class RobotNotConnectedError(PoseProviderError):
    code = "ROBOT_NOT_CONNECTED"
    retryable = True


class RobotMovingError(PoseProviderError):
    code = "ROBOT_MOVING"
    retryable = True


class PoseUnavailableError(PoseProviderError):
    code = "POSE_UNAVAILABLE"
    retryable = True


class FrameUnsupportedError(PoseProviderError):
    code = "FRAME_UNSUPPORTED"


def common_message(message_type: str) -> dict[str, Any]:
    return {
        "protocol": PROTOCOL_NAME,
        "version": PROTOCOL_VERSION,
        "type": message_type,
        "message_id": str(uuid.uuid4()),
        "sent_at_unix_ns": time.time_ns(),
    }


class JsonLineSocket:
    def __init__(self, connection: socket.socket) -> None:
        self.connection = connection
        self._buffer = bytearray()

    def send(self, message: dict[str, Any]) -> None:
        payload = (
            json.dumps(message, separators=(",", ":"), ensure_ascii=True) + "\n"
        ).encode("utf-8")
        if len(payload) > MAX_MESSAGE_BYTES:
            raise ValueError("protocol message exceeds 65536 bytes")
        self.connection.sendall(payload)

    def receive(self) -> dict[str, Any]:
        while True:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                raw = bytes(self._buffer[:newline])
                del self._buffer[: newline + 1]
                if not raw:
                    continue
                message = json.loads(raw.decode("utf-8"))
                if not isinstance(message, dict):
                    raise ValueError("protocol message must be a JSON object")
                return message
            if len(self._buffer) > MAX_MESSAGE_BYTES:
                raise ValueError("protocol message exceeds 65536 bytes")
            chunk = self.connection.recv(4096)
            if not chunk:
                raise ConnectionError("WSL pose bridge server disconnected")
            self._buffer.extend(chunk)


@dataclass
class PiperFeedbackPoseProvider:
    client: PiperXClient
    flange_to_gripper_base: np.ndarray
    stopped_velocity_threshold_rad_s: float = 0.01
    stability_wait_timeout_s: float = 1.5
    poll_interval_s: float = 0.05

    @classmethod
    def from_config(
        cls,
        client: PiperXClient,
        config: dict[str, Any],
    ) -> "PiperFeedbackPoseProvider":
        feedback = config.get("feedback", {})
        return cls(
            client=client,
            flange_to_gripper_base=load_matrix(
                config["frames"]["T_flange_gripper_base"]
            ),
            stopped_velocity_threshold_rad_s=float(
                feedback.get("stopped_velocity_threshold_rad_s", 0.01)
            ),
            stability_wait_timeout_s=float(
                feedback.get("stability_wait_timeout_s", 1.5)
            ),
            poll_interval_s=float(feedback.get("poll_interval_s", 0.05)),
        )

    def capture(self, request: dict[str, Any]) -> dict[str, Any]:
        if not self.client.is_connected():
            raise RobotNotConnectedError("PiperX is not connected")
        if request.get("required_parent_frame") != "base_link":
            raise FrameUnsupportedError("only base_link is supported as parent")
        if request.get("required_child_frame") != "gripper_base":
            raise FrameUnsupportedError("only gripper_base is supported as child")

        required_stable_ms = int(request.get("minimum_stable_for_ms", 300))
        initial_pose_feedback = self.client.get_flange_pose_feedback()
        initial_pose_timestamp = (
            None
            if initial_pose_feedback is None
            else initial_pose_feedback[1]
        )
        deadline = time.monotonic() + self.stability_wait_timeout_s
        stable_since: float | None = None
        latest_velocity = math.inf
        waiting_for_fresh_pose = False
        missing_feedback: list[str] = []
        while time.monotonic() < deadline:
            status = self.client.get_arm_status()
            velocities = self.client.get_joint_velocities()
            joint_positions = self.client.get_joint_angles()
            missing_feedback = []
            if status is None:
                missing_feedback.append("arm_status")
            if velocities is None:
                missing_feedback.append("joint_velocities")
            if joint_positions is None:
                missing_feedback.append("joint_angles")
            if missing_feedback:
                stable_since = None
                time.sleep(self.poll_interval_s)
                continue
            latest_velocity = max(abs(float(value)) for value in velocities)
            stopped = (
                getattr(status, "motion_status", None) == 0
                and latest_velocity <= self.stopped_velocity_threshold_rad_s
            )
            now = time.monotonic()
            if stopped:
                if stable_since is None:
                    stable_since = now
                stable_for_ms = int((now - stable_since) * 1000.0)
                if stable_for_ms >= required_stable_ms:
                    pose_feedback = self.client.get_flange_pose_feedback()
                    fresh_pose = bool(
                        pose_feedback is not None
                        and (
                            initial_pose_timestamp is None
                            or pose_feedback[1] > initial_pose_timestamp
                        )
                    )
                    if fresh_pose:
                        return self._capture_pose(
                            stable_for_ms,
                            latest_velocity,
                            pose_feedback,
                            joint_positions,
                        )
                    waiting_for_fresh_pose = True
            else:
                stable_since = None
            time.sleep(self.poll_interval_s)
        if missing_feedback:
            raise PoseUnavailableError(
                "robot feedback unavailable: " + ", ".join(missing_feedback)
            )
        if waiting_for_fresh_pose:
            raise PoseUnavailableError(
                "no fresh flange feedback arrived after the pose request"
            )
        raise RobotMovingError(
            "robot feedback did not remain stopped for "
            f"{required_stable_ms}ms; max_velocity={latest_velocity:.6f}rad/s"
        )

    def _capture_pose(
        self,
        stable_for_ms: int,
        maximum_velocity: float,
        flange_feedback: tuple[Any, float],
        joint_positions: list[float],
    ) -> dict[str, Any]:
        flange_pose, _feedback_timestamp = flange_feedback
        observed_at = time.time_ns()
        base_to_flange = xyz_rpy_to_matrix(
            [float(value) for value in flange_pose]
        )
        base_to_gripper = base_to_flange @ self.flange_to_gripper_base
        return {
            "observed_at_unix_ns": observed_at,
            "pose": {
                "parent_frame": "base_link",
                "child_frame": "gripper_base",
                "translation_m": base_to_gripper[:3, 3].tolist(),
                "rotation_xyzw": matrix_to_quaternion_xyzw(base_to_gripper),
            },
            "robot_state": {
                "source": "actual_feedback",
                "motion_state": "stopped",
                "stable_for_ms": stable_for_ms,
                "max_abs_joint_velocity_rad_s": maximum_velocity,
                "joint_names": [
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6",
                ],
                "joint_positions_rad": [
                    float(value) for value in joint_positions
                ],
            },
        }


@dataclass
class PoseBridgeClient:
    provider: PiperFeedbackPoseProvider
    host: str = "127.0.0.1"
    port: int = 8765
    client_id: str = "windows-piperx-controller"
    robot_id: str = "piperx-001"
    connect_timeout_s: float = 3.0
    reconnect_delay_s: float = 1.0

    @classmethod
    def from_config(
        cls,
        provider: PiperFeedbackPoseProvider,
        config: dict[str, Any],
    ) -> "PoseBridgeClient":
        network = config.get("network", {})
        identity = config.get("identity", {})
        return cls(
            provider=provider,
            host=str(network.get("host", "127.0.0.1")),
            port=int(network.get("port", 8765)),
            client_id=str(
                identity.get("client_id", "windows-piperx-controller")
            ),
            robot_id=str(identity.get("robot_id", "piperx-001")),
            connect_timeout_s=float(network.get("connect_timeout_s", 3.0)),
            reconnect_delay_s=float(network.get("reconnect_delay_s", 1.0)),
        )

    def run_forever(self, stop_event: Event | None = None) -> None:
        stop = stop_event or Event()
        while not stop.is_set():
            try:
                self._run_connection(stop)
            except (ConnectionError, OSError, ValueError) as error:
                if not stop.is_set():
                    print(f"pose bridge disconnected: {error}; reconnecting...")
                    stop.wait(self.reconnect_delay_s)

    def _run_connection(self, stop: Event) -> None:
        with socket.create_connection(
            (self.host, self.port),
            timeout=self.connect_timeout_s,
        ) as connection:
            connection.settimeout(None)
            stream = JsonLineSocket(connection)
            hello = {
                **common_message("hello"),
                "client_id": self.client_id,
                "robot_id": self.robot_id,
                "capabilities": ["get_robot_pose"],
            }
            stream.send(hello)
            ack = stream.receive()
            self._validate_ack(ack, hello["message_id"])
            print(
                f"pose bridge connected to {self.host}:{self.port} "
                f"as {self.robot_id}"
            )
            sequence = 0
            while not stop.is_set():
                request = stream.receive()
                if request.get("type") != "get_robot_pose":
                    self._send_error(
                        stream,
                        request,
                        "INVALID_REQUEST",
                        "only get_robot_pose is supported",
                        False,
                    )
                    continue
                sequence += 1
                self._handle_pose_request(stream, request, sequence)

    def _handle_pose_request(
        self,
        stream: JsonLineSocket,
        request: dict[str, Any],
        sequence: int,
    ) -> None:
        try:
            self._validate_request(request)
            feedback = self.provider.capture(request)
            response = {
                **common_message("robot_pose"),
                "reply_to": request["message_id"],
                "robot_id": self.robot_id,
                "sequence": sequence,
                **feedback,
            }
            stream.send(response)
        except PoseProviderError as error:
            self._send_error(
                stream,
                request,
                error.code,
                str(error),
                error.retryable,
            )
        except (KeyError, TypeError, ValueError) as error:
            self._send_error(
                stream,
                request,
                "INVALID_REQUEST",
                str(error),
                False,
            )

    @staticmethod
    def _validate_ack(message: dict[str, Any], hello_id: str) -> None:
        if message.get("protocol") != PROTOCOL_NAME:
            raise ValueError("invalid hello_ack protocol")
        if message.get("version") != PROTOCOL_VERSION:
            raise ValueError("unsupported hello_ack version")
        if message.get("type") != "hello_ack":
            raise ValueError("expected hello_ack")
        if message.get("reply_to") != hello_id:
            raise ValueError("hello_ack reply_to mismatch")

    @staticmethod
    def _validate_request(message: dict[str, Any]) -> None:
        if message.get("protocol") != PROTOCOL_NAME:
            raise ValueError("invalid protocol")
        if message.get("version") != PROTOCOL_VERSION:
            raise ValueError("unsupported protocol version")
        uuid.UUID(str(message["message_id"]))
        if message.get("require_stopped") is not True:
            raise ValueError("require_stopped must be true")
        if int(message["minimum_stable_for_ms"]) < 0:
            raise ValueError("minimum_stable_for_ms must not be negative")
        if int(message["maximum_pose_age_ms"]) <= 0:
            raise ValueError("maximum_pose_age_ms must be positive")

    @staticmethod
    def _send_error(
        stream: JsonLineSocket,
        request: dict[str, Any],
        code: str,
        message: str,
        retryable: bool,
    ) -> None:
        stream.send({
            **common_message("error"),
            "reply_to": str(request.get("message_id", "")),
            "code": code,
            "message": message,
            "retryable": retryable,
        })
