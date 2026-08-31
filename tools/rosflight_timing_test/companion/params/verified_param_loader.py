#!/usr/bin/env python3
"""Load a ROSflight parameter YAML file with acknowledgment-backed verification."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.client import Client
from rclpy.node import Node
from rosflight_msgs.srv import ParamGet, ParamSet


@dataclass(frozen=True)
class Parameter:
    name: str
    mav_type: int
    value: float


@dataclass(frozen=True)
class ReadResult:
    exists: bool
    value: float | None
    error: str | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Set ROSflight parameters sequentially and verify each changed value "
            "through /param_get."
        )
    )
    parser.add_argument("filename", type=Path, help="ROSflight parameter YAML file")
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="validate YAML syntax and parameter structure without starting ROS",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=3,
        help="maximum /param_set attempts for each changed parameter (default: 3)",
    )
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=1.5,
        help="seconds to wait for acknowledgment-backed readback per attempt (default: 1.5)",
    )
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=15.0,
        help="seconds to wait for rosflight_io to discover every file parameter (default: 15)",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for ROS services and individual calls (default: 5)",
    )
    args = parser.parse_args()

    if args.retries < 1:
        parser.error("--retries must be at least 1")
    for option in ("ack_timeout", "discovery_timeout", "service_timeout"):
        if getattr(args, option) <= 0.0:
            parser.error(f"--{option.replace('_', '-')} must be greater than zero")

    return args


def load_parameters(filename: Path) -> list[Parameter]:
    try:
        with filename.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load {filename}: {exc}") from exc

    if not isinstance(document, list):
        raise ValueError(f"{filename}: top-level YAML value must be a sequence")

    parameters: list[Parameter] = []
    seen: set[str] = set()
    for index, item in enumerate(document, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"{filename}: entry {index} must be a mapping")

        missing = [key for key in ("name", "type", "value") if key not in item]
        if missing:
            raise ValueError(
                f"{filename}: entry {index} is missing {', '.join(missing)}"
            )

        name = item["name"]
        mav_type = item["type"]
        value = item["value"]
        if not isinstance(name, str) or not name:
            raise ValueError(f"{filename}: entry {index} has an invalid name")
        if name in seen:
            raise ValueError(f"{filename}: duplicate parameter {name}")
        if isinstance(mav_type, bool) or not isinstance(mav_type, int):
            raise ValueError(f"{filename}: {name} has a non-integer MAVLink type")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{filename}: {name} has a non-numeric value")

        numeric_value = float(value)
        if not math.isfinite(numeric_value):
            raise ValueError(f"{filename}: {name} has a non-finite value")

        seen.add(name)
        parameters.append(Parameter(name, mav_type, numeric_value))

    if not parameters:
        raise ValueError(f"{filename}: parameter sequence is empty")

    return parameters


def call_service(
    node: Node, client: Client, request: Any, timeout: float
) -> tuple[Any | None, str | None]:
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        future.cancel()
        return None, f"service call timed out after {timeout:g}s"
    try:
        response = future.result()
    except Exception as exc:  # ROS middleware exceptions are runtime-specific.
        return None, f"service call failed: {exc}"
    if response is None:
        return None, "service returned no response"
    return response, None


def read_parameter(
    node: Node, client: Client, name: str, timeout: float
) -> ReadResult:
    request = ParamGet.Request()
    request.name = name
    response, error = call_service(node, client, request, timeout)
    if error is not None:
        return ReadResult(False, None, error)
    if not response.exists:
        return ReadResult(False, None, "not known by rosflight_io")
    return ReadResult(True, float(response.value))


def set_parameter(
    node: Node, client: Client, parameter: Parameter, timeout: float
) -> str | None:
    request = ParamSet.Request()
    request.name = parameter.name
    request.value = parameter.value
    response, error = call_service(node, client, request, timeout)
    if error is not None:
        return error
    if not response.exists:
        return "not known by rosflight_io"
    return None


def values_match(parameter: Parameter, actual: float) -> bool:
    # MAV_PARAM_TYPE_REAL32 is 9. Integer values are transported losslessly in
    # ROSflight's bytewise MAVLink parameter encoding and should compare exactly.
    if parameter.mav_type == 9:
        return math.isclose(actual, parameter.value, rel_tol=1e-6, abs_tol=1e-6)
    return actual == parameter.value


def format_value(value: float | None) -> str:
    return "unavailable" if value is None else f"{value:.17g}"


def wait_for_discovery(
    node: Node,
    get_client: Client,
    parameters: list[Parameter],
    discovery_timeout: float,
    service_timeout: float,
) -> dict[str, ReadResult]:
    deadline = time.monotonic() + discovery_timeout
    previous_missing_count: int | None = None

    while True:
        missing: dict[str, ReadResult] = {}
        for parameter in parameters:
            result = read_parameter(node, get_client, parameter.name, service_timeout)
            if not result.exists:
                missing[parameter.name] = result

        if not missing:
            return {}
        if time.monotonic() >= deadline:
            return missing
        if len(missing) != previous_missing_count:
            print(
                f"Waiting for rosflight_io to discover {len(missing)} "
                "file parameter(s)...",
                file=sys.stderr,
            )
            previous_missing_count = len(missing)
        time.sleep(0.25)


def wait_for_value(
    node: Node,
    get_client: Client,
    parameter: Parameter,
    ack_timeout: float,
    service_timeout: float,
) -> ReadResult:
    deadline = time.monotonic() + ack_timeout
    last_result = ReadResult(False, None, "no readback received")
    while True:
        last_result = read_parameter(
            node,
            get_client,
            parameter.name,
            min(service_timeout, max(0.05, deadline - time.monotonic())),
        )
        if (
            last_result.exists
            and last_result.value is not None
            and values_match(parameter, last_result.value)
        ):
            return last_result
        if time.monotonic() >= deadline:
            return last_result
        time.sleep(0.05)


def verify_all(
    node: Node,
    get_client: Client,
    parameters: list[Parameter],
    service_timeout: float,
) -> list[str]:
    mismatches: list[str] = []
    for parameter in parameters:
        result = read_parameter(node, get_client, parameter.name, service_timeout)
        if result.value is None or not values_match(parameter, result.value):
            detail = result.error or f"actual {format_value(result.value)}"
            mismatches.append(
                f"{parameter.name}: expected {format_value(parameter.value)}, {detail}"
            )
    return mismatches


def run(args: argparse.Namespace) -> int:
    try:
        parameters = load_parameters(args.filename)
    except ValueError as exc:
        print(f"Parameter load failed: {exc}", file=sys.stderr)
        return 1

    if args.check_only:
        print(f"Valid parameter YAML: {args.filename} ({len(parameters)} parameters)")
        return 0

    rclpy.init()
    node = rclpy.create_node("verified_firmware_param_loader")
    get_client = node.create_client(ParamGet, "/param_get")
    set_client = node.create_client(ParamSet, "/param_set")

    try:
        for service_name, client in (
            ("/param_get", get_client),
            ("/param_set", set_client),
        ):
            if not client.wait_for_service(timeout_sec=args.service_timeout):
                print(
                    f"Parameter load failed: {service_name} was unavailable after "
                    f"{args.service_timeout:g}s",
                    file=sys.stderr,
                )
                return 2

        print(
            f"Preflighting {len(parameters)} parameter(s) from {args.filename}..."
        )
        missing = wait_for_discovery(
            node,
            get_client,
            parameters,
            args.discovery_timeout,
            args.service_timeout,
        )
        if missing:
            print(
                "Parameter load aborted before making changes; rosflight_io did not "
                "discover:",
                file=sys.stderr,
            )
            for name, result in missing.items():
                print(f"  - {name}: {result.error}", file=sys.stderr)
            return 3

        changed = 0
        unchanged = 0
        attempt_failures: dict[str, str] = {}
        for position, parameter in enumerate(parameters, start=1):
            current = read_parameter(
                node, get_client, parameter.name, args.service_timeout
            )
            if (
                current.value is not None
                and values_match(parameter, current.value)
            ):
                unchanged += 1
                continue

            old_value = format_value(current.value)
            succeeded = False
            last_error = current.error or "value did not match"
            for attempt in range(1, args.retries + 1):
                error = set_parameter(
                    node, set_client, parameter, args.service_timeout
                )
                if error is not None:
                    last_error = error
                else:
                    readback = wait_for_value(
                        node,
                        get_client,
                        parameter,
                        args.ack_timeout,
                        args.service_timeout,
                    )
                    if (
                        readback.value is not None
                        and values_match(parameter, readback.value)
                    ):
                        succeeded = True
                        changed += 1
                        print(
                            f"[{position}/{len(parameters)}] {parameter.name}: "
                            f"{old_value} -> {format_value(readback.value)}"
                        )
                        break
                    last_error = readback.error or (
                        f"read back {format_value(readback.value)}"
                    )

                if attempt < args.retries:
                    print(
                        f"Retrying {parameter.name} after attempt {attempt}: "
                        f"{last_error}",
                        file=sys.stderr,
                    )

            if not succeeded:
                attempt_failures[parameter.name] = last_error

        mismatches = verify_all(
            node, get_client, parameters, args.service_timeout
        )
        if mismatches:
            print(
                f"FAILED: {len(mismatches)} parameter(s) did not verify:",
                file=sys.stderr,
            )
            for mismatch in mismatches:
                name = mismatch.split(":", maxsplit=1)[0]
                attempt_detail = attempt_failures.get(name)
                suffix = (
                    f"; last set error: {attempt_detail}"
                    if attempt_detail is not None
                    else ""
                )
                print(f"  - {mismatch}{suffix}", file=sys.stderr)
            return 4

        print(
            f"PASS: verified all {len(parameters)} parameter(s) "
            f"({changed} changed, {unchanged} already matched)."
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
