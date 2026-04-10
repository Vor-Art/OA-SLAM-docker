#!/usr/bin/env python3
"""
Run OA-SLAM from docker-compose using a small YAML config file.
"""
import os
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path

import yaml

PROJECT_ROOT = Path("/opt/OA-SLAM")
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "docker" / "run_config.yaml"
DEFAULT_CPP_INSTALL_PREFIX = Path(os.environ.get("OASLAM_CPP_INSTALL_PREFIX", "/opt/oaslam_artifacts/cpp/install"))
NONE_VALUES = {"", "none", "null", "None", "NULL", None}
GENERIC_INPUT_DIRS = {"color", "rgb", "images", "image", "frames"}
ALLOWED_RELOCALIZATION_MODES = {"points", "objects", "points+objects"}


def load_config() -> dict:
    config_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CONFIG_PATH
    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config must contain a YAML mapping: {config_path}")
    return data


def as_optional_string(value) -> str:
    if value in NONE_VALUES:
        return "none"
    return str(value).strip()


def resolve_path(value, *, allow_none: bool = False) -> str:
    raw = as_optional_string(value)
    if raw == "none":
        if allow_none:
            return "none"
        raise ValueError("Missing required path value")
    path = Path(raw)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return str(path)


def validate_existing_path(label: str, value, *, allow_none: bool = False) -> str:
    resolved = resolve_path(value, allow_none=allow_none)
    if resolved == "none":
        return resolved
    if not Path(resolved).exists():
        raise FileNotFoundError(f"{label} does not exist: {resolved}")
    return resolved


def resolve_image_source(value) -> str:
    raw = as_optional_string(value)
    if raw.startswith("webcam"):
        return raw
    return validate_existing_path("Image source", raw)


def derive_run_name(image_source: str, run_name: str) -> str:
    if run_name:
        return run_name
    if image_source.startswith("webcam"):
        suffix = image_source.replace(":", "_")
    else:
        path = Path(image_source)
        suffix = path.stem if path.suffix else path.name
        if suffix in GENERIC_INPUT_DIRS and path.parent.name:
            suffix = path.parent.name
    return f"mapping_{suffix or 'run'}"


def resolve_executable() -> Path:
    candidates = [
        DEFAULT_CPP_INSTALL_PREFIX / "bin" / "oa-slam",
        PROJECT_ROOT / "build" / "bin" / "oa-slam",
        PROJECT_ROOT / "bin" / "oa-slam",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        "OA-SLAM executable does not exist. Build it inside the container first with: ./docker/run.sh rebuild-cpp"
    )


def build_command(config: dict) -> tuple[list[str], Path]:
    vocabulary = validate_existing_path("Vocabulary", config.get("vocabulary"))
    camera_config = validate_existing_path("Camera config", config.get("camera_config"))
    image_source = resolve_image_source(config.get("image_source"))
    detections = validate_existing_path("Detections", config.get("detections", "none"), allow_none=True)
    ignored_categories = validate_existing_path(
        "Ignored categories file", config.get("ignored_categories", "none"), allow_none=True
    )

    relocalization_mode = str(config.get("relocalization_mode", "points+objects")).strip()
    if relocalization_mode not in ALLOWED_RELOCALIZATION_MODES:
        raise ValueError("relocalization_mode must be 'points', 'objects', or 'points+objects'")

    output_root = Path(resolve_path(config.get("output_root", "Data/runs")))
    output_root.mkdir(parents=True, exist_ok=True)
    run_name = "" if config.get("run_name") in NONE_VALUES else str(config.get("run_name")).strip()
    output_dir = output_root / f"{derive_run_name(image_source, run_name)}_{datetime.now():%Y%m%d_%H%M%S}"
    output_dir.mkdir(parents=True, exist_ok=True)

    executable = resolve_executable()
    command = [
        str(executable),
        vocabulary,
        camera_config,
        image_source,
        detections,
        ignored_categories,
        relocalization_mode,
        str(output_dir),
    ]

    return command, output_dir


def forward_stream(stream, destination) -> None:
    for line in stream:
        if "Attribute name doesn't exist for program" not in line:
            destination.write(line)
            destination.flush()


def main() -> int:
    os.environ["MESA_GL_VERSION_OVERRIDE"] = "3.3"
    os.environ["MESA_GLSL_VERSION_OVERRIDE"] = "330"

    try:
        command, output_dir = build_command(load_config())
    except Exception as exc:
        print(f"Launcher configuration error: {exc}", file=sys.stderr)
        return 1

    print(f"Running OA-SLAM with output directory: {output_dir}")
    print(f"Command: {' '.join(command)}")
    print("-" * 80, flush=True)

    try:
        process = subprocess.Popen(
            command,
            cwd=PROJECT_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1,
        )
    except Exception as exc:
        print(f"Failed to start OA-SLAM: {exc}", file=sys.stderr)
        return 1

    stdout_thread = threading.Thread(target=forward_stream, args=(process.stdout, sys.stdout))
    stderr_thread = threading.Thread(target=forward_stream, args=(process.stderr, sys.stderr))
    stdout_thread.start()
    stderr_thread.start()

    return_code = process.wait()
    stdout_thread.join()
    stderr_thread.join()

    print("-" * 80, flush=True)
    if return_code == 0:
        print(f"OA-SLAM completed successfully. Output saved to: {output_dir}", flush=True)
    else:
        print(f"OA-SLAM exited with code: {return_code}", file=sys.stderr)
    return return_code


if __name__ == "__main__":
    sys.exit(main())
