#!/usr/bin/env python3
"""Detect and mitigate trajectory jumps in TUM-format camera trajectory files.

This script reads a TUM-format trajectory file (timestamp tx ty tz qx qy qz qw),
detects frames where the pose changes abruptly (indicating map resets, relocalization
snaps, or other discontinuities), and replaces those poses with a zero/identity marker
so downstream consumers can recognize and handle them.

Jump detection criteria:
  - Translational delta between consecutive frames exceeds a threshold (default 0.5m)
  - Rotational delta between consecutive frames exceeds a threshold (default 30 degrees)

For each detected jump frame, the output contains:
  timestamp 0 0 0 0 0 0 1
(zero position, identity quaternion)

Usage:
  python3 detect_trajectory_jumps.py input.txt output.txt [--trans-thresh 0.5] [--rot-thresh 30]
  python3 detect_trajectory_jumps.py input.txt  # prints to stdout
  python3 detect_trajectory_jumps.py input.txt --report-only  # only prints jump report
"""

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Optional, TextIO


@dataclass
class TumPose:
    """A single pose in TUM format."""
    timestamp: float
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float
    line_number: int  # 1-based line number in the input file


def parse_tum_file(filepath: str) -> List[TumPose]:
    """Parse a TUM-format trajectory file, skipping comments and blank lines."""
    poses = []
    with open(filepath, "r") as f:
        for line_num, line in enumerate(f, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 8:
                print(
                    f"WARNING: Line {line_num}: expected 8 fields, got {len(parts)}. Skipping.",
                    file=sys.stderr,
                )
                continue
            try:
                pose = TumPose(
                    timestamp=float(parts[0]),
                    tx=float(parts[1]),
                    ty=float(parts[2]),
                    tz=float(parts[3]),
                    qx=float(parts[4]),
                    qy=float(parts[5]),
                    qz=float(parts[6]),
                    qw=float(parts[7]),
                    line_number=line_num,
                )
                poses.append(pose)
            except ValueError as e:
                print(
                    f"WARNING: Line {line_num}: failed to parse floats: {e}. Skipping.",
                    file=sys.stderr,
                )
    return poses


def quaternion_angle_deg(
    qx1: float, qy1: float, qz1: float, qw1: float,
    qx2: float, qy2: float, qz2: float, qw2: float,
) -> float:
    """Compute the angular distance (degrees) between two unit quaternions.

    Uses the formula: angle = 2 * acos(|q1 · q2|)
    """
    dot = qx1 * qx2 + qy1 * qy2 + qz1 * qz2 + qw1 * qw2
    # Clamp to [-1, 1] to handle numerical errors
    dot = max(-1.0, min(1.0, dot))
    angle_rad = 2.0 * math.acos(abs(dot))
    return math.degrees(angle_rad)


def translation_distance(p1: TumPose, p2: TumPose) -> float:
    """Euclidean distance between two pose positions."""
    dx = p2.tx - p1.tx
    dy = p2.ty - p1.ty
    dz = p2.tz - p1.tz
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def rotation_distance_deg(p1: TumPose, p2: TumPose) -> float:
    """Angular distance (degrees) between two pose orientations."""
    return quaternion_angle_deg(
        p1.qx, p1.qy, p1.qz, p1.qw,
        p2.qx, p2.qy, p2.qz, p2.qw,
    )


@dataclass
class JumpInfo:
    """Information about a detected trajectory jump."""
    frame_index: int  # 0-based index in the pose list
    line_number: int  # 1-based line number in the input file
    timestamp: float
    trans_delta: float  # meters
    rot_delta: float  # degrees
    reason: str  # "translation", "rotation", or "both"


def detect_jumps(
    poses: List[TumPose],
    trans_thresh: float = 0.5,
    rot_thresh: float = 30.0,
) -> List[JumpInfo]:
    """Detect trajectory jumps by thresholding frame-to-frame deltas.

    Args:
        poses: List of TUM poses in chronological order.
        trans_thresh: Maximum allowed translational delta (meters).
        rot_thresh: Maximum allowed rotational delta (degrees).

    Returns:
        List of JumpInfo for each detected jump.
    """
    jumps = []
    for i in range(1, len(poses)):
        prev = poses[i - 1]
        curr = poses[i]

        # Skip if previous pose is already a zero marker
        if prev.tx == 0 and prev.ty == 0 and prev.tz == 0 and abs(prev.qw - 1.0) < 1e-6:
            continue

        trans_d = translation_distance(prev, curr)
        rot_d = rotation_distance_deg(prev, curr)

        is_trans_jump = trans_d > trans_thresh
        is_rot_jump = rot_d > rot_thresh

        if is_trans_jump or is_rot_jump:
            if is_trans_jump and is_rot_jump:
                reason = "both"
            elif is_trans_jump:
                reason = "translation"
            else:
                reason = "rotation"

            jumps.append(JumpInfo(
                frame_index=i,
                line_number=curr.line_number,
                timestamp=curr.timestamp,
                trans_delta=trans_d,
                rot_delta=rot_d,
                reason=reason,
            ))

    return jumps


def print_report(
    poses: List[TumPose],
    jumps: List[JumpInfo],
    out: TextIO = sys.stderr,
) -> None:
    """Print a human-readable report of detected jumps."""
    print(f"\n{'='*70}", file=out)
    print(f"  Trajectory Jump Detection Report", file=out)
    print(f"{'='*70}", file=out)
    print(f"  Total poses:    {len(poses)}", file=out)
    print(f"  Jumps detected: {len(jumps)}", file=out)

    if poses:
        duration = poses[-1].timestamp - poses[0].timestamp
        print(f"  Time span:      {duration:.3f} s", file=out)
        print(f"  First timestamp: {poses[0].timestamp:.6f}", file=out)
        print(f"  Last timestamp:  {poses[-1].timestamp:.6f}", file=out)

    if jumps:
        print(f"\n  {'#':>3}  {'Line':>6}  {'Timestamp':>16}  {'Trans(m)':>10}  {'Rot(deg)':>10}  {'Reason'}", file=out)
        print(f"  {'---':>3}  {'------':>6}  {'----------------':>16}  {'--------':>10}  {'--------':>10}  {'------'}", file=out)
        for idx, j in enumerate(jumps, start=1):
            print(
                f"  {idx:>3}  {j.line_number:>6}  {j.timestamp:>16.6f}  {j.trans_delta:>10.4f}  {j.rot_delta:>10.2f}  {j.reason}",
                file=out,
            )
    else:
        print(f"\n  No trajectory jumps detected.", file=out)

    print(f"{'='*70}\n", file=out)


def write_cleaned_trajectory(
    poses: List[TumPose],
    jumps: List[JumpInfo],
    out: TextIO,
) -> None:
    """Write the cleaned trajectory, replacing jump frames with zero/identity poses."""
    jump_indices = {j.frame_index for j in jumps}

    out.write("# TUM trajectory format: timestamp tx ty tz qx qy qz qw\n")
    out.write("# Jump frames replaced with zero position and identity quaternion\n")

    for i, pose in enumerate(poses):
        if i in jump_indices:
            # Zero position, identity quaternion
            out.write(
                f"{pose.timestamp:.6f} 0.000000000 0.000000000 0.000000000 "
                f"0.000000000 0.000000000 0.000000000 1.000000000\n"
            )
        else:
            out.write(
                f"{pose.timestamp:.6f} {pose.tx:.9f} {pose.ty:.9f} {pose.tz:.9f} "
                f"{pose.qx:.9f} {pose.qy:.9f} {pose.qz:.9f} {pose.qw:.9f}\n"
            )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Detect and mitigate trajectory jumps in TUM-format trajectory files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "input",
        help="Input TUM trajectory file path",
    )
    parser.add_argument(
        "output",
        nargs="?",
        default=None,
        help="Output cleaned trajectory file path (default: stdout)",
    )
    parser.add_argument(
        "--trans-thresh",
        type=float,
        default=0.5,
        help="Translational jump threshold in meters (default: 0.5)",
    )
    parser.add_argument(
        "--rot-thresh",
        type=float,
        default=30.0,
        help="Rotational jump threshold in degrees (default: 30.0)",
    )
    parser.add_argument(
        "--report-only",
        action="store_true",
        help="Only print the jump detection report; do not write cleaned trajectory",
    )

    args = parser.parse_args()

    # Parse input
    poses = parse_tum_file(args.input)
    if not poses:
        print("ERROR: No valid poses found in input file.", file=sys.stderr)
        sys.exit(1)

    # Detect jumps
    jumps = detect_jumps(poses, args.trans_thresh, args.rot_thresh)

    # Print report
    print_report(poses, jumps)

    if args.report_only:
        return

    # Write cleaned trajectory
    if args.output:
        with open(args.output, "w") as f:
            write_cleaned_trajectory(poses, jumps, f)
        print(f"Cleaned trajectory written to: {args.output}", file=sys.stderr)
        print(
            f"  {len(jumps)} jump frames replaced with zero/identity poses.",
            file=sys.stderr,
        )
    else:
        write_cleaned_trajectory(poses, jumps, sys.stdout)


if __name__ == "__main__":
    main()
