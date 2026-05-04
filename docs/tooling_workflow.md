# ScoutSLAM Tooling Workflow

Docker is the canonical build and runtime environment for this module. Use the repository wrapper instead of running host-side CMake or ROS 2 commands directly.

## Local Check

Run the implementation quality loop from the ScoutSLAM repository root:

```bash
./scripts/check_scoutslam.sh
```

By default this uses:

```text
/opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/my_records.yaml
```

To pass another container-visible parameter file:

```bash
./scripts/check_scoutslam.sh /opt/OA-SLAM/ros2/oaslam_ros2_wrapper/config/another_record.yaml
```

To verify only the C++ and ROS 2 builds:

```bash
SCOUTSLAM_SKIP_OFFLINE_RUN=1 ./scripts/check_scoutslam.sh
```

The offline run is headless by default. The script creates a temporary ignored params copy with `use_viewer: false` so automated checks do not require X11. To run with the selected params file exactly as written:

```bash
SCOUTSLAM_HEADLESS=0 ./scripts/check_scoutslam.sh
```

The script performs:

- Docker wrapper/container availability check
- `./docker/run.sh rebuild-cpp`
- `./docker/run.sh rebuild-ros2`
- `./docker/run.sh run-offline <params-file>`

## Engineering Focus

Keep the current code direction conservative:

- use ORB-SLAM3/OA-SLAM as the local UAV visual-semantic backbone
- use ROS 2 semantic map topics as the integration boundary
- add focused tests around config parsing, semantic map message conversion, and target verification before changing ORB-SLAM3 internals
- prioritize reproducible offline bag runs over broad architecture rewrites

The thesis SOTA position is best supported by reliable object-level semantic evidence, multi-agent map fusion, and map-grounded target verification rather than by adding open-vocabulary mapping unless the perception stack is explicitly changed.

## GitHub Connector Status

The GitHub connector is available for `Vor-Art/OA-SLAM-docker`, so PR review, issue triage, branch comparison, and CI debugging can use the GitHub plugin for this implementation repository.
