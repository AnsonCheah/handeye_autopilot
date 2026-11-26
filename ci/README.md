# CI-only material

Not released. The `*_IGNORE` markers in this directory keep colcon, ament and
bloom/catkin_pkg from treating anything here as part of the repo's packages.

- `scripts/check_deps.sh`: the dependency gate (design §4 of 01-architecture).
- `interface_consumer/`: a downstream package that only depends on
  `handeye_autopilot_interfaces`. CI copies it into a separate workspace on
  top of the install space and runs its tests (M0 done-when).
