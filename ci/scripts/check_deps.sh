#!/usr/bin/env bash
# Dependency gate (docs/design/01-architecture.md §4).
# Every rosdep key declared by the packages under $1 must resolve to the apt
# installer with an install candidate. Run BEFORE installing anything, so an
# already-installed key can't hide a pip-only or missing one.
# Usage: check_deps.sh <path> <rosdistro>
set -euo pipefail
path=${1:?path}; distro=${2:?rosdistro}
fail=0
# No -i: with -i rosdep also drops every key already installed under
# AMENT_PREFIX_PATH, which is exactly what this gate must not do. Drop only the
# packages that live in this repo.
own=$(python3 -c "from catkin_pkg.packages import find_packages as f; print('\n'.join(p.name for p in f('$path').values()))")
keys=$(rosdep keys --from-paths "$path" --rosdistro "$distro" | sort -u | grep -vxF "$own")
[ -n "$keys" ] || { echo "no external keys found"; exit 1; }
for key in $keys; do
  if ! out=$(rosdep resolve "$key" --rosdistro "$distro" 2>&1); then
    echo "::error::$key: rosdep cannot resolve it"; echo "$out"; fail=1; continue
  fi
  installer=$(echo "$out" | sed -n 's/^#//p' | head -1)
  pkgs=$(echo "$out" | grep -v '^#' | xargs)
  if [ "$installer" != "apt" ]; then
    echo "::error::$key resolves via '$installer', not apt"; fail=1; continue
  fi
  for pkg in $pkgs; do
    cand=$(apt-cache policy "$pkg" | sed -n 's/^ *Candidate: //p')
    if [ -z "$cand" ] || [ "$cand" = "(none)" ]; then
      echo "::error::$key -> $pkg has no apt install candidate"; fail=1
    else
      echo "ok  $key -> $pkg ($cand)"
    fi
  done
done
exit $fail
