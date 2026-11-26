#!/usr/bin/env bash
# Dependency gate (docs/design/01-architecture.md §4).
# Every rosdep key declared by the packages under <path> must resolve to the
# apt installer with an install candidate. Run BEFORE installing anything, so
# an already-installed key can't hide a pip-only or missing one.
# Usage: check_deps.sh <path> <rosdistro> [known_missing.txt] [skip_keys_out]
set -euo pipefail
path=${1:?path}; distro=${2:?rosdistro}; known_file=${3:-}; skip_out=${4:-/dev/null}
fail=0
known=""
if [ -n "$known_file" ] && [ -f "$known_file" ]; then
  known=$(awk -v d="$distro" '$1 == d {print $2}' "$known_file")
fi
: > "$skip_out"
# No -i: with -i rosdep also drops every key already installed under
# AMENT_PREFIX_PATH, which is exactly what this gate must not do. Drop only the
# packages that live in this repo.
own=$(python3 -c "from catkin_pkg.packages import find_packages as f; print('\n'.join(p.name for p in f('$path').values()))")
keys=$(rosdep keys --from-paths "$path" --rosdistro "$distro" | sort -u | grep -vxF "$own")
[ -n "$keys" ] || { echo "no external keys found"; exit 1; }
for key in $keys; do
  is_known=0
  if printf '%s\n' "$known" | grep -qxF "$key"; then is_known=1; fi
  if ! out=$(rosdep resolve "$key" --rosdistro "$distro" 2>&1); then
    if [ $is_known = 1 ]; then
      echo "::warning::$key: known missing on $distro (ci/known_missing.txt)"
      echo "$key" >> "$skip_out"; continue
    fi
    echo "::error::$key: rosdep cannot resolve it"; echo "$out"; fail=1; continue
  fi
  installer=$(echo "$out" | sed -n 's/^#//p' | head -1)
  pkgs=$(echo "$out" | grep -v '^#' | xargs)
  if [ "$installer" != "apt" ]; then
    echo "::error::$key resolves via '$installer', not apt"; fail=1; continue
  fi
  missing=0
  for pkg in $pkgs; do
    cand=$(apt-cache policy "$pkg" | sed -n 's/^ *Candidate: //p')
    if [ -z "$cand" ] || [ "$cand" = "(none)" ]; then
      missing=1
      [ $is_known = 1 ] || { echo "::error::$key -> $pkg has no apt install candidate"; fail=1; }
    else
      echo "ok  $key -> $pkg ($cand)"
    fi
  done
  if [ $is_known = 1 ]; then
    if [ $missing = 1 ]; then
      echo "::warning::$key: known missing on $distro (ci/known_missing.txt)"
      echo "$key" >> "$skip_out"
    else
      echo "::error::$key is listed in known_missing.txt for $distro but is now available; remove the line"
      fail=1
    fi
  fi
done
exit $fail
