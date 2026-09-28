#!/bin/bash
#
# Build Debian packages of eplansys for ROS 2 Humble, and an apt repository of
# them.
#
#   packaging/build-debs.sh <workspace> <output directory>
#
# The workspace is one `vcs import src < dependency_repos.repos` produced, with
# this repository at src/eplansys. Run it in a ros:humble container: it
# installs every package it builds, since the next one builds against it.
#
# The PlanSys2 packages here are a fork, released under the upstream names at
# version 3.0.0. Upstream Humble ships 2.0.9, so apt prefers these once the
# repository is added. cascade_lifecycle is built too, from the rolling-devel
# checkout the repos file pins: plansys2_executor links a target Humble's 1.1.0
# does not export, and at 2.0.6 it is preferred over it the same way. popf
# comes from upstream as it is.

set -euo pipefail

WS=$(realpath "$1")
OUT=$(realpath -m "$2")
DISTRO=humble
mkdir -p "$OUT"

# The setup script reads variables it does not define.
set +u
# shellcheck disable=SC1091
source /opt/ros/$DISTRO/setup.bash
set -u
export DEBIAN_FRONTEND=noninteractive
export DEB_BUILD_OPTIONS=nocheck

cd "$WS"

# plank is a plain CMake project, and bloom packages only what has a
# package.xml. The grounder runs plank without linking it, which is why its
# package.xml does not name it; an installed grounder needs it all the same.
cp src/eplansys/packaging/plank.package.xml src/plank/package.xml
grep -q '<exec_depend>plank</exec_depend>' src/eplansys/plansys2_epddl_grounder/package.xml ||
  sed -i 's|<depend>plansys2_pddl_parser</depend>|&\n  <exec_depend>plank</exec_depend>|' \
    src/eplansys/plansys2_epddl_grounder/package.xml

PACKAGES=$(colcon list --topological-order --names-only \
  --packages-up-to eplansys eplansys_demo plank \
  --packages-skip popf)

# rosdep resolves what each package depends on. The packages built here have to
# resolve to the Debian names bloom gives them, and those that upstream also
# ships already do; aletheia and plank are known to no one else.
ROSDEP=/etc/ros/rosdep/sources.list.d/00-eplansys.list
{
  for name in $PACKAGES; do
    echo "$name:"
    echo "  ubuntu: [ros-$DISTRO-${name//_/-}]"
  done
} > "$OUT/eplansys-rosdep.yaml"
echo "yaml file://$OUT/eplansys-rosdep.yaml" > "$ROSDEP"
rosdep update --rosdistro $DISTRO >/dev/null

for name in $PACKAGES; do
  path=$(colcon list --packages-select "$name" --paths-only)

  # A package already in the output was built by an earlier run over the same
  # sources, and is only installed again for what depends on it.
  built=$(ls "$OUT"/ros-$DISTRO-"${name//_/-}"_*.deb 2>/dev/null | head -1 || true)
  if [ -n "$built" ]; then
    echo "=== $name (already built)"
    apt-get install -y -q "$built" >/dev/null
    continue
  fi
  echo "=== $name ($path)"

  rosdep install --from-paths "$path" --ignore-src -y --rosdistro $DISTRO -q

  (
    cd "$path"
    rm -rf debian obj-*
    bloom-generate rosdebian --ros-distro $DISTRO >/dev/null
    fakeroot debian/rules binary >/dev/null
  )

  # apt reads an argument as a file only when it is a path it can tell apart
  # from a package name, hence the absolute one.
  deb=$(realpath "$(ls -t "$(dirname "$path")"/ros-$DISTRO-"${name//_/-}"_*.deb | head -1)")
  apt-get install -y -q "$deb" >/dev/null
  mv "$deb" "$OUT"/
  rm -rf "$path/debian" "$path"/obj-* "$(dirname "$path")"/*.ddeb
done

# A flat repository: the packages and their index side by side, which apt reads
# from any URL that serves the directory, a GitHub release included.
cd "$OUT"
dpkg-scanpackages --multiversion . /dev/null > Packages
gzip -9kf Packages
apt-ftparchive release . > Release
ls -1 ./*.deb | wc -l
