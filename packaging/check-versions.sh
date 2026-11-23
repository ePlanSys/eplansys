#!/usr/bin/env bash
# Refuses a release whose packages changed under an unchanged version.
#
#   packaging/check-versions.sh [<previous-tag>] [<workspace-src-dir>]
#
# apt never replaces an installed package with one of the same version, so a
# release that ships changed contents under the version an earlier release
# used is one its users never receive. 0.3.0, 0.3.5 and 0.4.0 all shipped the
# PlanSys2 fork at 3.0.0, and users of the earlier ones kept the old
# plansys2_msgs under epistemic packages built against the new one.
#
# Three rules, checked against the previous release tag (the latest v* tag
# before HEAD unless one is given):
#
#   1. A package whose directory changed has a new version.
#   2. If plansys2_msgs changed, every package that depends on it has a new
#      version too: it is rebuilt against a different message layout.
#   3. A pinned source whose pin moved has a new version in the package.xml
#      it is built from: packaging/plank.package.xml for plank, and
#      del-planner's own for aletheia, read from its checkout under the
#      workspace src dir.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

prev=${1:-}
if [ -z "$prev" ]; then
  prev=$(git describe --tags --abbrev=0 --match 'v*' HEAD^ 2>/dev/null || true)
fi
src=${2:-}
if [ -z "$prev" ]; then
  echo "no previous release tag; nothing to compare against"
  exit 0
fi

version() { grep -oP '(?<=<version>)[^<]+' | head -1; }
fail=0

msgs_changed=0
git diff --quiet "$prev" HEAD -- plansys2_msgs || msgs_changed=1

while read -r xml; do
  dir=$(dirname "$xml")
  [ "$dir" = packaging ] && continue
  old=$(git show "$prev:$xml" 2>/dev/null | version || true)
  [ -n "$old" ] || continue                  # new since the previous release
  new=$(version < "$xml")
  [ "$new" = "$old" ] || continue
  if ! git diff --quiet "$prev" HEAD -- "$dir"; then
    echo "$dir changed since $prev but is still $new"
    fail=1
  elif [ "$msgs_changed" = 1 ] && [ "$dir" != plansys2_msgs ] &&
       grep -qE '<(build_|exec_)?depend>plansys2_msgs</' "$xml"; then
    echo "$dir depends on plansys2_msgs, which changed since $prev, but is still $new"
    fail=1
  fi
done < <(git ls-files '*package.xml' | grep -vE '(^|/)test/')

pin() {
  git show "$1:dependency_repos.repos" |
    awk -v repo="$2:" '$1 == repo { found = 1 } found && $1 == "version:" { print $2; exit }'
}

for repo in aletheia plank; do
  before=$(pin "$prev" "$repo")
  after=$(pin HEAD "$repo")
  [ "$before" = "$after" ] && continue
  case $repo in
    plank)
      old=$(git show "$prev:packaging/plank.package.xml" | version)
      new=$(version < packaging/plank.package.xml)
      ;;
    aletheia)
      if [ -z "$src" ] || [ ! -d "$src/aletheia/.git" ]; then
        echo "aletheia's pin moved since $prev; pass the workspace src dir to check its version"
        fail=1
        continue
      fi
      old=$(git -C "$src/aletheia" show "$before:package.xml" | version)
      new=$(git -C "$src/aletheia" show "$after:package.xml" | version)
      ;;
  esac
  if [ "$old" = "$new" ]; then
    echo "$repo's pin moved since $prev (${before:0:8} -> ${after:0:8}) but it is still $new"
    fail=1
  fi
done

if [ "$fail" = 0 ]; then
  echo "versions OK against $prev"
fi
exit "$fail"
