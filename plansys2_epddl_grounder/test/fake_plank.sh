#!/bin/sh
# A stand-in for `plank export`, so the grounder's own behaviour — argument
# construction, output discovery, caching, failure reporting — is tested
# without needing the real toolchain built. It accepts the same flags and
# writes the canned task in FAKE_PLANK_TASK to <output dir>/<problem>.json.
#
# FAKE_PLANK_FAIL=1 makes it behave the way plank does on a bad specification:
# print a diagnostic, write nothing, and die on a signal rather than exiting.

problem=""
outdir=""

while [ $# -gt 0 ]; do
  case "$1" in
    -p) problem="$2"; shift 2 ;;
    -o) outdir="$2"; shift 2 ;;
    -d) shift 2 ;;
    -l)
      shift
      while [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; do shift; done
      ;;
    *) shift ;;
  esac
done

echo "fake plank: problem=${problem} outdir=${outdir}"

if [ "${FAKE_PLANK_FAIL}" = "1" ]; then
  echo "Syntax error: invalid keyword identifier ':predicatesX'."
  kill -SEGV $$
fi

[ -n "${outdir}" ] || exit 1
mkdir -p "${outdir}"

stem=$(basename "${problem}" .epddl)
cp "${FAKE_PLANK_TASK}" "${outdir}/${stem}.json"
