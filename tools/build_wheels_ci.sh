#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Build platform-specific wheels for every dist_name_* sidecar produced
# by the upstream build jobs. Invoked from .gitlab/ci/wheels.yml.

set -euo pipefail
shopt -s nullglob

DIST_ART_DIR="${DIST_ART_DIR:-dist}"
WHEELHOUSE="${WHEELHOUSE:-wheelhouse}"

rm -rf "${WHEELHOUSE}"
mkdir -p "${WHEELHOUSE}"

dist_files=( "${DIST_ART_DIR}"/dist_name_* )
if [ "${#dist_files[@]}" -eq 0 ]; then
	echo "ERROR: no ${DIST_ART_DIR}/dist_name_* files found"
	exit 1
fi

for dist_file in "${dist_files[@]}"; do
	platform="$(basename "${dist_file}" | sed 's/^dist_name_//')"
	archive="$(cat "${dist_file}")"
	echo "Building wheel for ${platform} from ${archive}"
	python tools/build_wheel.py "${DIST_ART_DIR}/${archive}" "${WHEELHOUSE}/"
done

ls -la "${WHEELHOUSE}/"

if [ -z "$(ls -A "${WHEELHOUSE}/")" ]; then
	echo "ERROR: no wheels built"
	exit 1
fi
