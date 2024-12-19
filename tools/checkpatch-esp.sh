#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

since=${1:-origin/master}
tools/scripts/checkpatch.pl --git ${since}..
