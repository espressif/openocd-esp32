#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

since=${1:-origin/master}
commit_count=$(git rev-list --count "${since}..HEAD")
if [ "$commit_count" -eq 0 ]; then
    echo "No new commits between '${since}' and 'master'."
    exit 0
fi
tools/scripts/checkpatch.pl --git ${since}..HEAD
