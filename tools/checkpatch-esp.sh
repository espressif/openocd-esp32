#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

cnt=$(git rev-list --count HEAD ^origin/master)
git diff HEAD~${cnt} \
            | filterdiff \
                -x "a/src/jtag/drivers/libjaylink/*" \
                -x "a/tools/git2cl/*" \
                -x "a/.gitlab/*" \
                -x "a/HACKING" \
                -x "a/testing/esp/*" \
                -x "a/contrib/loaders/flash/esp/*" \
				-x "*/tools/scanbuild_reference.html" \
            | ./tools/scripts/checkpatch.pl --no-signoff --ignore STATIC_CONST_CHAR_ARRAY --ignore MACRO_ARG_PRECEDENCE
