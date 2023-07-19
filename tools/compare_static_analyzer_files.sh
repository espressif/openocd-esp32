#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

out_file="tmp_out_file.txt"
ref_file="tmp_ref_file.txt"

cat "$1" | grep -v "jimtcl" | sort > $out_file
cat "$2" | grep -v "jimtcl" | sort > $ref_file

diff_output=$(diff -B $out_file $ref_file)

warning_count=$(wc -l < "$out_file")
reference_warning_count=$(wc -l < "$ref_file")
if [ "$warning_count" -gt "$reference_warning_count" ]
then
    echo "Test failed! Warning count increased from $reference_warning_count to $warning_count"
fi
new_warnings=$(echo "$diff_output" | grep '^<' | wc -l)
removed_warnings=$(echo "$diff_output" | grep '^>' | wc -l)
ret=0
if [ "$removed_warnings" -gt "0" ]
then
    echo "${removed_warnings} warnings resolved. Don't forget to update the reference file with artifact"
    echo "$diff_output" | grep '^>'
    ret=1
fi
if [ "$new_warnings" -gt "0" ]
then
    echo "${new_warnings} new warnings found"
    echo "$diff_output" | grep '^<'
    ret=1
fi
if [ "$#" -eq "3" ]
then
    cat $out_file > "$3"
fi
rm $out_file
rm $ref_file
exit $ret
