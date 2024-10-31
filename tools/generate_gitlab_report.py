#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

from bs4 import BeautifulSoup
import argparse
import hashlib
import json
import os
import sys

ROOT_PATH = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

def process_scanbuild_row(row):
    tds = row.find_all('td')
    td_values = [td.get_text(strip=True) for td in tds]
    description = td_values[1]
    path = td_values[2]
    row_num = int(td_values[4])
    return description, path, row_num

def process_sparse_row(row):
    tmp = row.split()[0]
    description = row[len(tmp) + 1:].strip()
    path = tmp.split(':')[0][3:]
    row_num = int(tmp.split(':')[1])
    return description, path, row_num

def main():
    with open(args.input, "r") as report:
        if args.type == 'sparse':
            rows = report.readlines()
        else:
            soup = BeautifulSoup(report, 'html.parser')
            tbody = soup.find('tbody')
            rows = tbody.find_all('tr')

    warnings = list()
    for row in rows:
        description, path, row_num = (process_sparse_row if args.type == 'sparse' else process_scanbuild_row)(row)
        warn = {
            "fingerprint" : hashlib.md5(row.encode()).hexdigest(),
            "check_name" : args.type,
            "description" : description,
            "severity" : "minor",
            "location" : {
                "path" : path,
                "lines" : {
                    "begin" : row_num
                }
            }
        }
        warnings.append(warn)

    with open(args.output, "w") as out:
        out.write(json.dumps(warnings, indent=4))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Gitlab report generator', prog='Gitlab report generator')
    parser.add_argument('--input', '-i',
                        help='Static analysis report file to source',
                        default="index.html")
    parser.add_argument('--output', '-o',
                        help='Output file',
                        default=os.path.join(ROOT_PATH, "report.json"))
    parser.add_argument('--type', '-t',
                        help='Report type',
                        choices=['sparse', 'scanbuild'],
                        required=True)

    args = parser.parse_args()
    main()
