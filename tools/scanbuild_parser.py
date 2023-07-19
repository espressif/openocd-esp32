#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

from bs4 import BeautifulSoup
import argparse
import os
import sys

ROOT_PATH = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

def main():
    with open(args.input, "r") as html_content:
        soup = BeautifulSoup(html_content, 'html.parser')
        tbody = soup.find('tbody')
        rows = tbody.find_all('tr')

        with open(args.output, "w") as out:
            for row in rows:
                tds = row.find_all('td')
                td_values = [td.get_text(strip=True) for td in tds][2:5]
                out.write("%s\n" % ":".join(td_values))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Scan build output parser', prog='Scan build output parser')
    parser.add_argument('--input', '-i',
                        help='Scan build output file name to be parsed',
                        default="index.html")
    parser.add_argument('--output', '-o',
                        help='Output file to be write parsed result',
                        default=os.path.join(ROOT_PATH, "scanbuild_report.txt"))

    args = parser.parse_args()
    main()
