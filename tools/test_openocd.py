#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

import subprocess
import sys

def main(ocd_bin):
    version = subprocess.check_output(f'{ocd_bin} --version', stderr=subprocess.STDOUT, shell=True)
    print(version.decode('UTF-8'))
    return not b'Open On-Chip Debugger v0.12.0' in version

if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
