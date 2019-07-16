#!/usr/bin/env python

import os
import subprocess
import re
import argparse

def main(repo_path, comp):
	try:
		os.chdir(repo_path)
		ver_out = subprocess.check_output(['git', 'describe', '--tags'], stderr=subprocess.STDOUT)
	except:
		print '0'
		return
	m = re.search(r'(?P<major>\d+)(\.(?P<minor>\d+)(\.(?P<bugfix>\d+)(\.(?P<update>\d+))?)?)?', ver_out)
	if m and m.group(comp):
		print m.group(comp)
	else:
		print '0'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='get_idf_ver.py - Retrieves version of IDF', prog='get_idf_ver')

    parser.add_argument('--component', '-c',
                        help='Version string component. Using format: <major>[.<minor>[.<bugfix>[.<update>]]]',
                        choices=['major', 'minor', 'bugfix', 'update'],
                        default='major')
    parser.add_argument('git_path', help='Path to GIT repo', type=str, default=os.getenv("IDF_PATH"))
    args = parser.parse_args()
    main(args.git_path, args.component)
