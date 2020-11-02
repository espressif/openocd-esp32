#!/usr/bin/env python

import os
import subprocess
import re
import argparse

def main(repo_path, comp):
	try:
		os.chdir(repo_path)
		ver_out = subprocess.check_output(['git', 'describe', '--tags'], stderr=subprocess.STDOUT).decode('utf-8')
	except:
		print ('0')
		return
	m = re.search(r'(?P<major>\d+)(\.(?P<minor>\d+)(\.(?P<bugfix>\d+)(\.(?P<update>\d+))?)?)?', ver_out)
	if not m:
		print ('0')
		return
	if comp == "pack2hex":
		major = int(m.group('major')) if m.group('major') else 0
		minor = int(m.group('minor')) if m.group('minor') else 0
		bugfix = int(m.group('bugfix')) if m.group('bugfix') else 0
		update = int(m.group('update')) if m.group('update') else 0
		print ("0x{0:x}".format(((major & 0xFF) << 24) | ((minor & 0xFF) << 16) | ((bugfix & 0xFF) << 8) | (update & 0xFF)))
	elif m.group(comp):
		print( m.group(comp))
	else:
		print ('0')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='get_idf_ver.py - Retrieves version of IDF', prog='get_idf_ver')

    parser.add_argument('--component', '-c',
                        help='Version string component. Using format: <major>[.<minor>[.<bugfix>[.<update>]]]',
                        choices=['major', 'minor', 'bugfix', 'update', 'pack2hex'],
                        default='major')
    parser.add_argument('git_path', help='Path to GIT repo', type=str, default=os.getenv("IDF_PATH"))
    args = parser.parse_args()
    main(args.git_path, args.component)
