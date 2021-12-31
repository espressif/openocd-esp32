#!/usr/bin/env python3

import os
import sys
import json 
import argparse
import jsonschema

ROOT_PATH = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

def board_config_file_check(tools_json):
    tcl_dir = os.path.join(ROOT_PATH, 'tcl')

    for each_board in tools_json["boards"]:
        for each_config_file in each_board["config_files"]:
            
            file_full_path = os.path.join(tcl_dir, each_config_file)
            if not os.path.isfile(file_full_path):
                print("Error. {}: file not found".format(each_config_file), file=sys.stderr)
                raise SystemExit(1)

def action_validate(config_file, config_schema_file):
    tools_json = json.load(config_file)
    schema_json = json.load(config_schema_file)

    jsonschema.validate(tools_json, schema_json)
    board_config_file_check(tools_json)

def main():
    parser = argparse.ArgumentParser(description="esp-config-validator.py - Validate esp-config file on json")

    parser.add_argument("--json", "-j",
                        help="esp-config file path including file e.g. 'tcl/esp-config.json'",
                        type=argparse.FileType('r'),
                        default=os.path.join(ROOT_PATH, 'tcl', 'esp-config.json'))

    parser.add_argument("--schema", "-s",
                        help="esp-config file schema path including file e.g. 'tcl/esp-config-schema.json'",
                        type=argparse.FileType('r'),
                        default=os.path.join(ROOT_PATH, 'tcl', 'esp-config-schema.json'))

    args = parser.parse_args()

    config_file =  args.json
    config_schema_file = args.schema

    try:
        action_validate(config_file, config_schema_file)
    except jsonschema.exceptions.ValidationError as err:
        print(err, file=sys.stderr)
        raise SystemExit(1)
    
if __name__ == '__main__':
    main()
