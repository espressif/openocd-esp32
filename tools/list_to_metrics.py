#!/usr/bin/env python3

import argparse
import os
import sys

OUTPUT_FILE_NAME = "metrics.txt"
ROOT_PATH = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

def get_values(file):
    values = []

    for line in file[4:]:
        if("=" in line):
            break
        else:
            info = line.replace(" ", "").replace("%", "%|").split("|")
            if (len(info) < 2):
                continue
            file_name, coverage_percentage = info[0], info[1]
            file_name = file_name.replace(ROOT_PATH, ".")
            values.append("{}".format(file_name + " " + coverage_percentage))

    return values


def write_to_metrics_file(values_for_write, path):
    with open(path, "a+") as file:
        for line in values_for_write:
            file.write("{}\n".format(line))


def main():
    parser = argparse.ArgumentParser(description="list_to_metrics.py - Convert coverage file output to metrics.txt file")
    parser.add_argument("--file", "-f",
                        help="Coverage output file's folder path, including file name",
                        type=argparse.FileType('r'),
                        default=os.path.join(ROOT_PATH, "cov_infos", "metrics_input.txt"))

    args = parser.parse_args()

    file_path, _ = os.path.split(args.file.name)
    metrics_path = os.path.join(file_path, OUTPUT_FILE_NAME)
    print("Metrics file will be written to: " + metrics_path)

    file_lines = args.file.readlines()
    values_for_write = get_values(file_lines)

    write_to_metrics_file(values_for_write, metrics_path)


if __name__ == '__main__':
    main()
