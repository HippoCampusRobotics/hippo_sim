#!/usr/bin/env python3
import xacro
import json
import sys
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mappings', required=True)
    parser.add_argument('--input', required=True)
    args = parser.parse_args()
    mappings = json.loads(args.mappings)
    doc = xacro.process_file(args.input, mappings=mappings)
    sys.stdout.write(doc.toxml())
    return 0


if __name__ == "__main__":
    main()
