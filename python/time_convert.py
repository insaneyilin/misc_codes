#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from __future__ import print_function
import sys
import argparse
import datetime


def get_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input_str', help="timestamp(by default) or date string")
    parser.add_argument('-d', '--date2ts', action="store_true",
                        help="convert date to unix timestamp")

    return parser.parse_args()


def main():
    if sys.version_info < (3, 0):
        print('Need Python 3!')
        print('Current Python version:\n', sys.version)
        return 1
    args = get_args()
    if not args.date2ts:
        print(datetime.datetime.fromtimestamp(float(args.input_str)))
    else:
        # need datetime.datetime.timestamp() needs python3
        print(datetime.datetime.strptime(args.input_str, '%Y-%m-%d %H:%M:%S').timestamp())
    return 0


if __name__ == '__main__':
    sys.exit(main())
