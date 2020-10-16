#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
import os
import sys
import argparse
import cv2


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('dir1', help="directory 1")
    parser.add_argument('dir2', help="directory 2")
    parser.add_argument('output_dir', help="output directory")

    return parser.parse_args()


def main():
    args = get_args()
    print(args)
    if not os.path.exists(args.dir1):
        print('invalid dir1: {}'.format(args.dir1))
        return
    if not os.path.exists(args.dir2):
        print('invalid dir2: {}'.format(args.dir2))
        return
    if not os.path.exists(args.output_dir):
        os.system('mkdir -p {}'.format(args.output_dir))
    img_file_list1 = []
    img_file_list2 = []
    for fname in os.listdir(args.dir1):
        img_file_list1.append(os.path.join(args.dir1, fname))
    for fname in os.listdir(args.dir2):
        img_file_list2.append(os.path.join(args.dir2, fname))
    print('img_file_list1 size: {}'.format(len(img_file_list1)))
    print('img_file_list2 size: {}'.format(len(img_file_list2)))
    img_file_list1.sort()
    img_file_list2.sort()

    for f1, f2 in zip(img_file_list1, img_file_list2):
        #print('{} {}'.format(os.path.basename(f1), os.path.basename(f2)))
        fname1 = os.path.splitext(os.path.basename(f1))[0]
        fname2 = os.path.splitext(os.path.basename(f2))[0]
        if fname1 != fname2:
            print('inconsistent filenames: {} {}'.format(fname1, fname2))
            return
        img1 = cv2.imread(f1)
        img2 = cv2.imread(f2)
        img_hconcat = cv2.hconcat([img1, img2])
        #cv2.imshow('img_hconcat', img_hconcat)
        #cv2.waitKey(0)
        output_fname = os.path.join(args.output_dir, '{}.png'.format(fname1))
        cv2.imwrite(output_fname, img_hconcat)


if __name__ == '__main__':
    sys.exit(main())
