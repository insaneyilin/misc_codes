# -*- coding: utf-8 -*-
# @Author: insaneyilin
# Python version 2.7

import os
import sys
import time
import argparse
import multiprocessing
from PIL import Image


def get_args():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('src_dir', type=str, help='source directory')
    arg_parser.add_argument('dest_dir', type=str, help='destination directory')
    arg_parser.add_argument('width', type=int, help='resize image width')
    arg_parser.add_argument('height', type=int, help='resize image height')
    arg_parser.add_argument('--image_ext', type=str, help='image file extension',
            default=".jpg")
    arg_parser.add_argument('--pnum', type=int, help='number of processes',
            default=4)
    return arg_parser.parse_args()


def get_out_filename(in_file, dest_dir):
    dir_path, filename = os.path.split(in_file)
    # fname, ext = os.path.splitext(filename)
    return os.path.join(dest_dir, filename)


def get_filepaths(image_dir, ext='.jpg'):
    img_paths = [os.path.abspath(os.path.join(image_dir, x)) \
        for x in os.listdir(image_dir) \
            if os.path.isfile(os.path.abspath(os.path.join(image_dir, x))) and \
                os.path.splitext(x)[1] == ext]
    return img_paths


def resize_image(in_file_path, out_dir, w, h):
    img = Image.open(in_file_path)
    new_img = img.resize((w, h), Image.ANTIALIAS)
    out_filepath = get_out_filename(in_file_path, out_dir)
    new_img.save(out_filepath)


# wrapper to call Pool.map()
def resize_func(tup):
    resize_image(*tup)


if __name__ == '__main__':
    start = time.time()
    args = get_args()
    filepaths = get_filepaths(args.src_dir, args.image_ext)
    pool = multiprocessing.Pool(args.pnum)

    pool.map(resize_func, [(fpath, args.dest_dir, args.width, args.height) \
        for fpath in filepaths])
    pool.close()
    pool.join()
    print('elapsed time: {}'.format(time.time() - start))
