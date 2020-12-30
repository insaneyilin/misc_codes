#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 XYZ_FILE"
    echo "  e.g. $0 ./0000.xyz"
    exit 1
fi

# use 'readlink -f' to get absolute path
XYZ_FILE=`readlink -f $1`

cloudcompare.ccViewer $XYZ_FILE
