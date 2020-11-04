#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 INPUT_FILE OUTPUT_FILE"
    echo "  e.g. $0 ./input.txt ./output.txt"
    exit 1
fi

INPUT_FILE=$1
OUTPUT_FILE=$2

# define a function to print args.
foo() {
    echo "1st arg: $1"
    echo "2nd arg: $2"
}

foo $INPUT_FILE $OUTPUT_FILE

