#bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 AUTHOR_NAME"
    exit 1
fi

AUTHOR_NAME=$1

git log --author=$AUTHOR_NAME --pretty=tformat: --numstat \
| awk '{ add += $1; subs += $2; loc += $1 - $2 } END { printf "added lines: %s removed lines: %s total lines: %s\n", add, subs, loc }' -
