#!/usr/bin/env bash

PROG=pathplanning
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PUSHED=0
if test -f $DIR/../$PROG; then
    pushd $DIR/..;
    PUSHED=1;
fi

./pathplanning -i $DIR/configs/narrowmazeconfig.xml -n 700 -f -d 20

if [ "$PUSHED" -eq "1" ]; then
   popd;
   exit;
fi
