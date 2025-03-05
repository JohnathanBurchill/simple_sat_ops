#!/usr/bin/env bash

(cd ${HOME}/src/simple_sat_ops
next_in_queue satellites/amateur_20250303.tle 0 1000 --list --reverse 
)
