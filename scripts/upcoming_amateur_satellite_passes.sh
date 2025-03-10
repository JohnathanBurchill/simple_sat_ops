#!/usr/bin/env bash

(cd ${HOME}/src/simple_sat_ops
next_in_queue satellites/amateur.tle 0 1000 --list --reverse 
)
