#!/usr/bin/env bash

id=$(ssm --pretty trajectories 2>&1 | tail -1 | cut -c 1-36)

tle=$(ssm --pretty trajectory ${id} --export-tle 2>&1 | tail -1 | cut -c 14-)

next_in_queue SX-FRONTIERSAT --tle=${tle} --list --reverse

