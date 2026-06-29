#!/usr/bin/env bash

tle="$1"

sat=$(next_in_queue --tle ${tle} | tail -n 1 | cut -c -26) && sat=${sat##*( )}
echo ${sat}
