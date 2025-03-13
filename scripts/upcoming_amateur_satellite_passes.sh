#!/usr/bin/env bash

satellite="$1"

(cd ${HOME}/src/simple_sat_ops
if [[ -n "${satellite}" ]]; then
    next_in_queue satellites/amateur.tle 0 1000 "${satellite}" --list --reverse --show-radio-info
else
    next_in_queue satellites/amateur.tle 0 1000 --list --reverse --show-radio-info
fi
)
