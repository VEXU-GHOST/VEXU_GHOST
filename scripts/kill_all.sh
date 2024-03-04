#!/bin/bash

# List whatever keywords show up in process name
KEYWORDS=( gz ros )

# Iterate through keywords
for k in "${KEYWORDS[@]}"
do
    # Find all processes that match
    PID_STR=$(ps aux | grep $k  | awk '{print $2}')
    PID_LIST=($PID_STR)

    # Murder processes
    for PID in "${PID_LIST[@]}"
    do
      kill -9 $PID &> /dev/null
    done

done



