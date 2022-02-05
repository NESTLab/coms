#!/bin/bash

URL="http://localhost:8080/vnc.html"
tries=500
timeout=1
i=0

while [ "$i" -le "$tries" ]
do
    if curl ${URL} > /dev/null 2>&1
    then
        echo "Health Check (${i}/${tries}): PASSED at $(date +%H:%M:%S)"
        exit 0
    fi
    echo "Health Check (${i}/${tries}): FAILED at $(date +%H:%M:%S)"
    sleep $timeout
    ((i=i+1))
done

exit 1
