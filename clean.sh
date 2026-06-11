#!/usr/bin/env bash

find . -type f \( \
-name "*_BASE_*" -o \
-name "*_LOCAL_*" -o \
-name "*_REMOTE_*" -o \
-name "*_BACKUP_*" \
\) -delete


pkill -9 kdiff3 2>/dev/null
clear
sleep 1.5
echo "cleanup done"
