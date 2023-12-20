#!/bin/bash

##
## Script installs package in "editable" mode (symlinks) into Python's user directory.
##

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


echo "Installing package $SCRIPT_DIR"


## creates "*.egg-info" directory along package dir

if [[ $* == *--venv* ]]; then
    pip3 install -e "$SCRIPT_DIR" 
else
    pip3 install --user -e "$SCRIPT_DIR" 
fi
