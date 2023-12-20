#!/bin/bash

##
## Script installs copy of package into Python's user directory.
##

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


echo "Installing package $SCRIPT_DIR"

cd "$SCRIPT_DIR"


## creates "*.egg-info" and "build" directory along package dir

if [[ $* == *--venv* ]]; then
    pip3 install .
else
    pip3 install --user .
fi
