#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/doc/generate-doc.sh

$SCRIPT_DIR/tools/mdpreproc.py $SCRIPT_DIR/README.md

# run tests in venv (it verifies required packages)
if [[ $* == *--no-venv* ]]; then
    # skip venv step - run tests locally 
    $SCRIPT_DIR/src/testrostopicmonitor/runtests.py
else
    $SCRIPT_DIR/tools/configure-venv.sh --no-prompt
    $SCRIPT_DIR/venv/runtests.sh
fi

$SCRIPT_DIR/tools/checkall.sh


echo "processing completed"
