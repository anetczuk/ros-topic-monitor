#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


SRC_DIR="$SCRIPT_DIR/../src"


generate_help() {
    echo "generating help output"

    EXE_NAME="rostopicmon.py"
    HELP_PATH=$SCRIPT_DIR/cmdargs.md
    
    echo "## ${EXE_NAME} --help" > ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
    $SRC_DIR/$EXE_NAME --help >> ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
}


generate_help


$SCRIPT_DIR/generate_small.sh
