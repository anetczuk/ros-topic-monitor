#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


SRC_DIR="$SCRIPT_DIR/../src"


generate_tools_help() {
    HELP_PATH=$SCRIPT_DIR/cmdargs.md
    
    echo "## <a name=\"main_help\"></a> rostopicmon.py --help" > ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
    $SRC_DIR/rostopicmon.py --help >> ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
    
    
    tools=$($SRC_DIR/rostopicmon.py --listtools)
    
    IFS=', ' read -r -a tools_list <<< "$tools"
    
    
    for item in ${tools_list[@]}; do
        echo $item
        echo -e "\n\n" >> ${HELP_PATH}
        echo "## <a name=\"${item}_help\"></a> rostopicmon.py $item --help" >> ${HELP_PATH}
        echo -e "\`\`\`" >> ${HELP_PATH}
        $SRC_DIR/rostopicmon.py $item --help >> ${HELP_PATH}
        echo -e "\`\`\`"  >> ${HELP_PATH}
    done
}


generate_tools_help


$SCRIPT_DIR/generate_small.sh
