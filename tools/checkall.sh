#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


cd $SCRIPT_DIR


./codecheck.sh
./doccheck.sh
./typecheck.sh

echo -e "\nchecking links in MD files"
./mdcheck.sh


echo "everything is fine"
