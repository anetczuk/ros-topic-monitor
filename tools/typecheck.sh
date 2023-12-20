#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


CACHE_DIR=$SCRIPT_DIR/../tmp/.mypy_cache


cd $SCRIPT_DIR/../src


examples_dir=$SCRIPT_DIR/../examples
all_examples=$(find "$examples_dir" -type f -name "*.py")


echo "running mypy"
echo "ignore line warning using: # type: ignore"
mypy --cache-dir $CACHE_DIR --no-strict-optional --ignore-missing-imports --pretty . $all_examples

echo "mypy finished"
