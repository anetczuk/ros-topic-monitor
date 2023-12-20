#!/bin/bash

#set -eu
set -u


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


src_dir=$SCRIPT_DIR/../src
examples_dir=$SCRIPT_DIR/../examples


echo "running black"
black --line-length=120 $src_dir $examples_dir $SCRIPT_DIR
exit_code=$?

if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi

echo "black -- no warnings found"


## E115 intend of comment
## E126 continuation line over-indented for hanging indent
## E201 whitespace after '('
## E202 whitespace before ')'
## E221 multiple spaces before equal operator
## E241 multiple spaces after ':'
## E262 inline comment should start with '# '
## E265 block comment should start with '# '
## E266 too many leading '#' for block comment
## E402 module level import not at top of file
## E501 line too long (80 > 79 characters)
## W391 blank line at end of file
## D    all docstyle checks
ignore_errors=E115,E126,E201,E202,E221,E241,E262,E265,E266,E402,E501,W391,D


echo "running pycodestyle"
echo "to ignore warning inline add comment at the end of line: # noqa"
pycodestyle --show-source --statistics --count --ignore=$ignore_errors $src_dir $examples_dir $SCRIPT_DIR
exit_code=$?

if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi

echo "pycodestyle -- no warnings found"


## F401 'PyQt5.QtCore' imported but unused
ignore_errors=$ignore_errors,F401


echo "running flake8"
python3 -m flake8 --show-source --statistics --count --ignore=$ignore_errors $src_dir $examples_dir $SCRIPT_DIR
exit_code=$?

if [ $exit_code -ne 0 ]; then
    echo -e "\nflake8 errors found"
    exit $exit_code
fi

echo "flake8 -- no warnings found"


src_files=$(find "$src_dir" -type f -name "*.py")
example_files=$(find "$examples_dir" -type f -name "*.py")

echo "running pylint3"
echo "to ignore warning for module put following line on top of file: # pylint: disable=<check_id>"
echo "to ignore warning for one line put following comment in end of line: # pylint: disable=<check_id>"
pylint --rcfile=$SCRIPT_DIR/pylint3.config $src_files $example_files
exit_code=$?
if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi
echo "pylint3 -- no warnings found"


echo "running bandit"
echo "to ignore warning for one line put following comment in end of line: # nosec"

## [B301:blacklist] Pickle and modules that wrap it can be unsafe when used to deserialize untrusted data, possible security issue.
## [B403:blacklist] Consider possible security implications associated with pickle module.
skip_list="B301,B403"

#echo "to ignore warning for one line put following comment in end of line: # nosec
bandit --skip "${skip_list}" -r $src_dir $example_files $SCRIPT_DIR
exit_code=$?
if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi
echo "bandit -- no warnings found"


echo "running safety"
safety check -r $src_dir/requirements.txt
exit_code=$?
if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi
echo "safety -- no warnings found"
