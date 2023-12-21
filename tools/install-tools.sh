#!/bin/bash

set -eu


pip3 install --user xmltodict flake8 click==8.0.2 safety pydocstyle

pip3 install https://github.com/anetczuk/mdlinkscheck/archive/master.zip#subdirectory=src

sudo aptitude -y install black pycodestyle pylint bandit mypy
