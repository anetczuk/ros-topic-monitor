#
# Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

#
# Entry file when package is used as the main program.
# can be executed in following way:
#        python -m <package>
#

import sys

from rostopicmonitor.main import main


if __name__ == "__main__":
    EXIT_CODE = main()
    sys.exit(EXIT_CODE)
