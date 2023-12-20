#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os

import roslib

from rostopicmonitor.sizecalculator import AbstractCalculator, CalculatorFactory


## =====================================================


def generate_calculator(msg_class) -> AbstractCalculator:
    factory = ROSCalculatorFactory()
    return factory.generate(msg_class)


class ROSCalculatorFactory(CalculatorFactory):
    def getTypeByName(self, type_name):
        return roslib.message.get_message_class(type_name)

    def getBaseTypeSize(self, type_name):
        return BUILTIN_TYPE_SIZE.get(type_name)


# values based on http://wiki.ros.org/msg
BUILTIN_TYPE_SIZE = {
    "bool": 1,
    "byte": 1,
    "char": 1,

    "int8": 1,
    "uint8": 1,
    "int16": 2,
    "uint16": 2,
    "int32": 4,
    "uint32": 4,
    "int64": 8,
    "uint64": 8,

    "float32": 4,
    "float64": 8,

    "time": 8,
    "duration": 8
}
