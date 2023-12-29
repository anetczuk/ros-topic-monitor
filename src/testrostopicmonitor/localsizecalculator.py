#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

from dict2obj import Dict2Obj

from rostopicmonitor.sizecalculator import AbstractCalculator, CalculatorFactory


## =====================================================


def generate_type(subtype_name_list):
    list_size = len(subtype_name_list)
    fields_list = []
    for i in range(0, list_size):
        fields_list.append(f"f{i}")
    data = {"_slot_types": subtype_name_list, "__slots__": fields_list}
    return Dict2Obj(data)


def generate_data(values_list):
    list_size = len(values_list)
    data = {}
    for i in range(0, list_size):
        data[f"f{i}"] = values_list[i]
    return Dict2Obj(data)


def generate_calculator(msg_class) -> AbstractCalculator:
    factory = LocalCalculatorFactory()
    return factory.generate(msg_class)


class LocalCalculatorFactory(CalculatorFactory):
    def getTypeByName(self, type_name):
        return None

    def getBaseTypeSize(self, type_name):
        return TYPE_SIZES.get(type_name)


TYPE_SIZES = {"int32": 4, "int64": 8}
