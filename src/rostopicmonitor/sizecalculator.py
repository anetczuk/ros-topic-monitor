#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import logging
from typing import Dict
from abc import ABC, abstractmethod


_LOGGER = logging.getLogger(__name__)


## =====================================================


class AbstractCalculator(ABC):
    @abstractmethod
    def isFixed(self) -> bool:
        """Return 'True' if calculator represents fixed-sized data, otherwise 'False'."""
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def getFixedSize(self) -> int:
        """Return size of represented data."""
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def calculate(self, field_data) -> int:
        """Calculate size of given data."""
        raise NotImplementedError("You need to define this method in derived class!")


## =====================================================


class ConstCalculator(AbstractCalculator):
    def __init__(self, type_size: int):
        self.type_size: int = type_size

    def isFixed(self) -> bool:
        return True

    def getFixedSize(self) -> int:
        return self.type_size

    def calculate(self, field_data) -> int:
        return self.type_size


# array of types of fixed-length size
class FixedArrayCalculator(AbstractCalculator):
    def __init__(self, type_size: int):
        self.type_size: int = type_size

    def isFixed(self) -> bool:
        return False

    def getFixedSize(self) -> int:
        return self.type_size

    def calculate(self, field_data) -> int:
        return self.type_size * len(field_data) + 4  # +4 for array length


# array of types of variable size
class VarArrayCalculator(AbstractCalculator):
    def __init__(self, type_calc: AbstractCalculator):
        self.type_calc: AbstractCalculator = type_calc

    def isFixed(self) -> bool:
        return False

    def getFixedSize(self) -> int:
        return self.type_calc.getFixedSize()

    def calculate(self, field_data) -> int:
        # field_data is an array
        field_size: int = 0
        for value in field_data:
            field_size += self.type_calc.calculate(value)
        return field_size + 4  # +4 for array length


class StructCalculator(AbstractCalculator):
    def __init__(self):
        self.const_size: int = 0  # total size of non-variadic fields
        self.field_calc: Dict[str, AbstractCalculator] = {}  # subcalculators for variadic fields

    def isFixed(self) -> bool:
        # field_calc: AbstractCalculator
        for field_calc in self.field_calc.values():
            if not field_calc.isFixed():
                return False
        return True

    def getFixedSize(self) -> int:
        fixed_size = self.const_size
        # field_calc: AbstractCalculator
        for field_calc in self.field_calc.values():
            fixed_size += field_calc.getFixedSize()
        return fixed_size

    def addConstSize(self, new_size: int):
        self.const_size += new_size

    def addSubcalc(self, field_name, calc: AbstractCalculator):
        self.field_calc[field_name] = calc

    def calculate(self, field_data) -> int:
        field_size = self.const_size
        # field_calc: AbstractCalculator
        for field_name, field_calc in self.field_calc.items():
            subfield_value = getattr(field_data, field_name)
            field_size += field_calc.calculate(subfield_value)
        return field_size


## =====================================================


class CalculatorFactory(ABC):
    def generate(self, type_object) -> AbstractCalculator:
        member_types_list = type_object._slot_types  # pylint: disable=W0212
        member_vars_list = type_object.__slots__  # pylint: disable=W0212

        struct_calc = StructCalculator()

        for field_index, field_type_name in enumerate(member_types_list):
            calc = self.generateByName(field_type_name)
            if not calc:
                continue

            if calc.isFixed():
                # reduce calculator
                const_size = calc.getFixedSize()
                struct_calc.addConstSize(const_size)
                continue

            field_name = member_vars_list[field_index]
            struct_calc.addSubcalc(field_name, calc)

        if struct_calc.isFixed():
            # reduce calculator
            const_size = struct_calc.getFixedSize()
            return ConstCalculator(const_size)

        return struct_calc

    def generateByName(self, type_name) -> AbstractCalculator:
        type_size = self.getBaseTypeSize(type_name)
        if type_size is not None:
            return ConstCalculator(type_size)

        if type_name == "string":
            return FixedArrayCalculator(1)

        if type_name.endswith("[]"):
            # +4 bytes for array length
            arr_type_name = type_name[: len(type_name) - 2]
            arr_type_calc = self.generateByName(arr_type_name)
            if not arr_type_calc:
                return None
            if arr_type_calc.isFixed():
                type_size = arr_type_calc.getFixedSize()
                return FixedArrayCalculator(type_size)
            return VarArrayCalculator(arr_type_calc)

        try:
            subtype = self.getTypeByName(type_name)
            # subtype = roslib.message.get_message_class(type_name)
            return self.generate(subtype)
        except:  # noqa pylint: disable=W0702
            _LOGGER.warning("unhandled type: %s", type_name)

        return None

    @abstractmethod
    def getTypeByName(self, type_name):
        """Return type by it's name."""
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def getBaseTypeSize(self, type_name):
        """Return size of type by it's name."""
        raise NotImplementedError("You need to define this method in derived class!")
