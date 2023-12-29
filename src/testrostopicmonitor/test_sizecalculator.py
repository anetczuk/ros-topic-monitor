#
# Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

import logging

from testrostopicmonitor.localsizecalculator import (
    generate_calculator as generate_calculator_mock,
    generate_type,
    generate_data,
)
from rostopicmonitor.sizecalculator import AbstractCalculator


_LOGGER = logging.getLogger(__name__)


## =====================================================


class CalculatorTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_generate_calculator_empty(self):
        MessageClass = generate_type([])

        calculator: AbstractCalculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertTrue(calculator.isFixed())
        self.assertEqual(0, calculator.getFixedSize())

    def test_generate_calculator_base_string(self):
        MessageClass = generate_type(["string"])

        calculator: AbstractCalculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertFalse(calculator.isFixed())
        self.assertEqual(1, calculator.getFixedSize())

        msg_data = generate_data(["xxx"])
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(7, msg_size)

    def test_generate_calculator_base(self):
        MessageClass = generate_type(["int32"])

        calculator: AbstractCalculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertTrue(calculator.isFixed())
        self.assertEqual(4, calculator.getFixedSize())

        msg_data = generate_data([5])
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(4, msg_size)

    def test_generate_calculator_base_array_var_01(self):
        MessageClass = generate_type(["int32[]"])

        calculator: AbstractCalculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertFalse(calculator.isFixed())
        self.assertEqual(4, calculator.getFixedSize())

        msg_data = generate_data([[]])  # empty array
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(4, msg_size)

        msg_data = generate_data([[5, 4, 1, 2, 3]])
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(24, msg_size)

    def test_generate_calculator_base_array_var_02(self):
        MessageClass = generate_type(["int64[]"])

        calculator: AbstractCalculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertFalse(calculator.isFixed())
        self.assertEqual(8, calculator.getFixedSize())

        msg_data = generate_data([[]])  # empty array
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(4, msg_size)

        msg_data = generate_data([[5, 4, 1, 2, 3]])
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(44, msg_size)

    def test_generate_calculator_base_array_fixed_01(self):
        MessageClass = generate_type(["int32[5]"])

        calculator: AbstractCalculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertTrue(calculator.isFixed())
        self.assertEqual(20, calculator.getFixedSize())

        msg_data = generate_data([[]])  # empty array
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(20, msg_size)

        msg_data = generate_data([[5, 4, 1, 2, 3]])
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(20, msg_size)

    def test_generate_calculator_base_array_fixed_02(self):
        MessageClass = generate_type(["string[2]"])

        calculator: AbstractCalculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertFalse(calculator.isFixed())
        self.assertEqual(1, calculator.getFixedSize())

        msg_data = generate_data([["", ""]])  # empty array
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(8, msg_size)

        msg_data = generate_data([["123", "12"]])
        msg_size = calculator.calculate(msg_data)
        self.assertEqual(13, msg_size)
