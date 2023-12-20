#
# Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

import logging

from testrostopicmonitor.localsizecalculator import generate_calculator as generate_calculator_mock, generate_type


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

        calculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertTrue(calculator.isFixed())
        self.assertEqual(0, calculator.getFixedSize())

    def test_generate_calculator_base(self):
        MessageClass = generate_type(["int32"])

        calculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertTrue(calculator.isFixed())
        self.assertEqual(4, calculator.getFixedSize())

    def test_generate_calculator_base_array(self):
        MessageClass = generate_type(["int32[]"])

        calculator = generate_calculator_mock(MessageClass)
        self.assertTrue(calculator)
        self.assertFalse(calculator.isFixed())
        self.assertEqual(4, calculator.getFixedSize())
