#
# Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

import logging

from rostopicmonitor.topicstats import RingValueBuffer, ListValueBuffer


_LOGGER = logging.getLogger(__name__)


## =====================================================


class ValueBufferTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_add_empty(self):
        buffer = ListValueBuffer()
        self.assertEqual(len(buffer), 0)
        self.assertEqual(buffer.min(), 0)
        self.assertEqual(buffer.max(), 0)
        self.assertEqual(buffer.sum(), 0)
        self.assertEqual(buffer.mean(), 0)
        self.assertEqual(buffer.stddev(), 0)

    def test_add_infinite(self):
        buffer = ListValueBuffer()
        buffer.add(6)
        buffer.add(10)
        buffer.add(20)
        self.assertEqual(len(buffer), 3)
        self.assertEqual(buffer.min(), 6)
        self.assertEqual(buffer.max(), 20)
        self.assertEqual(buffer.sum(), 36)
        self.assertEqual(buffer.mean(), 12)
        self.assertEqual(buffer.stddev(), 5.887840577551898)


class RingValueBufferTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_add_empty(self):
        buffer = RingValueBuffer()
        self.assertEqual(len(buffer), 0)
        self.assertEqual(buffer.min(), 0)
        self.assertEqual(buffer.max(), 0)
        self.assertEqual(buffer.sum(), 0)
        self.assertEqual(buffer.mean(), 0)
        self.assertEqual(buffer.stddev(), 0)

    def test_add_limited(self):
        buffer = RingValueBuffer(2)
        buffer.add(5)  # will be ignored
        buffer.add(10)
        buffer.add(20)
        self.assertEqual(len(buffer), 2)
        self.assertEqual(buffer.min(), 10)
        self.assertEqual(buffer.max(), 20)
        self.assertEqual(buffer.sum(), 30)
        self.assertEqual(buffer.mean(), 15)
        self.assertEqual(buffer.stddev(), 5.0)

    def test_add_limited_02(self):
        buffer = RingValueBuffer(3)
        buffer.add(2)  # will be ignored
        buffer.add(6)
        buffer.add(10)
        buffer.add(20)
        self.assertEqual(len(buffer), 3)
        self.assertEqual(buffer.min(), 6)
        self.assertEqual(buffer.max(), 20)
        self.assertEqual(buffer.sum(), 36)
        self.assertEqual(buffer.mean(), 12)
        self.assertEqual(buffer.stddev(), 5.887840577551898)
