#
# Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

import logging
import time

from rostopicmonitor.sizecalculator import AbstractCalculator
from rostopicmonitor.topicstats import TopicStats, TopicListener, RingValueBuffer, ListValueBuffer

from testrostopicmonitor.localsizecalculator import generate_calculator as generate_calculator_mock, generate_type


_LOGGER = logging.getLogger(__name__)


## =====================================================


class TopicStatsTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_update_infinite(self):
        topic_stats = TopicStats()
        topic_stats.start()

        time.sleep(0.2)
        topic_stats.update(6)
        topic_stats.update(10)
        topic_stats.update(20)
        time.sleep(0.2)

        topic_stats.stop()
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 9)
        self.assertEqual(stats["total_count"], 3)
        self.assertEqual(stats["total_size"], 36)
        self.assertGreater(stats["total_time"], 0.4)  # duration will always be a bit greater than 0.4
        self.assertLess(stats["total_freq"], 7.5)
        self.assertLess(stats["total_bw"], 90)
        self.assertEqual(stats["min"], 6)
        self.assertEqual(stats["max"], 20)
        self.assertEqual(stats["mean"], 12)
        self.assertEqual(stats["stddev"], 5.887840577551898)

    def test_update_windowed(self):
        topic_stats = TopicStats()
        topic_stats.start(window_size=2)

        time.sleep(0.2)
        topic_stats.update(6)
        topic_stats.update(10)
        topic_stats.update(20)
        time.sleep(0.2)

        topic_stats.stop()
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 9)
        self.assertEqual(stats["total_count"], 3)
        self.assertEqual(stats["total_size"], 36)
        self.assertGreater(stats["total_time"], 0.4)  # duration will always be a bit greater than 0.4
        self.assertLess(stats["total_freq"], 7.5)
        self.assertLess(stats["total_bw"], 90)
        self.assertEqual(stats["min"], 10)
        self.assertEqual(stats["max"], 20)
        self.assertEqual(stats["mean"], 15)
        self.assertEqual(stats["stddev"], 5.0)


## ===================================================


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


## ===================================================


class TopicListenerTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_update(self):
        MessageClass = generate_type(["int32"])

        topic_mon = TopicListenerMock("/test_topic", MessageClass)
        topic_mon.start()

        time.sleep(0.2)
        topic_mon.update(None)
        time.sleep(0.2)

        topic_mon.stop()
        stats = topic_mon.getStats()

        self.assertEqual(1, stats["total_count"])
        self.assertEqual(4, stats["total_size"])
        self.assertLess(0, stats["total_freq"])
        self.assertLess(0, stats["total_bw"])


class TopicListenerMock(TopicListener):
    def update(self, data):
        self._updateState(data)

    def _generateCalculator(self) -> AbstractCalculator:
        return generate_calculator_mock(self.message_class)

    def _startMonitor(self):
        # do nothing
        pass
