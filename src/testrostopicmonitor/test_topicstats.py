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

from rostopicmonitor.topicstats import WindowTopicStats, RingValueBuffer, ListValueBuffer, RawTopicStats


_LOGGER = logging.getLogger(__name__)


## =====================================================


class RawTopicStatsTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_update(self):
        topic_stats = RawTopicStats()
        topic_stats.start()

        time.sleep(0.2)
        topic_stats.update(6)
        topic_stats.update(10)
        topic_stats.update(20)
        time.sleep(0.2)

        topic_stats.stop()
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 1)
        self.assertIn("data", stats)
        samples = stats["data"]
        self.assertEqual(len(samples), 2)
        self.assertGreater(samples["timestamp"][0], 0.2)
        self.assertEqual(get_dict_from_samples(samples, 0), {"size": 6})
        self.assertEqual(get_dict_from_samples(samples, 1), {"size": 10})
        self.assertEqual(get_dict_from_samples(samples, 2), {"size": 20})


## =====================================================


class WindowTopicStatsTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_update_infinite(self):
        topic_stats = WindowTopicStats()
        topic_stats.start()

        time.sleep(0.2)
        topic_stats.update(6)
        topic_stats.update(10)
        topic_stats.update(20)
        time.sleep(0.2)

        topic_stats.stop()
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 6)
        self.assertEqual(stats["total_count"], 3)
        self.assertEqual(stats["total_size"], 36)
        self.assertGreater(stats["total_time"], 0.4)  # duration will always be a bit greater than 0.4
        self.assertLess(stats["total_freq"], 7.5)
        self.assertLess(stats["total_bw"], 90)
        samples = stats["data"]
        self.assertEqual(len(samples), 5)
        self.assertGreater(samples["timestamp"][0], 0.2)
        self.assertEqual(get_dict_from_samples(samples, 0), {"max": 6, "mean": 6.0, "min": 6, "stddev": 0.0})
        self.assertEqual(get_dict_from_samples(samples, 1), {"max": 10, "mean": 8.0, "min": 6, "stddev": 2.0})
        self.assertEqual(
            get_dict_from_samples(samples, 2), {"max": 20, "mean": 12.0, "min": 6, "stddev": 5.887840577551898}
        )

    def test_update_windowed(self):
        topic_stats = WindowTopicStats(window_size=2)
        topic_stats.start()

        time.sleep(0.2)
        topic_stats.update(6)
        topic_stats.update(10)
        topic_stats.update(20)
        time.sleep(0.2)

        topic_stats.stop()
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 6)
        self.assertEqual(stats["total_count"], 3)
        self.assertEqual(stats["total_size"], 36)
        self.assertGreater(stats["total_time"], 0.4)  # duration will always be a bit greater than 0.4
        self.assertLess(stats["total_freq"], 7.5)
        self.assertLess(stats["total_bw"], 90)
        samples = stats["data"]
        self.assertEqual(len(samples), 5)
        self.assertGreater(samples["timestamp"][0], 0.2)
        self.assertEqual(get_dict_from_samples(samples, 0), {"max": 6, "mean": 3.0, "min": 0, "stddev": 3.0})
        self.assertEqual(get_dict_from_samples(samples, 1), {"max": 10, "mean": 8.0, "min": 6, "stddev": 2.0})
        self.assertEqual(get_dict_from_samples(samples, 2), {"max": 20, "mean": 15.0, "min": 10, "stddev": 5.0})


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


def get_dict_from_samples(samples_data, row_index):
    ret_dict = {}
    for key, val in samples_data.items():
        ret_dict[key] = val[row_index]
    del ret_dict["timestamp"]
    return ret_dict
