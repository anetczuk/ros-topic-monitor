#
# Copyright (c) 2023, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

import logging

from rostopicmonitor.topicstats import WindowTopicStats, RawTopicStats


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
        topic_stats.update(0.1, 6)
        topic_stats.update(0.2, 10)
        topic_stats.update(0.3, 20)
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 7)
        self.assertEqual(stats["window"], 0)
        self.assertEqual(stats["total_count"], 3)
        self.assertEqual(stats["total_size"], 36)
        self.assertEqual(stats["total_time"], 0.3)
        self.assertEqual(stats["total_freq"], 10.0)
        self.assertEqual(stats["total_bw"], 120)
        self.assertIn("data", stats)
        samples = stats["data"]
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples["time"][0], 0.1)
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
        topic_stats.update(0.1, 6)
        topic_stats.update(0.2, 10)
        topic_stats.update(0.3, 20)
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 7)
        self.assertEqual(stats["window"], 0)
        self.assertEqual(stats["total_count"], 3)
        self.assertEqual(stats["total_size"], 36)
        self.assertEqual(stats["total_time"], 0.3)
        self.assertEqual(stats["total_freq"], 10.0)
        self.assertEqual(stats["total_bw"], 120)
        samples = stats["data"]
        self.assertEqual(len(samples), 12)
        self.assertEqual(samples["time"][0], 0.1)
        self.assertEqual(
            {
                "bw": 60.0,
                "dt_max": 0.1,
                "dt_mean": 0.1,
                "dt_min": 0.1,
                "dt_stddev": 0.0,
                "freq": 10.0,
                "size": 6,
                "size_max": 6,
                "size_mean": 6.0,
                "size_min": 6,
                "size_stddev": 0.0,
            },
            get_dict_from_samples(samples, 0),
        )
        self.assertEqual(
            {
                "bw": 80.0,
                "dt_max": 0.1,
                "dt_mean": 0.1,
                "dt_min": 0.1,
                "dt_stddev": 0.0,
                "freq": 10.0,
                "size": 10,
                "size_max": 10,
                "size_mean": 8.0,
                "size_min": 6,
                "size_stddev": 2.0,
            },
            get_dict_from_samples(samples, 1),
        )
        self.assertEqual(
            {
                "bw": 120.0,
                "dt_max": 0.1,
                "dt_mean": 0.09999999999999999,
                "dt_min": 0.09999999999999998,
                "dt_stddev": 1.3877787807814457e-17,
                "freq": 10.0,
                "size": 20,
                "size_max": 20,
                "size_mean": 12.0,
                "size_min": 6,
                "size_stddev": 5.887840577551898,
            },
            get_dict_from_samples(samples, 2),
        )

    def test_update_windowed(self):
        topic_stats = WindowTopicStats(window_size=2)
        topic_stats.update(0.1, 6)
        topic_stats.update(0.2, 10)
        topic_stats.update(0.3, 20)
        stats = topic_stats.getStats()

        self.assertEqual(len(stats), 7)
        self.assertEqual(stats["window"], 2)
        self.assertEqual(stats["total_count"], 3)
        self.assertEqual(stats["total_size"], 36)
        self.assertEqual(stats["total_time"], 0.3)  # duration will always be a bit greater than 0.3
        self.assertEqual(stats["total_freq"], 10.0)
        self.assertEqual(stats["total_bw"], 120)
        samples = stats["data"]
        self.assertEqual(len(samples), 12)
        self.assertEqual(samples["time"][0], 0.1)
        self.assertEqual(
            {
                "bw": 60.0,
                "dt_max": 0.1,
                "dt_mean": 0.1,
                "dt_min": 0.1,
                "dt_stddev": 0.0,
                "freq": 10.0,
                "size": 6,
                "size_max": 6,
                "size_mean": 6.0,
                "size_min": 6,
                "size_stddev": 0.0,
            },
            get_dict_from_samples(samples, 0),
        )
        self.assertEqual(
            {
                "bw": 80.0,
                "dt_max": 0.1,
                "dt_mean": 0.1,
                "dt_min": 0.1,
                "dt_stddev": 0.0,
                "freq": 10.0,
                "size": 10,
                "size_max": 10,
                "size_mean": 8.0,
                "size_min": 6,
                "size_stddev": 2.0,
            },
            get_dict_from_samples(samples, 1),
        )
        self.assertEqual(
            {
                "bw": 150.0,
                "dt_max": 0.1,
                "dt_mean": 0.09999999999999999,
                "dt_min": 0.09999999999999998,
                "dt_stddev": 1.3877787807814457e-17,
                "freq": 10.0,
                "size": 20,
                "size_max": 20,
                "size_mean": 15.0,
                "size_min": 10,
                "size_stddev": 5.0,
            },
            get_dict_from_samples(samples, 2),
        )


def get_dict_from_samples(samples_data, row_index):
    ret_dict = {}
    for key, val in samples_data.items():
        ret_dict[key] = val[row_index]
    del ret_dict["time"]
    return ret_dict
