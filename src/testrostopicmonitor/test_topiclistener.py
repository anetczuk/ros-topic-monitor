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
from rostopicmonitor.topiclistener import TopicListener
from rostopicmonitor.topicstats import WindowTopicStats

from testrostopicmonitor.localsizecalculator import generate_calculator as generate_calculator_mock, generate_type


_LOGGER = logging.getLogger(__name__)


## =====================================================


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
        topic_mon.setStatsCollector(WindowTopicStats())
        topic_mon.start()

        time.sleep(0.2)
        topic_mon.updateStats(None)
        time.sleep(0.2)

        topic_mon.stop()
        stats = topic_mon.getStats()

        self.assertEqual(1, stats["total_count"])
        self.assertEqual(4, stats["total_size"])
        self.assertLess(0, stats["total_freq"])
        self.assertLess(0, stats["total_bw"])


class TopicListenerMock(TopicListener):
    def __init__(self, topic_name, topic_type):
        message_calc: AbstractCalculator = generate_calculator_mock(topic_type)
        super().__init__(topic_name, message_calc)
