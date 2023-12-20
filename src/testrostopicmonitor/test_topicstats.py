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
from rostopicmonitor.topicstats import TopicStats

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

    def test_update(self):
        MessageClass = generate_type(["int32"])

        topic_mon = TopicStatsMock("/test_topic", MessageClass)
        topic_mon.start()

        time.sleep(0.2)
        topic_mon.update(None)
        time.sleep(0.2)

        topic_mon.stop()
        stats = topic_mon.getStats()

        self.assertEqual(1, stats["count"])
        self.assertEqual(4, stats["size"])
        self.assertLess(0, stats["freq"])
        self.assertLess(0, stats["bw"])


## ============================================================


class TopicStatsMock(TopicStats):
    
    def __init__(self, topic_name, topic_type):
        super().__init__(topic_name, topic_type)

    def update(self, data):
        self._updateState(data)

    def _generateCalculator(self) -> AbstractCalculator:
        return generate_calculator_mock( self.message_class )

    def _startMonitor(self):
        # do nothing
        pass
