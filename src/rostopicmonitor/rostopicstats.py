#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import rospy
import rostopic

from rostopicmonitor.topicstats import TopicStats
from rostopicmonitor.sizecalculator import AbstractCalculator
from rostopicmonitor.rossizecalculator import generate_calculator


_LOGGER = logging.getLogger(__name__)


## =====================================================


class ROSTopicStats(TopicStats):

    def __init__(self, topic_name):
        topic_data = rostopic.get_topic_class( topic_name )
        topic_type = topic_data[0]
        super().__init__(topic_name, topic_type)

    def _generateCalculator(self) -> AbstractCalculator:
        return generate_calculator( self.message_class )

    def _startMonitor(self):
        _LOGGER.info( f"subscribing to topic: {self.topic_name} fixed: {self.message_calc.isFixed()}" )
        rospy.Subscriber(self.topic_name, self.message_class, self._updateState)
