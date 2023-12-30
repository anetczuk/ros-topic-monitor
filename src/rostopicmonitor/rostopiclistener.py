#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import logging

import rospy
import rostopic

from rostopicmonitor.topiclistener import TopicListener
from rostopicmonitor.sizecalculator import AbstractCalculator
from rostopicmonitor.rossizecalculator import generate_calculator


_LOGGER = logging.getLogger(__name__)


## =====================================================


class ROSTopicListener(TopicListener):
    def __init__(self, topic_name):
        topic_data = rostopic.get_topic_class(topic_name)
        topic_type = topic_data[0]
        self.message_class = topic_type
        message_calc: AbstractCalculator = generate_calculator(self.message_class)
        super().__init__(topic_name, message_calc)

    def start(self):
        super().start()
        _LOGGER.info("subscribing to topic: %s fixed: %s", self.topic_name, self.message_calc.isFixed())
        rospy.Subscriber(self.topic_name, self.message_class, self.updateStats)
