#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import logging
import json

from rostopicmonitor.sizecalculator import AbstractCalculator
from rostopicmonitor.topicstats import BaseTopicStats, ElapsedTime


_LOGGER = logging.getLogger(__name__)


## =====================================================


class TopicListener:
    def __init__(self, topic_name, message_calc: AbstractCalculator):
        self.topic_name = topic_name
        self.message_calc: AbstractCalculator = message_calc
        self.elapsed_time: ElapsedTime = ElapsedTime()
        self.stats: BaseTopicStats = None

    def setStatsCollector(self, collector: BaseTopicStats):
        self.stats = collector

    def start(self):
        self.elapsed_time.start()

    def stop(self):
        self.elapsed_time.stop()

    def updateStats(self, msg_data):
        if self.elapsed_time.isStopped():
            # stop requested - do not consider more data
            return
        msg_time = self.elapsed_time.getCurrentDuration()
        msg_size = self.message_calc.calculate(msg_data)
        self.stats.update(msg_time, msg_size)

    def getStats(self):
        stats_dict = self.stats.getStats()
        topic_data = {"topic": self.topic_name, "fixed_size:": self.message_calc.isFixed()}
        topic_data.update(stats_dict)
        return topic_data

    def getStatsRaw(self):
        stats_dict = self.stats.getStatsRaw()
        topic_data = {"topic": self.topic_name, "fixed_size:": self.message_calc.isFixed()}
        topic_data.update(stats_dict)
        return topic_data

    def getStatsSummary(self):
        stats_dict = self.stats.getStatsSummary()
        topic_data = {"topic": self.topic_name, "fixed_size:": self.message_calc.isFixed()}
        topic_data.update(stats_dict)
        return topic_data

    def printInfo(self):
        stats_dict = self.stats.getStats()
        _LOGGER.info("topic %s data:\n%s", self.topic_name, json.dumps(stats_dict, indent=4))
