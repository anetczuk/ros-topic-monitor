#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import logging
import datetime
from abc import ABC, abstractmethod

from rostopicmonitor.sizecalculator import AbstractCalculator


_LOGGER = logging.getLogger(__name__)


## =====================================================


class TopicStats(ABC):
    def __init__(self, topic_name, topic_type):
        self.topic_name = topic_name
        self.message_class = topic_type
        self.message_calc: AbstractCalculator = self._generateCalculator()

        self.msg_count = 0
        self.msg_size = 0
        self.start_time = None
        self.stop_time = None

    def start(self):
        self.msg_count = 0
        self.msg_size = 0
        self.start_time = datetime.datetime.now()
        self.stop_time = None

        self._startMonitor()

    def stop(self):
        self.stop_time = datetime.datetime.now()

    def getStats(self):
        duration = self.stop_time - self.start_time
        duration_secs = duration.total_seconds()
        stats_dict = {
            "count": self.msg_count,
            "size": self.msg_size,
            "duration": duration,
            "freq": float(self.msg_count) / duration_secs,
            "bw": float(self.msg_size) / duration_secs,
        }
        return stats_dict

    def printInfo(self):
        duration = self.stop_time - self.start_time
        _LOGGER.info(
            "topic %s msg_count: %s msg_size: %s duration: %s", self.topic_name, self.msg_count, self.msg_size, duration
        )

    def _updateState(self, data):
        self.msg_count += 1
        self.msg_size += self.message_calc.calculate(data)

    @abstractmethod
    def _generateCalculator(self) -> AbstractCalculator:
        """Return 'True' if calculator represents fixed-sized data, otherwise 'False'."""
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def _startMonitor(self):
        """Subscribe to topic."""
        raise NotImplementedError("You need to define this method in derived class!")
