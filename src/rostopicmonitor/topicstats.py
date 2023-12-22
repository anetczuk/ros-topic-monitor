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


class TopicListener(ABC):
    def __init__(self, topic_name, topic_type):
        self.topic_name = topic_name
        self.message_class = topic_type
        self.message_calc: AbstractCalculator = self._generateCalculator()

        self.stats = TopicStats()

    def start(self):
        self.stats = TopicStats()
        self.stats.start()
        self._startMonitor()

    def stop(self):
        self.stats.stop()

    @abstractmethod
    def _generateCalculator(self) -> AbstractCalculator:
        """Return 'True' if calculator represents fixed-sized data, otherwise 'False'."""
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def _startMonitor(self):
        """Subscribe to topic."""
        raise NotImplementedError("You need to define this method in derived class!")

    def getStats(self):
        return self.stats.getStats()

    def printInfo(self):
        stats_dict = self.stats.getStats()
        _LOGGER.info(
            "topic %s total_count: %s total_size: %s duration: %s",
            self.topic_name,
            stats_dict["total_count"],
            stats_dict["total_size"],
            stats_dict["duration"]
        )

    def _updateState(self, data):
        if self.stats.isStopped():
            # stop requested - do not consider more data
            return
        msg_size = self.message_calc.calculate(data)
        self.stats.update(msg_size)


class TopicStats:
    def __init__(self):
        self.total_count = 0
        self.total_size = 0
        self.start_time = None
        self.stop_time = None

    def start(self):
        self.total_count = 0
        self.total_size = 0
        self.start_time = datetime.datetime.now()
        self.stop_time = None

    def stop(self):
        self.stop_time = datetime.datetime.now()

    def isStopped(self):
        return self.stop_time is not None

    def update(self, data_size):
        self.total_count += 1
        self.total_size += data_size

    def getStats(self):
        duration = self.stop_time - self.start_time
        duration_secs = duration.total_seconds()
        stats_dict = {
            "total_count": self.total_count,
            "total_size": self.total_size,
            "duration": duration_secs,
            "total_freq": float(self.total_count) / duration_secs,
            "total_bw": float(self.total_size) / duration_secs,
        }
        return stats_dict
