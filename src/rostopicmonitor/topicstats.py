#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import logging
from abc import ABC, abstractmethod
import datetime
import math

from rostopicmonitor.sizecalculator import AbstractCalculator


_LOGGER = logging.getLogger(__name__)


## =====================================================


class TopicListener(ABC):
    def __init__(self, topic_name, topic_type):
        self.topic_name = topic_name
        self.message_class = topic_type
        self.message_calc: AbstractCalculator = self._generateCalculator()

        self.stats = TopicStats()

    def start(self, window_size=None):
        self.stats = TopicStats()
        self.stats.start(window_size=window_size)
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
            stats_dict["total_duration"],
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
        self.window_buffer: ValueBuffer = ListValueBuffer()

    def start(self, window_size=None):
        w_size = 0    # non-positive value means consider all
        if window_size and window_size > 0:
            w_size = window_size
        self.total_count = 0
        self.total_size = 0
        self.start_time = datetime.datetime.now()
        self.stop_time = None
        if w_size < 1:
            self.window_buffer = ListValueBuffer()
        else:
            self.window_buffer = RingValueBuffer(w_size)

    def stop(self):
        self.stop_time = datetime.datetime.now()

    def isStopped(self):
        return self.stop_time is not None

    def update(self, data_size):
        self.total_count += 1
        self.total_size += data_size
        self.window_buffer.add(data_size)

    def getStats(self):
        duration = self.stop_time - self.start_time
        duration_secs = duration.total_seconds()
        stats_dict = {
            # total stats
            "total_count": self.total_count,
            "total_size": self.total_size,
            "total_duration": duration_secs,
            "total_freq": float(self.total_count) / duration_secs,
            "total_bw": float(self.total_size) / duration_secs,
            # window stats
            "min": self.window_buffer.min(),
            "max": self.window_buffer.max(),
            "mean": self.window_buffer.mean(),
            "stddev": self.window_buffer.stddev(),
        }
        return stats_dict


# ==================================================================


class ValueBuffer(ABC):
    @abstractmethod
    def min(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def max(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def sum(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def mean(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def stddev(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def add(self, new_value):
        raise NotImplementedError("You need to define this method in derived class!")


class ListValueBuffer(ValueBuffer):
    def __init__(self):
        self.values = []
        self.buffer_sum = 0

    def __len__(self):
        return len(self.values)

    def min(self):
        if len(self.values) < 1:
            return 0
        return min(self.values)

    def max(self):
        if len(self.values) < 1:
            return 0
        return max(self.values)

    def sum(self):
        return self.buffer_sum

    def mean(self):
        buffer_size = len(self.values)
        if buffer_size < 1:
            return 0
        return self.buffer_sum / buffer_size

    def stddev(self):
        buffer_size = len(self.values)
        if buffer_size < 1:
            return 0
        mean = self.mean()
        pow_dist = 0
        for val in self.values:
            pow_dist += math.pow(val - mean, 2)
        return math.sqrt(pow_dist / buffer_size)

    def add(self, new_value):
        self.buffer_sum += new_value
        self.values.append(new_value)


class RingValueBuffer(ValueBuffer):
    def __init__(self, buffer_size=0):
        self.buffer_size = buffer_size
        if self.buffer_size < 0:
            raise ValueError("invalid size: expected positive value")
        self.values = [0] * self.buffer_size
        self.next_index = 0
        self.buffer_sum = 0

    def __len__(self):
        return self.buffer_size

    def min(self):
        if self.buffer_size < 1:
            return 0
        return min(self.values)

    def max(self):
        if self.buffer_size < 1:
            return 0
        return max(self.values)

    def sum(self):
        return self.buffer_sum

    def mean(self):
        if self.buffer_size < 1:
            return 0
        return self.buffer_sum / self.buffer_size

    def stddev(self):
        if self.buffer_size < 1:
            return 0
        mean = self.mean()
        pow_dist = 0
        for val in self.values:
            pow_dist += math.pow(val - mean, 2)
        return math.sqrt(pow_dist / self.buffer_size)

    def add(self, new_value):
        if self.buffer_size < 1:
            return
        self.buffer_sum -= self.values[self.next_index] - new_value
        self.values[self.next_index] = new_value
        self.next_index = (self.next_index + 1) % self.buffer_size
