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


_LOGGER = logging.getLogger(__name__)


# ==================================================================


class BaseTopicStats(ABC):
    def __init__(self):
        self.start_time = None
        self.stop_time = None

    def getDuration(self):
        duration = self.stop_time - self.start_time
        return duration.total_seconds()

    def start(self):
        self.start_time = datetime.datetime.now()
        self.stop_time = None
        self._start()

    def stop(self):
        self.stop_time = datetime.datetime.now()

    def isStopped(self):
        return self.stop_time is not None

    def _start(self):
        # implement if needed
        pass

    @abstractmethod
    def update(self, data_size):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def getStats(self):
        raise NotImplementedError("You need to define this method in derived class!")


class RawTopicStats(BaseTopicStats):
    def __init__(self):
        super().__init__()
        self.samples = []

    def _start(self):
        self.samples = []

    def update(self, data_size):
        duration = datetime.datetime.now() - self.start_time
        duration_secs = duration.total_seconds()
        self.samples.append((duration_secs, data_size))

    def getStats(self):
        return {"data": self.samples}


class WindowTopicStats(RawTopicStats):
    def __init__(self, window_size=0):
        super().__init__()
        self.total_count = 0
        self.total_size = 0
        self.window_size = window_size

    def _start(self):
        super()._start()
        self.total_count = 0
        self.total_size = 0

    def update(self, data_size):
        super().update(data_size)
        self.total_count += 1
        self.total_size += data_size

    def getStats(self):
        duration_secs = self.getDuration()
        stats_dict = {
            # total stats
            "total_count": self.total_count,
            "total_size": self.total_size,
            "total_time": duration_secs,
            "total_freq": float(self.total_count) / duration_secs,
            "total_bw": float(self.total_size) / duration_secs,
        }
        stats_list = []
        buffer = self._getBuffer()
        for data in self.samples:
            buffer.add(data[1])
            buff_data = buffer.getData()
            stats_list.append((data[0], buff_data))
        stats_dict["data"] = stats_list
        return stats_dict

    def _getBuffer(self):
        if self.window_size < 1:
            return ListValueBuffer()
        return RingValueBuffer(self.window_size)


# ==================================================================


class ValueBuffer(ABC):
    @abstractmethod
    def reset(self):
        raise NotImplementedError("You need to define this method in derived class!")

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

    def getData(self):
        stats_dict = {"min": self.min(), "max": self.max(), "mean": self.mean(), "stddev": self.stddev()}
        return stats_dict


class ListValueBuffer(ValueBuffer):
    def __init__(self):
        self.values = []
        self.buffer_sum = 0

    def __len__(self):
        return len(self.values)

    def reset(self):
        self.values = []
        self.buffer_sum = 0

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

    def reset(self):
        self.values = [0] * self.buffer_size
        self.next_index = 0
        self.buffer_sum = 0

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
