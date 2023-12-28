#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import logging
from abc import ABC, abstractmethod
import math


_LOGGER = logging.getLogger(__name__)


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
        stats_dict = {
            "size_min": self.min(),
            "size_max": self.max(),
            "size_mean": self.mean(),
            "size_stddev": self.stddev(),
        }
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
