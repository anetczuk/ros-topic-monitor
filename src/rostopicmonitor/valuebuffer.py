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

    def getData(self, prefix="", stddev=True):
        if prefix:
            prefix += "_"
        stats_dict = {f"{prefix}min": self.min(), f"{prefix}max": self.max(), f"{prefix}mean": self.mean()}
        if stddev:
            stats_dict[f"{prefix}stddev"] = self.stddev()
        return stats_dict


class ListValueBuffer(ValueBuffer):
    def __init__(self):
        self.values = []
        self.buffer_sum = 0
        self._cached_min = None
        self._cached_max = None
        self._cached_mean = 0
        self._cached_stddev: CachedStdDevCalc = CachedStdDevCalc()

    def __len__(self):
        return len(self.values)

    def reset(self):
        self.values = []
        self.buffer_sum = 0
        self._cached_min = None
        self._cached_max = None
        self._cached_mean = 0
        self._cached_stddev = CachedStdDevCalc()

    def min(self):
        if self._cached_min is None:
            return 0
        return self._cached_min

    def max(self):
        if self._cached_max is None:
            return 0
        return self._cached_max

    def sum(self):
        return self.buffer_sum

    def mean(self):
        return self._cached_mean

    def stddev(self):
        buffer_size = len(self.values)
        if buffer_size < 1:
            return 0
        mean = self.mean()
        # return self._cached_stddev.calc(self.values, mean)

        new_value = self.values[-1]
        new_stddev = self._cached_stddev.extend(new_value, mean)

        if new_stddev is None:
            # could not calc stddev - use standard algorithm
            new_stddev = self._cached_stddev.calc(self.values, mean)

        return new_stddev

    def add(self, new_value):
        self.buffer_sum += new_value
        self.values.append(new_value)

        if self._cached_min is None:
            self._cached_min = new_value
        elif self._cached_min > new_value:
            self._cached_min = new_value

        if self._cached_max is None:
            self._cached_max = new_value
        elif self._cached_max < new_value:
            self._cached_max = new_value

        buffer_size = len(self.values)
        self._cached_mean = self.buffer_sum / buffer_size


class RingValueBuffer(ValueBuffer):
    def __init__(self, buffer_size=0):
        self.buffer: RingBuffer = RingBuffer(buffer_size)
        self.buffer_sum = 0
        self._cached_min = None
        self._cached_max = None
        self._cached_mean = 0
        self._cached_stddev: CachedStdDevCalc = CachedStdDevCalc()

    def __len__(self):
        return len(self.buffer)

    def reset(self):
        self.buffer.reset()
        self.buffer_sum = 0
        self._cached_min = None
        self._cached_max = None
        self._cached_mean = 0
        self._cached_stddev = CachedStdDevCalc()

    def min(self):
        if self._cached_min is None:
            return 0
        return self._cached_min

    def max(self):
        if self._cached_max is None:
            return 0
        return self._cached_max

    def sum(self):
        return self.buffer_sum

    def mean(self):
        return self._cached_mean

    def stddev(self):
        if self.buffer.buffer_size < 1:
            return 0
        mean = self.mean()
        # values = self.buffer.currValues()
        # return self._cached_stddev.calc(values, mean)

        new_stddev = None
        if self.buffer.isFull():
            old_value = self.buffer.oldestValue()
            new_value = self.buffer.newestValue()
            new_stddev = self._cached_stddev.replace(new_value, old_value, mean)
        else:
            new_value = self.buffer.newestValue()
            new_stddev = self._cached_stddev.extend(new_value, mean)

        if new_stddev is None:
            # could not calc stddev - use standard algorithm
            values = self.buffer.currValues()
            new_stddev = self._cached_stddev.calc(values, mean)

        return new_stddev

    def add(self, new_value):
        oldest = self.buffer.oldestValue()
        self.buffer_sum -= self.buffer.oldestValue() - new_value
        self.buffer.add(new_value)

        if self._cached_min is None:
            self._cached_min = new_value
        elif oldest > self._cached_min:
            self._cached_min = min(self._cached_min, new_value)
        else:
            values = self.buffer.currValues()
            self._cached_min = min(values)

        if self._cached_max is None:
            self._cached_max = new_value
        elif oldest < self._cached_min:
            self._cached_max = max(self._cached_max, new_value)
        else:
            values = self.buffer.currValues()
            self._cached_max = max(values)

        buffer_size = len(self.buffer)
        self._cached_mean = self.buffer_sum / buffer_size


# ==================================================================


class RingBuffer:
    def __init__(self, buffer_size=0):
        self.buffer_size = buffer_size
        if self.buffer_size < 0:
            raise ValueError("invalid size: expected positive value")
        self.count = 0
        self.values = [0] * self.buffer_size
        self.next_index = 0

    def __len__(self):
        return min(self.buffer_size, self.count)

    def reset(self):
        self.count = 0
        self.values = [0] * self.buffer_size
        self.next_index = 0

    def isFull(self):
        return self.count >= self.buffer_size

    def currValues(self):
        if self.count < self.buffer_size:
            size = len(self)
            return self.values[0:size]
        return self.values

    def oldestValue(self):
        return self.values[self.next_index]

    def newestValue(self):
        prev_index = (self.next_index - 1) % self.buffer_size
        return self.values[prev_index]

    def add(self, new_value):
        self.count += 1
        if self.buffer_size < 1:
            return
        self.values[self.next_index] = new_value
        self.next_index = (self.next_index + 1) % self.buffer_size


class CachedStdDevCalc:
    """Fast way of calculating standard deviation assuming.

    Implementation assumes that new value does not change mean of value range.
    """

    def __init__(self):
        self.prev_mean = None
        self.val_size = None
        self.pow_sum = None

    def calc(self, values, mean):
        self.prev_mean = mean
        self.pow_sum = 0
        for val in values:
            diff = val - mean
            self.pow_sum += diff * diff
            # pow_dist += math.pow(val - mean, 2)
        self.val_size = len(values)
        return math.sqrt(self.pow_sum / self.val_size)

    def replace(self, new_value, old_value, mean):
        """Return value if calculation succeed, otherwise None (if could not calculate stddev).

        Method assumes that new_value replaces old value (calculation window is constant).
        """
        if self.prev_mean is None or self.prev_mean != mean:
            # unable calculate stddev
            return None

        old_diff = old_value - mean
        self.pow_sum -= old_diff * old_diff
        new_diff = new_value - mean
        self.pow_sum += new_diff * new_diff
        return math.sqrt(self.pow_sum / self.val_size)

    def extend(self, new_value, mean):
        """Return value if calculation succeed, otherwise None (if could not calculate stddev)."""
        if self.prev_mean is None or self.prev_mean != mean:
            # unable calculate stddev
            return None

        new_diff = new_value - mean
        self.pow_sum += new_diff * new_diff
        self.val_size += 1
        return math.sqrt(self.pow_sum / self.val_size)
