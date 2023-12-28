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

from rostopicmonitor.valuebuffer import ValueBuffer, ListValueBuffer, RingValueBuffer


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
        timestamps = [sample[0] for sample in self.samples]
        sizes = [sample[1] for sample in self.samples]
        return {"data": {"timestamp": timestamps, "size": sizes}}


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
            "window": self.window_size,
            "total_count": self.total_count,
            "total_size": self.total_size,
            "total_time": duration_secs,
            "total_freq": float(self.total_count) / duration_secs,
            "total_bw": float(self.total_size) / duration_secs,
        }
        dict_list = []
        buffer: ValueBuffer = self._spawnBuffer()
        for data in self.samples:
            buff_dict = {"timestamp": data[0], "size": data[1]}
            buffer.add(data[1])
            buff_data = buffer.getData()
            buff_dict.update(buff_data)
            dict_list.append(buff_dict)
        stats_list = {}
        if dict_list:
            stats_list = {dict_key: [item_dict[dict_key] for item_dict in dict_list] for dict_key in dict_list[0]}
        stats_dict["data"] = stats_list
        return stats_dict

    def _spawnBuffer(self) -> ValueBuffer:
        if self.window_size < 1:
            return ListValueBuffer()
        return RingValueBuffer(self.window_size)
