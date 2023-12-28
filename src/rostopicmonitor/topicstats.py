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


# class ElapseTime:
#
#     def __init__(self):
#         self.start_time: datetime.datetime = None
#         self.stop_time: float = None                # seconds since start
#
#     def start(self):
#         self.start_time = datetime.datetime.now()
#         self.stop_time = None
#
#     def stop(self):
#         self.stop_time = self._currDuration()
#
#     def _currDuration(self):
#         duration = datetime.datetime.now() - self.start_time
#         return duration.total_seconds()


# ==================================================================


class BaseTopicStats(ABC):
    def __init__(self):
        self.start_time: datetime.datetime = None
        self.stop_time: float = None                # seconds since start

    def getDuration(self):
        return self.stop_time

    def start(self):
        self.start_time = datetime.datetime.now()
        self.stop_time = None
        self._start()

    def stop(self):
        self.stop_time = self._currDuration()
        self._stop()

    def isStopped(self):
        return self.stop_time is not None

    def _start(self):
        # implement if needed
        pass

    def _stop(self):
        # implement if needed
        pass

    @abstractmethod
    def update(self, data_size):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def getStats(self):
        raise NotImplementedError("You need to define this method in derived class!")

    def _currDuration(self):
        duration = datetime.datetime.now() - self.start_time
        return duration.total_seconds()


class RawTopicStats(BaseTopicStats):
    def __init__(self):
        super().__init__()
        self.samples = []

    def _start(self):
        self.samples = []

    def _stop(self):
        if self.samples:
            # set stop time as time of last message
            # it makes statistics more accurate, because there is slight time gap
            # between stop of ROS main loop and stop time of 'self.stop_time'
            self.stop_time = self.samples[-1][0]

    def update(self, data_size):
        duration = self._currDuration()
        self.samples.append((duration, data_size))

    def getStats(self):
        timestamps = [sample[0] for sample in self.samples]
        sizes = [sample[1] for sample in self.samples]
        return {"data": {"time": timestamps, "size": sizes}}


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
        size_buffer: ValueBuffer = self._spawnBuffer()
        time_buffer: ValueBuffer = self._spawnBuffer()

        prev_time = 0
        for data in self.samples:
            sample_dict = {"time": data[0], "size": data[1]}

            time_diff = data[0] - prev_time
            prev_time = data[0]
            time_buffer.add(time_diff)
            time_data = time_buffer.getData("dt")
            sample_dict.update(time_data)
            sample_dict.update({"freq": 1.0 / time_buffer.mean()})

            size_buffer.add(data[1])
            size_data = size_buffer.getData("size")
            sample_dict.update(size_data)

            time_diff = time_buffer.sum()

            msg_bw = 0.0
            if time_diff == 0:
                sample_dict.update({"bw": ""})
            else:
                size_sum = size_buffer.sum()
                msg_bw = size_sum / time_diff
                sample_dict.update({"bw": msg_bw})

            dict_list.append(sample_dict)

        stats_list = {}
        if dict_list:
            stats_list = {dict_key: [item_dict[dict_key] for item_dict in dict_list] for dict_key in dict_list[0]}
        stats_dict["data"] = stats_list
        return stats_dict

    def _spawnBuffer(self) -> ValueBuffer:
        if self.window_size < 1:
            return ListValueBuffer()
        return RingValueBuffer(self.window_size)
