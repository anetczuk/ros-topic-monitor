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
from rostopicmonitor.utils import convert_listdicts_dictlists


_LOGGER = logging.getLogger(__name__)


# ==================================================================


class ElapsedTime:
    def __init__(self):
        self.start_time: datetime.datetime = None
        self.stop_time: float = None  # seconds since start

    def getCurrentDuration(self):
        duration = datetime.datetime.now() - self.start_time
        return duration.total_seconds()

    def getDuration(self):
        return self.stop_time

    def isStopped(self):
        return self.stop_time is not None

    def start(self):
        self.start_time = datetime.datetime.now()
        self.stop_time = None

    def stop(self):
        self.stop_time = self.getCurrentDuration()


# ==================================================================


class BaseTopicStats(ABC):
    @abstractmethod
    def reset(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def setSamplesFromDict(self, stats_dict):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def update(self, data_time, data_size):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def getStats(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def getStatsRaw(self):
        raise NotImplementedError("You need to define this method in derived class!")

    @abstractmethod
    def getStatsSummary(self):
        raise NotImplementedError("You need to define this method in derived class!")


class RawTopicStats(BaseTopicStats):
    def __init__(self):
        self.samples = []

    def reset(self):
        self.samples = []

    def setSamplesFromDict(self, stats_dict):
        samples_dict = stats_dict.get("data", {})
        timestamps = samples_dict.get("time", [])
        sizes = samples_dict.get("size", [])
        self.samples = list(zip(timestamps, sizes))

    def getDuration(self):
        if not self.samples:
            return 0.0
        return self.samples[-1][0]

    def update(self, data_time, data_size):
        self.samples.append((data_time, data_size))

    def getStats(self):
        stats_dict = self.getStatsSummary()
        timestamps = [sample[0] for sample in self.samples]
        sizes = [sample[1] for sample in self.samples]
        stats_dict["data"] = {"time": timestamps, "size": sizes}
        return stats_dict

    def getStatsRaw(self):
        stats_dict = self.getStatsSummary()
        timestamps = [sample[0] for sample in self.samples]
        sizes = [sample[1] for sample in self.samples]
        stats_dict["data"] = {"time": timestamps, "size": sizes}
        return stats_dict

    def getStatsSummary(self):
        total_count = len(self.samples)
        total_size = sum([sample[1] for sample in self.samples])

        stats_dict = {
            # total stats
            "window": 0,
            "total_count": total_count,
            "total_size": total_size,
        }
        if total_count < 1:
            stats_dict.update({"total_time": 0, "total_freq": "", "total_bw": ""})
        else:
            duration_secs = self.getDuration()
            stats_dict.update(
                {
                    "total_time": duration_secs,
                    "total_freq": float(total_count) / duration_secs,
                    "total_bw": float(total_size) / duration_secs,
                }
            )
        return stats_dict


class WindowTopicStats(RawTopicStats):
    def __init__(self, window_size=0):
        super().__init__()
        self.window_size = window_size

    def getStats(self):
        stats_dict = self.getStatsSummary()

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
            stats_list = convert_listdicts_dictlists(dict_list)
        stats_dict["data"] = stats_list

        return stats_dict

    def getStatsSummary(self):
        stats_dict = super().getStatsSummary()
        stats_dict["window"] = self.window_size
        return stats_dict

    def _spawnBuffer(self) -> ValueBuffer:
        if self.window_size < 1:
            return ListValueBuffer()
        return RingValueBuffer(self.window_size)
