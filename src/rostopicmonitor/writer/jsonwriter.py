#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import json

from rostopicmonitor.utils import prepare_filesystem_name


def write_json_file(out_file, dict_data, summary_data=None):
    out_data = dict_data
    if summary_data:
        out_data = [summary_data, dict_data]

    with open(out_file, "w", encoding="utf8") as fp:
        json.dump(out_data, fp, indent=4)


def write_json_dir(out_dir, data_dict, summary_data=None):
    for topic, data in data_dict.items():
        file_name = prepare_filesystem_name(topic)
        file_out_path = os.path.join(out_dir, file_name + ".txt")
        write_json_file(file_out_path, data)

    if summary_data:
        file_out_path = os.path.join(out_dir, "summary.txt")
        write_json_file(file_out_path, summary_data)
