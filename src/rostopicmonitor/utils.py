#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#


def prepare_filesystem_name(name):
    new_name = name
    new_name = new_name.replace("/", "_")
    new_name = new_name.replace("|", "_")
    new_name = new_name.replace("-", "_")
    return new_name


def convert_listdicts_dictlists(dict_list):
    return {dict_key: [item_dict[dict_key] for item_dict in dict_list] for dict_key in dict_list[0]}
