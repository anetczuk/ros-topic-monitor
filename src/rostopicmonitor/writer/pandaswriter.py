#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os

import pandas

from rostopicmonitor.utils import prepare_filesystem_name


def write_pandas_simple_file(out_file_path, numpy_data_list, out_format):
    if out_format == "json":
        with open(out_file_path, "w", encoding="utf-8") as data_file:
            # clear file content
            pass

        for numpy_data in numpy_data_list:
            with open(out_file_path, "a", encoding="utf-8") as data_file:
                numpy_data.to_json(data_file, orient="records")

    elif out_format == "csv":
        with open(out_file_path, "w", encoding="utf-8") as data_file:
            # clear file content
            pass

        for numpy_data in numpy_data_list:
            with open(out_file_path, "a", encoding="utf-8") as data_file:
                numpy_data.to_csv(data_file, sep=";", encoding="utf-8")
            with open(out_file_path, "a", encoding="utf-8") as data_file:
                data_file.write(";\n;\n")

    elif out_format == "xls":
        sheet_name = "data"
        # pylint: disable=abstract-class-instantiated
        with pandas.ExcelWriter(out_file_path, engine="openpyxl") as writer:
            start_row = 0
            for numpy_data in numpy_data_list:
                numpy_data.to_excel(writer, sheet_name=sheet_name, index=False, startrow=start_row)  # send df to writer
                start_row += numpy_data.shape[0] + 3

    elif out_format == "xlsx":
        sheet_name = "data"
        # pylint: disable=abstract-class-instantiated
        with pandas.ExcelWriter(out_file_path, engine="xlsxwriter") as writer:
            col_width_dict = {}
            start_row = 0
            for numpy_data in numpy_data_list:
                numpy_data.to_excel(writer, sheet_name=sheet_name, index=False, startrow=start_row)  # send df to writer
                start_row += numpy_data.shape[0] + 3

                for idx, col in enumerate(numpy_data):  # loop through all columns
                    series = numpy_data[col]
                    item_len = series.astype(str).map(len).max()  # len of largest item
                    name_len = len(str(series.name))  # len of column name/header
                    max_len = max(item_len, name_len) + 1  # adding a little extra space
                    curr_width = col_width_dict.get(idx, 0)
                    col_width_dict[idx] = max(max_len, curr_width)

            worksheet = writer.sheets[sheet_name]  # pull worksheet object
            for idx, width in col_width_dict.items():  # loop through all columns
                worksheet.set_column(idx, idx, width)  # set column width


def write_pandas_file(out_file_path, data_dict, out_format):
    numpy_list = []
    for data in data_dict.values():
        numpy_data_list = dict_to_numpy(data)
        numpy_list.extend(numpy_data_list)
    write_pandas_simple_file(out_file_path, numpy_list, out_format)


def write_pandas_dir(out_dir, data_dict, out_format):
    for topic, data in data_dict.items():
        file_name = prepare_filesystem_name(topic)
        file_out_path = os.path.join(out_dir, f"{file_name}.{out_format}")
        numpy_data_list = dict_to_numpy(data)
        write_pandas_simple_file(file_out_path, numpy_data_list, out_format)


def dict_to_numpy(data_dict):
    copied_dict = data_dict.copy()
    data_samples = copied_dict.pop("data")

    numpy_header_data = pandas.json_normalize(copied_dict)
    numpy_samples_data = pandas.DataFrame.from_dict(data_samples)

    return (numpy_header_data, numpy_samples_data)
