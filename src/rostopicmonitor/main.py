#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import sys
import os
import logging
import argparse
import re
import datetime
import json
from typing import Dict

from rostopicmonitor.topicstats import WindowTopicStats, RawTopicStats
from rostopicmonitor.writer.jsonwriter import write_json_file, write_json_dir
from rostopicmonitor.writer.pandaswriter import write_pandas_file, write_pandas_dir, summary_to_numpy
from rostopicmonitor.utils import convert_listdicts_dictlists
from rostopicmonitor.topiclistener import TopicListener, StatsObject


_MAIN_LOGGER_NAME = "rostopicmonitor"

_LOGGER = logging.getLogger(_MAIN_LOGGER_NAME)


# ============================================================


def process_list(_):
    try:
        topics_list = get_publishers()
        # _LOGGER.info("found topics:\n%s", topics_list)
        _LOGGER.info("found topics:\n%s", "\n".join(topics_list))
    except ConnectionRefusedError:
        _LOGGER.error("master not running")


def process_raw(args):
    topic_filter = args.topic
    topic_filter = get_topic_filters(topic_filter)
    listeners_dict: Dict[str, TopicListener] = init_listeners(topic_filter)

    # listener: TopicListener
    for listener in listeners_dict.values():
        collector = RawTopicStats()
        listener.setStatsCollector(collector)

    execute_listeners(listeners_dict, args)
    stats_dict: Dict[str, StatsObject] = extract_stats_obj_dict(listeners_dict)
    store_data(stats_dict, args)


def process_stats(args):
    if args.fromrawfile:
        process_stats_file(args)
    else:
        process_stats_ros(args)


def process_stats_file(args):
    topic_filter = args.topic
    in_raw_file = args.fromrawfile
    in_raw_file = os.path.abspath(in_raw_file)

    _LOGGER.info("Loading raw data from file %s", in_raw_file)

    with open(in_raw_file, "r", encoding="utf8") as fp:
        raw_dict = json.load(fp)

    topics_list = list(raw_dict.keys())
    topics_list = filter_items(topics_list, topic_filter)

    window_size = args.window

    # listener: TopicListener
    stats_dict: Dict[str, StatsObject] = {}
    for topic in topics_list:
        data_dict = raw_dict.get(topic, {})
        header_dict = {"topic": data_dict["topic"], "fixed_size": data_dict["fixed_size"]}
        collector = WindowTopicStats(window_size)
        collector.setSamplesFromDict(data_dict)
        stats_dict[topic] = StatsObject(header_dict, collector)

    store_data(stats_dict, args)


def process_stats_ros(args):
    topic_filter = args.topic
    topic_filter = get_topic_filters(topic_filter)
    listeners_dict = init_listeners(topic_filter)

    window_size = args.window

    # listener: TopicListener
    for listener in listeners_dict.values():
        collector = WindowTopicStats(window_size)
        listener.setStatsCollector(collector)

    execute_listeners(listeners_dict, args)
    stats_dict: Dict[str, StatsObject] = extract_stats_obj_dict(listeners_dict)
    store_data(stats_dict, args)


# ============================================================


def init_listeners(topic_filter) -> Dict[str, TopicListener]:
    # not nice, but allows running some functionality on machine without ROS
    from rostopicmonitor.rostopiclistener import ROSTopicListener

    try:
        topics_list = get_publishers()
    except ConnectionRefusedError:
        _LOGGER.error("master not running")
        return {}

    _LOGGER.info("subscribing to topics: %s", topics_list)

    topics_list = filter_items(topics_list, topic_filter)
    listeners_dict: Dict[str, TopicListener] = {}
    for topic in topics_list:
        topic_stats = ROSTopicListener(topic)
        listeners_dict[topic] = topic_stats
    return listeners_dict


def execute_listeners(listeners_dict: Dict[str, TopicListener], args):
    # not nice, but allows running some functionality on machine without ROS
    from rostopicmonitor.rosutils import ros_spin

    init_ros_node(args.logall)

    for listener in listeners_dict.values():
        listener.start()

    _LOGGER.info("Starting listening. Press CTRL+C to stop and write data.")

    start_time = datetime.datetime.now()

    ros_spin(args.duration)

    duration = datetime.datetime.now() - start_time
    _LOGGER.info("Listening duration: %s", duration)

    for listener in listeners_dict.values():
        listener.stop()


def extract_stats_obj_dict(listeners_dict: Dict[str, TopicListener]) -> Dict[str, StatsObject]:
    stats_dict: Dict[str, StatsObject] = {}
    for topic, listener in listeners_dict.items():
        stats_dict[topic] = listener.getStatsObject()
    return stats_dict


def store_data(stats_dict: Dict[str, StatsObject], args):
    if not stats_dict:
        return

    out_file = args.outfile
    out_dir = args.outdir
    if not out_file and not out_dir:
        # nothing to store
        _LOGGER.info("Calculating summary")
        data_dict = get_stats_summary(stats_dict)
        summary_dict = calculate_summary(data_dict)
        summary_dataframe = summary_to_numpy(summary_dict)
        _LOGGER.info("Summary:\n%s", summary_dataframe)
        _LOGGER.info("nothing to store (no file or dir passed to store data)")
        return

    if "nostoreraw" in args and not args.nostoreraw:
        raw_dict = get_stats_raw(stats_dict)
        store_raw_data(raw_dict, out_file, out_dir)

    _LOGGER.info("Calculating statistics")
    calc_stddev = args.calcstddev
    data_dict = get_stats(stats_dict, stddev=calc_stddev)

    out_format = args.outformat
    calc_summary = not args.nosummary

    store_data_dict(data_dict, out_file, out_dir, out_format, calc_summary)


def store_data_dict(data_dict, out_file=None, out_dir=None, out_format=None, calc_summary=None):
    summary_dict = {}
    if calc_summary:
        # calculate summary dict
        _LOGGER.info("Calculating summary")
        summary_dict = calculate_summary(data_dict)
        summary_dataframe = summary_to_numpy(summary_dict)
        _LOGGER.info("Summary:\n%s", summary_dataframe)

    if out_format == "json":
        if out_file:
            _LOGGER.info("Writing output to file: %s", out_file)
            dir_path = os.path.dirname(os.path.realpath(out_file))
            os.makedirs(dir_path, exist_ok=True)
            write_json_file(out_file, data_dict)  # do not store summary_dict in single file mode

        if out_dir:
            _LOGGER.info("Writing output to directory: %s", out_dir)
            os.makedirs(out_dir, exist_ok=True)
            write_json_dir(out_dir, data_dict, summary_dict)
    else:
        if out_file:
            out_file_path = f"{out_file}.{out_format}"
            _LOGGER.info("Writing output to file: %s", out_file_path)
            dir_path = os.path.dirname(os.path.realpath(out_file_path))
            os.makedirs(dir_path, exist_ok=True)
            write_pandas_file(out_file_path, data_dict, out_format, summary_dict)

        if out_dir:
            _LOGGER.info("Writing output to directory: %s", out_dir)
            os.makedirs(out_dir, exist_ok=True)
            write_pandas_dir(out_dir, data_dict, out_format, summary_dict)


def store_raw_data(data_dict, out_file=None, out_dir=None):
    if out_file:
        out_raw_path = f"{out_file}.raw.json"
        _LOGGER.info("Writing raw data to file: %s", out_raw_path)
        dir_path = os.path.dirname(os.path.realpath(out_raw_path))
        os.makedirs(dir_path, exist_ok=True)
        write_json_file(out_raw_path, data_dict)  # do not store summary_dict in single file mode

    if out_dir:
        out_raw_path = os.path.join(out_dir, "raw_data.json")
        _LOGGER.info("Writing raw data to file: %s", out_raw_path)
        os.makedirs(out_dir, exist_ok=True)
        write_json_file(out_raw_path, data_dict)  # do not store summary_dict in single file mode


def get_stats(stats_dict: Dict[str, StatsObject], stddev=True):
    data_dict = {}
    for topic, stats_obj in stats_dict.items():
        stats_data = stats_obj.getStats(stddev=stddev)
        data_dict[topic] = stats_data
    data_dict = dict(sorted(data_dict.items()))  # sort keys in dict
    return data_dict


def get_stats_raw(stats_dict: Dict[str, StatsObject]):
    data_dict = {}
    for topic, listener in stats_dict.items():
        stats_data = listener.getStatsRaw()
        data_dict[topic] = stats_data
    data_dict = dict(sorted(data_dict.items()))  # sort keys in dict
    return data_dict


def get_stats_summary(stats_dict: Dict[str, StatsObject]):
    data_dict = {}
    for topic, listener in stats_dict.items():
        stats_data = listener.getStatsSummary()
        data_dict[topic] = stats_data
    data_dict = dict(sorted(data_dict.items()))  # sort keys in dict
    return data_dict


def calculate_summary(data_dict):
    data_list = list(data_dict.values())
    summary_dict = convert_listdicts_dictlists(data_list)
    if "data" in summary_dict:
        summary_dict.pop("data")
    return summary_dict


def get_topic_filters(topic_filer_data_list):
    if topic_filer_data_list is None:
        return None
    topic_filters = []
    for filter_item in topic_filer_data_list:
        if not os.path.isfile(filter_item):
            # not a file -- assume normal regex
            topic_filters.append(filter_item)
        # existing file - read regexes from it
        with open(filter_item, "r", encoding="utf8") as fp:
            regex_lines = fp.readlines()
            for file_item in regex_lines:
                item = file_item.strip()  # we do not want any prefix/suffix whitespaces or newline characters
                if not item:
                    # skip empty string
                    continue
                topic_filters.append(item)
    return topic_filters


def filter_items(items_list, regex_list):
    if not items_list:
        return items_list
    if not regex_list:
        return items_list

    ret_list = []
    pattern_list = [re.compile(item) for item in regex_list]
    for item in items_list:
        for pattern in pattern_list:
            if pattern.match(item):
                ret_list.append(item)
                continue

    return ret_list


## =====================================================


_LOGGER_SIMPLE_FORMAT = "%(levelname)-8s %(name)-12s: %(message)s"
_LOGGER_ALL_FORMAT = "[%(asctime)s] %(levelname)-8s %(name)-12s: %(message)s"


def init_ros_node(logall=False):
    # not nice, but allows running some functionality on machine without ROS
    from rostopicmonitor.rosutils import ros_init

    ros_init(shutdown_node)

    # configure logger to receive messages to command line (rospy overrides logging configuration)
    consoleHandler = logging.StreamHandler(stream=sys.stdout)
    if logall:
        formatter = logging.Formatter(_LOGGER_ALL_FORMAT)
    else:
        formatter = logging.Formatter(_LOGGER_SIMPLE_FORMAT)
    consoleHandler.setFormatter(formatter)

    main_logger = logging.getLogger(_MAIN_LOGGER_NAME)
    main_logger.addHandler(consoleHandler)


def init_logger(logall=False):
    # configure loggers AFTER 'init_node'
    if logall:
        logging.basicConfig(format=_LOGGER_ALL_FORMAT)
    else:
        logging.basicConfig(format=_LOGGER_SIMPLE_FORMAT)

    main_logger = logging.getLogger(_MAIN_LOGGER_NAME)
    if logall:
        main_logger.setLevel(logging.DEBUG)
    else:
        main_logger.setLevel(logging.INFO)


def shutdown_node():
    _LOGGER.info("Stopping monitor")


def get_publishers():
    # not nice, but allows running some functionality on machine without ROS
    from rostopicmonitor.rosutils import get_all_publishers

    return get_all_publishers()


## =====================================================


def add_common_args(parser):
    parser.add_argument(
        "--topic",
        metavar="N",
        type=str,
        nargs="+",
        help="""Space separated list of regex strings applied on found topics to listen on."""
        """ It can also be paths to files with regexes. Or mix of both."""
        """ Example: "--topic '/turtle1/.*' '/ros.*'" """,
    )
    parser.add_argument(
        "--duration",
        action="store",
        type=int_positive,
        default=0,
        help="Set monitor time in seconds. Stop application after timeout.",
    )
    parser.add_argument("--nosummary", action="store_true", help="Do not generate topics summary.")
    parser.add_argument("--calcstddev", action="store_true", help="Calculate standard deviation (time consuming).")
    parser.add_argument("-la", "--logall", action="store_true", help="Log all messages")
    parser.add_argument(
        "--outfile",
        action="store",
        required=False,
        default="",
        help="Path to output file (store collected data in single file).",
    )
    parser.add_argument(
        "--outdir",
        action="store",
        required=False,
        default="",
        help="Path to output dir (store collected data in directory).",
    )
    parser.add_argument(
        "--outformat",
        action="store",
        required=False,
        choices=["json", "csv", "xls", "xlsx"],
        default="json",
        help="Output format. Default: %(default)s.",
    )


def int_positive(value):
    ivalue = int(value)
    if ivalue <= 0:
        raise argparse.ArgumentTypeError(f"expected positive int, invalid value: {value}")
    return ivalue


def main():
    main_description = (
        "ROS topic measurement tools."
        " Collect size of messages passed through topics and calculate various statistics."
        " All data is measured in Bytes, Secounds and Hertz accordingly."
    )
    parser = argparse.ArgumentParser(description=main_description)
    parser.add_argument("--listtools", action="store_true", help="List tools")
    parser.set_defaults(func=None)

    subparsers = parser.add_subparsers(help="one of tools", description="use one of tools", dest="tool", required=False)

    ## =================================================

    description = "list topics"
    subparser = subparsers.add_parser("list", help=description)
    subparser.description = description
    subparser.set_defaults(func=process_list)
    parser.add_argument("-la", "--logall", action="store_true", help="Log all messages")

    ## =================================================

    description = "collect and store raw data (sizes of messages)"
    subparser = subparsers.add_parser("raw", help=description)
    subparser.description = description
    subparser.set_defaults(func=process_raw)
    add_common_args(subparser)

    ## =================================================

    description = "collect and store stats data"
    subparser = subparsers.add_parser("stats", help=description)
    subparser.description = description
    subparser.set_defaults(func=process_stats)
    subparser.add_argument(
        "-w",
        "--window",
        action="store",
        type=int_positive,
        default=0,
        help="Set window size, otherwise collect all samples.",
    )
    subparser.add_argument("--nostoreraw", action="store_true", help="Do not store raw data additionally.")
    subparser.add_argument(
        "--fromrawfile", action="store", required=False, default=None, help="Path to raw file to get data from."
    )
    add_common_args(subparser)

    ## =================================================

    args = parser.parse_args()

    init_logger(args.logall)

    if args.listtools is True:
        tools_list = list(subparsers.choices.keys())
        print(", ".join(tools_list))
        return 0

    if not args.func:
        ## no command given -- print help message
        parser.print_help()
        sys.exit(1)
        return 1

    args.func(args)

    _LOGGER.info("Completed")
    return 0


if __name__ == "__main__":
    main()
