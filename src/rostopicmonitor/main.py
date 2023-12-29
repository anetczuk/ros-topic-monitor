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

import rostopic
import rospy

from rostopicmonitor.rostopiclistener import ROSTopicListener
from rostopicmonitor.topicstats import WindowTopicStats, RawTopicStats
from rostopicmonitor.writer.jsonwriter import write_json_file, write_json_dir
from rostopicmonitor.writer.pandaswriter import write_pandas_file, write_pandas_dir, summary_to_numpy
from rostopicmonitor.utils import convert_listdicts_dictlists


_MAIN_LOGGER_NAME = "rostopicmonitor"

_LOGGER = logging.getLogger(_MAIN_LOGGER_NAME)


# ============================================================


def process_list(_):
    try:
        topics_list = get_all_publishers()
        # _LOGGER.info("found topics:\n%s", topics_list)
        _LOGGER.info("found topics:\n%s", "\n".join(topics_list))
    except ConnectionRefusedError:
        _LOGGER.error("master not running")


def process_raw(args):
    topic_filter = args.topic
    listeners_dict = init_listeners(topic_filter)

    # listener: TopicListener
    for listener in listeners_dict.values():
        collector = RawTopicStats()
        listener.setStatsCollector(collector)

    execute_listeners(listeners_dict, args)
    store_data(listeners_dict, args)


def process_stats(args):
    topic_filter = args.topic
    listeners_dict = init_listeners(topic_filter)

    window_size = args.window

    # listener: TopicListener
    for listener in listeners_dict.values():
        collector = WindowTopicStats(window_size)
        listener.setStatsCollector(collector)

    execute_listeners(listeners_dict, args)
    store_data(listeners_dict, args)


# ============================================================


def init_listeners(topic_filter):
    try:
        topics_list = get_all_publishers()
    except ConnectionRefusedError:
        _LOGGER.error("master not running")
        return {}

    topics_list = filter_items(topics_list, topic_filter)

    # ['/rosout', '/rosout_agg', '/turtle1/cmd_vel', '/turtle1/color_sensor', '/turtle1/pose']
    _LOGGER.info("subscribing to topics: %s", topics_list)

    listeners_dict = {}
    for topic in topics_list:
        topic_stats = ROSTopicListener(topic)
        listeners_dict[topic] = topic_stats
    return listeners_dict


def execute_listeners(listeners_dict, args):
    init_ros_node(args.logall)

    for listener in listeners_dict.values():
        listener.start()

    _LOGGER.info("Starting listening")

    start_time = datetime.datetime.now()

    mon_duration = args.duration
    if mon_duration < 1:
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    else:
        # spin ROS for given time
        rospy.sleep(mon_duration)
        rospy.signal_shutdown("timeout")

    duration = datetime.datetime.now() - start_time
    _LOGGER.info("Listening duration: %s", duration)

    for listener in listeners_dict.values():
        listener.stop()


def store_data(listeners_dict, args):
    if not listeners_dict:
        return

    out_file = args.outfile
    out_dir = args.outdir
    if not out_file and not out_dir:
        # nothing to store
        _LOGGER.info("Calculating summary")
        data_dict = get_stats(listeners_dict, with_data=False)
        summary_dict = calculate_summary(data_dict)
        summary_dataframe = summary_to_numpy(summary_dict)
        _LOGGER.info("Summary:\n%s", summary_dataframe)
        _LOGGER.info("nothing to store (no file or dir passed to store data)")
        return

    _LOGGER.info("Calculating statistics")
    data_dict = get_stats(listeners_dict, with_data=True)

    summary_dict = {}
    calc_summary = not args.nosummary
    if calc_summary:
        # calculate summary dict
        _LOGGER.info("Calculating summary")
        summary_dict = calculate_summary(data_dict)
        summary_dataframe = summary_to_numpy(summary_dict)
        _LOGGER.info("Summary:\n%s", summary_dataframe)

    out_format = args.outformat
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


def get_stats(listeners_dict, with_data=True):
    data_dict = {}
    if with_data:
        for topic, listener in listeners_dict.items():
            stats_data = listener.getStats()
            data_dict[topic] = stats_data
    else:
        for topic, listener in listeners_dict.items():
            stats_data = listener.getStatsSummary()
            data_dict[topic] = stats_data
    data_dict = dict(sorted(data_dict.items()))  # sort keys in dict
    return data_dict


def calculate_summary(data_dict):
    data_list = list(data_dict.values())
    summary_dict = convert_listdicts_dictlists(data_list)
    summary_dict.pop("data")
    return summary_dict


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
    # anonymous=True flag means that rospy will choose a unique name
    rospy.init_node("topic_monitor", anonymous=True)
    rospy.on_shutdown(shutdown_node)

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


def get_all_publishers():
    pubs_subs = rostopic.get_topic_list()
    publishers = pubs_subs[0]

    topics_list = set()
    topics_list.update([pub[0] for pub in publishers])
    return topics_list


## =====================================================


def add_common_args(parser):
    parser.add_argument("-la", "--logall", action="store_true", help="Log all messages")
    parser.add_argument(
        "--topic",
        metavar="N",
        type=str,
        nargs="+",
        help="Space separated list of regex strings applied on found topics to listen on. "
        "Example: '--topic '/turtle1/.*' '/ros.*'",
    )
    parser.add_argument(
        "--duration",
        action="store",
        type=int_positive,
        default=0,
        help="Set monitor time in seconds. Stop application after timeout.",
    )
    parser.add_argument("--nosummary", action="store_true", help="Do not generate topics summary.")
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
    add_common_args(subparser)
    subparser.add_argument(
        "-w",
        "--window",
        action="store",
        type=int_positive,
        default=0,
        help="Set window size, otherwise collect all samples.",
    )

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
    return 0


if __name__ == "__main__":
    main()
