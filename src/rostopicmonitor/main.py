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

import rostopic
import rospy

from rostopicmonitor.rostopiclistener import ROSTopicListener
from rostopicmonitor import utils
from rostopicmonitor.topicstats import WindowTopicStats, RawTopicStats


_MAIN_LOGGER_NAME = "rostopicmonitor"

_LOGGER = logging.getLogger(_MAIN_LOGGER_NAME)


# ============================================================


def process_stats(args):
    listeners_dict = init_listeners()

    window_size = args.window

    # listener: TopicListener
    for listener in listeners_dict.values():
        collector = WindowTopicStats(window_size)
        listener.setStatsCollector(collector)

    execute_listeners(listeners_dict, args)
    store_data(listeners_dict, args)


def process_raw(args):
    listeners_dict = init_listeners()

    # listener: TopicListener
    for listener in listeners_dict.values():
        collector = RawTopicStats()
        listener.setStatsCollector(collector)

    execute_listeners(listeners_dict, args)
    store_data(listeners_dict, args)


# ============================================================


def init_listeners():
    try:
        topics_list = get_all_publishers()
    except ConnectionRefusedError:
        _LOGGER.error("master not running")
        return {}

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

    mon_duration = args.duration
    if mon_duration < 1:
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    else:
        # spin ROS for given time
        rospy.sleep(mon_duration)
        rospy.signal_shutdown("timeout")

    for listener in listeners_dict.values():
        listener.stop()

    for listener in listeners_dict.values():
        listener.printInfo()


def store_data(stats_dict, args):
    out_file = args.outfile
    out_dir = args.outdir
    if not out_file and not out_dir:
        # nothing to store
        return

    data_dict = {}
    for topic, listener in stats_dict.items():
        stats_data = listener.getStats()
        data_dict[topic] = stats_data
    data_dict = dict(sorted(data_dict.items()))  # sort keys in dict

    if out_file:
        _LOGGER.info("writing output to file: %s", out_file)
        dir_path = os.path.dirname(os.path.realpath(out_file))
        os.makedirs(dir_path, exist_ok=True)
        utils.write_data_file(out_file, data_dict)

    if out_dir:
        _LOGGER.info("writing output to directory: %s", out_dir)
        os.makedirs(out_dir, exist_ok=True)
        utils.write_data_dir(out_dir, data_dict)


## =====================================================


def init_ros_node(logall=False):
    # anonymous=True flag means that rospy will choose a unique name
    rospy.init_node("topic_monitor", anonymous=True)
    rospy.on_shutdown(shutdown_node)

    # configure loggers AFTER 'init_node'
    logging.basicConfig()

    main_logger = logging.getLogger(_MAIN_LOGGER_NAME)
    if logall:
        main_logger.setLevel(logging.DEBUG)
    else:
        main_logger.setLevel(logging.INFO)

    # configure logger to receive messages to command line
    consoleHandler = logging.StreamHandler(stream=sys.stdout)
    formatter = logging.Formatter("[%(levelname)-8s] %(name)-12s: %(message)s")
    consoleHandler.setFormatter(formatter)

    main_logger.addHandler(consoleHandler)


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
        "--duration",
        action="store",
        type=int_positive,
        default=0,
        help="Set monitor time in seconds. Stop application after timeout.",
    )
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
    parser.set_defaults(func=process_stats)

    subparsers = parser.add_subparsers(help="one of tools", description="use one of tools", dest="tool", required=False)

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

    description = "collect and store raw data (sizes of messages)"
    subparser = subparsers.add_parser("raw", help=description)
    subparser.description = description
    subparser.set_defaults(func=process_raw)
    add_common_args(subparser)

    ## =================================================

    args = parser.parse_args()

    if args.listtools is True:
        tools_list = list(subparsers.choices.keys())
        print(", ".join(tools_list))
        return 0

    args.func(args)
    return 0


if __name__ == "__main__":
    main()
