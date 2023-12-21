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

from rostopicmonitor.rostopicstats import ROSTopicStats
from rostopicmonitor import utils


_MAIN_LOGGER_NAME = "rostopicmonitor"

_LOGGER = logging.getLogger(_MAIN_LOGGER_NAME)


# ============================================================


def start_process(args):
    try:
        topics_list = get_all_publishers()
    except ConnectionRefusedError:
        _LOGGER.error("master not running")
        return

    init_ros_node(args.logall)

    # ['/rosout', '/rosout_agg', '/turtle1/cmd_vel', '/turtle1/color_sensor', '/turtle1/pose']
    _LOGGER.info("subscribing to topics: %s", topics_list)

    stats_dict = subscribe_to(topics_list)

    rospy.spin()  # simply keeps python from exiting until this node is stopped

    for stats in stats_dict.values():
        stats.stop()

    for stats in stats_dict.values():
        stats.printInfo()

    out_file = args.outfile
    out_dir = args.outdir
    if not out_file and not out_dir:
        # nothing to store
        return

    data_dict = {}
    for topic, stats in stats_dict.items():
        stats_dict = stats.getStats()
        topic_dict = {"topic": topic}
        topic_dict.update(stats_dict)
        data_dict[topic] = topic_dict
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


def subscribe_to(topic_list):
    stats_dict = {}
    for topic in topic_list:
        topic_stats = ROSTopicStats(topic)
        topic_stats.start()
        stats_dict[topic] = topic_stats
    return stats_dict


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


def main():
    parser = argparse.ArgumentParser(description="ROS parse tools")
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

    args = parser.parse_args()

    start_process(args)

    return 0


if __name__ == "__main__":
    main()
