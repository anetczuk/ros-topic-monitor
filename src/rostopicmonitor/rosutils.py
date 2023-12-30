#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import rostopic
import rospy
import roslib


def get_all_publishers():
    pubs_subs = rostopic.get_topic_list()
    publishers = pubs_subs[0]

    topics_list = set()
    topics_list.update([pub[0] for pub in publishers])
    return topics_list


def get_topic_class(topic_name):
    return rostopic.get_topic_class(topic_name)


def get_message_class_from_name(type_name):
    return roslib.message.get_message_class(type_name)


def ros_init(shutdown_callback):
    # anonymous=True flag means that rospy will choose a unique name
    rospy.init_node("topic_monitor", anonymous=True)
    rospy.on_shutdown(shutdown_callback)


def ros_subscribe(topic_name, message_class, message_callback):
    rospy.Subscriber(topic_name, message_class, message_callback)


def ros_spin(duration=0):
    if duration < 1:
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    else:
        # spin ROS for given time
        rospy.sleep(duration)
        rospy.signal_shutdown("timeout")
