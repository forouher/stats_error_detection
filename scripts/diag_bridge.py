#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Dariush Forouher
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

NAME = 'diag_bridge'

import sys

PKG = 'stats_error_detection' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
from rosgraph_msgs.msg import TopicStatistics
import collections

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from math import isnan

class Connection:

    last_good_stats = 0;
    last_stats = 0;

def tree():
    return collections.defaultdict(tree)

store = tree()

def publishDiagnostics(c):
    d = DiagnosticStatus()
    d.name = c.last_stats.topic+" ("+c.last_stats.node_pub+" -> "+c.last_stats.node_sub+")"
    if (c.last_stats.status & TopicStatistics.STATUS_ERROR_PERIOD) and (c.last_stats.status & TopicStatistics.STATUS_ERROR_DELAY):
	d.level = DiagnosticStatus.WARN
	d.message = "Period and Delay changed!."
    elif c.last_stats.status & TopicStatistics.STATUS_ERROR_PERIOD:
	d.level = DiagnosticStatus.WARN
	d.message = "Period changed!"
    elif c.last_stats.status & TopicStatistics.STATUS_ERROR_DELAY:
	d.level = DiagnosticStatus.WARN
	d.message = "Delay changed!"
    elif c.last_stats.status & TopicStatistics.STATUS_OK:
	d.level = DiagnosticStatus.OK
	d.message = "Good."
    else:
	d.level = DiagnosticStatus.OK
	d.message = "Unknown status."

    d.hardware_id = c.last_stats.topic
    d.values.append(KeyValue("period", str(c.last_stats.period_mean)))
    if not isnan(c.last_stats.stamp_delay_mean):
	d.values.append(KeyValue("delay", str(c.last_stats.stamp_delay_mean)))

    d.values.append(KeyValue("e_period", str(c.last_stats.e_period)))
    d.values.append(KeyValue("L_period", str(c.last_stats.L_period)))
    d.values.append(KeyValue("e_delay", str(c.last_stats.e_delay)))
    d.values.append(KeyValue("L_delay", str(c.last_stats.L_delay)))

    msg = DiagnosticArray()
    msg.header.stamp = rospy.Time.now()
    msg.status.append(d)
    pub.publish(msg)

def newstats(data, args):

    if data.topic.startswith("/statistics") or data.topic.startswith("/diagnostics"):
	return

    if not isinstance(store[data.topic][data.node_sub][data.node_pub], Connection):
	store[data.topic][data.node_sub][data.node_pub] = Connection()
	rospy.loginfo("new topic %s: %s -> %s", data.topic,data.node_pub,data.node_sub)

    # assume that stats are published at least a second

    store[data.topic][data.node_sub][data.node_pub].last_stats = data

    if data.status == TopicStatistics.STATUS_OK:
	store[data.topic][data.node_sub][data.node_pub].last_good_stats = data
    
    publishDiagnostics(store[data.topic][data.node_sub][data.node_pub])


if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    pub = rospy.Publisher('/diagnostics', DiagnosticArray)
    rospy.Subscriber("/statistics", TopicStatistics, newstats, 1)
    rospy.spin()

