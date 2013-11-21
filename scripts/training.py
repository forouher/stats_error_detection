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

NAME = 'training'

import sys

PKG = 'stats_error_detection' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
from rosgraph_msgs.msg import TopicStatistics
import collections

class ConnectionLimits:

    period_min = 10000000000
    period_var_max = -1000000
    period_max = -10000000
    delay_max = -10000000
    delay_var_max = -10000000000
    drops_max = -1000000

def tree():
    return collections.defaultdict(tree)

store = tree()

def callback(data, args):
    if not isinstance(store[data.topic][data.node_sub][data.node_pub], ConnectionLimits):
	store[data.topic][data.node_sub][data.node_pub] = ConnectionLimits()
	rospy.loginfo("new topic %s: %s <-> %s", data.topic,data.node_sub,data.node_pub)
    
    c = store[data.topic][data.node_sub][data.node_pub]
    if c.period_min >data.period_mean:
	c.period_min = data.period_mean
    if c.period_max < data.period_max:
	c.period_max = data.period_max
    if c.period_var_max < data.period_variance:
	c.period_var_max = data.period_variance
    if c.delay_max < data.stamp_delay_max:
	c.delay_max = data.stamp_delay_max
    if c.delay_var_max < data.stamp_delay_variance:
	c.delay_var_max = data.stamp_delay_variance
    if c.drops_max < data.dropped_msgs:
	c.drops_max = data.dropped_msgs

if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    rospy.Subscriber("/statistics", TopicStatistics, callback, 1)
    rospy.spin()

