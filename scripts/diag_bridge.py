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
import copy

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from math import isnan

class Connection:

    def __init__(self):
	self.stats = [0]*60

def tree():
    return collections.defaultdict(tree)

store = tree()

def publishDiagnostics(c):
    d = DiagnosticStatus()

    if c.stats[-2]==0:
	return

    d.name = c.stats[-1].topic+" ("+c.stats[-1].node_pub+" -> "+c.stats[-1].node_sub+")"

    if c.stats[-1].changes_period > c.stats[-2].changes_period and c.stats[-1].changes_delay > c.stats[-2].changes_delay:
	d.level = DiagnosticStatus.WARN
	d.message = "Period and Delay changed!."
    elif c.stats[-1].changes_period > c.stats[-2].changes_period:
	d.level = DiagnosticStatus.WARN
	d.message = "Period changed!"
    elif c.stats[-1].changes_delay > c.stats[-2].changes_delay:
	d.level = DiagnosticStatus.WARN
	d.message = "Delay changed!"
    else:
	d.level = DiagnosticStatus.OK
	d.message = "No recent changes observed."

    if not c.stats[0]==0:
	old = c.stats[0]
	new = c.stats[-1]
	relax = 2
	if old.changes_period+relax < new.changes_period or old.changes_delay+relax < new.changes_delay:
	    d.level = DiagnosticStatus.ERROR
	    d.message = "Topic is unstable!"

    d.hardware_id = c.stats[-1].topic
    d.values.append(KeyValue("period", str(c.stats[-1].period_mean)))
    if not isnan(c.stats[-1].stamp_delay_mean):
	d.values.append(KeyValue("delay", str(c.stats[-1].stamp_delay_mean)))

    d.values.append(KeyValue("e_period", str(c.stats[-1].e_period)))
    d.values.append(KeyValue("L_period", str(c.stats[-1].L_period)))
    d.values.append(KeyValue("e_delay", str(c.stats[-1].e_delay)))
    d.values.append(KeyValue("L_delay", str(c.stats[-1].L_delay)))

    d.values.append(KeyValue("change events period", str(c.stats[-1].changes_period)))
    d.values.append(KeyValue("change events delay", str(c.stats[-1].changes_delay)))
    d.values.append(KeyValue("change prev events period", str(c.stats[-2].changes_period)))
    d.values.append(KeyValue("change prev events delay", str(c.stats[-2].changes_delay)))

    msg = DiagnosticArray()
    msg.header.stamp = rospy.Time.now()
    msg.status.append(d)
    pub.publish(msg)

def newstats(data, args):

    if data.topic.startswith("/statistics") or data.topic.startswith("/diagnostics") or data.topic.endswith("parameter_updates"):
	return

    if not isinstance(store[data.topic][data.node_sub][data.node_pub], Connection):
	store[data.topic][data.node_sub][data.node_pub] = Connection()
	rospy.loginfo("new topic %s: %s -> %s", data.topic,data.node_pub,data.node_sub)

    # assume that stats are published at least a second

    store[data.topic][data.node_sub][data.node_pub].stats.pop(0)
    store[data.topic][data.node_sub][data.node_pub].stats.append(copy.deepcopy(data))

    publishDiagnostics(store[data.topic][data.node_sub][data.node_pub])

if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    pub = rospy.Publisher('/diagnostics', DiagnosticArray)
    rospy.Subscriber("/statistics", TopicStatistics, newstats, 1)
    rospy.spin()

