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

NAME = 'relay'

import sys

PKG = 'stats_error_detection' # this package name
import roslib; roslib.load_manifest(PKG)
import sys

import rospy
from sensor_msgs.msg import PointCloud2

from dynamic_reconfigure.server import Server
from stats_error_detection.cfg import TalkerConfig

config = 0

def callback(c, level):
    global config
    print "callback"
    config = c
    return c

if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    pub = rospy.Publisher(sys.argv[1], PointCloud2)
    srv = Server(TalkerConfig, callback)
    print "init done"
    while not rospy.is_shutdown():
	msg = PointCloud2()
	msg.header.stamp = rospy.Time.now() - rospy.Duration(config['delay'])
        pub.publish(msg)
        rospy.sleep(config['period'])

