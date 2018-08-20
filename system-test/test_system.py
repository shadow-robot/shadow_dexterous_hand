#!/usr/bin/env python

import os
import subprocess
import rospy
import unittest
import time
import urllib2
from sensor_msgs.msg import JointState


class TestSystem(unittest.TestCase):
    
    def test_roscore(self):
        for i in range(0,5):
            time.sleep(5)
            try:
                rospy.get_master().getPid()
                status = 0
                break
            except:
                status = 1
                if i < 4:
                    subprocess.call('echo "Can\'t connect to roscore, remaing retries: %d"' % (4-i), shell=True)
        self.assertEqual(status, 0, "roscore is not running")

if __name__ == "__main__":
    rospy.init_node('system_test_node', anonymous=True)
    unittest.main()
