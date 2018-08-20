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
        
    def test_joint_states_publisher(self):
        for i in range(0,5):
            time.sleep(5)
            try:
                _ = rospy.wait_for_message("joint_states", JointState, timeout=1)
                status = 0
                break
            except:
                status = 1
                if i < 4:
                    subprocess.call('echo "Can\'t get message from joint_states topic, remaing retries: %d"' % (4-i), shell=True)
        self.assertEqual(status, 0, "joint_state topic is not up")
        
    def test_rest_api(self):
        retry_count = 30
        for i in range(retry_count):
            time.sleep(5)
            server_url = os.environ['REST_SERVER_URI'] + "/joint_positions"
            try:
                urllib2.urlopen(server_url).read()
                status = 0
                break
            except:
                status = 1
                remaining = retry_count - 1 - i 
                if 0 < remaining:
                    subprocess.call(
                        'echo "Can\'t connect to REST server, remaining retries: %d"' % remaining, shell=True)
        self.assertEqual(status, 0, "REST server is not running")


if __name__ == "__main__":
    rospy.init_node('system_test_node', anonymous=True)
    unittest.main()
