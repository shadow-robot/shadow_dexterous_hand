#!/usr/bin/env python
# Copyright 2019, 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
import os
import subprocess
import rospy
import unittest
import time
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
