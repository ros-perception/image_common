#!/usr/bin/env python
#*
#*  Copyright (c) 2010, Jack O'Quin
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the author nor the names of other
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

PACKAGE='camera_info_manager'
import roslib; roslib.load_manifest(PACKAGE)
import sys
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

# Default service name for most ROS camera drivers.  Would be nice to
# make this a parameter.
service_name = 'camera/set_camera_info'

def test():
    rospy.wait_for_service(service_name)

    # build theoretical calibration for Unibrain Fire-i camera in
    # 640x480 mode
    cinfo = CameraInfo()
    cinfo.width = 640
    cinfo.height = 480
    cx = (cinfo.width - 1.0)/2.0
    cy = (cinfo.height - 1.0)/2.0
    fx = fy = 0.0043                    # Unibrain Fire-i focal length
    cinfo.K = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    cinfo.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    cinfo.P = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]

    try:
        set_camera_info = rospy.ServiceProxy(service_name, SetCameraInfo)
        response = set_camera_info(cinfo)
        rospy.loginfo("set_camera_info: success=" + str(response.success)
                      + ", status=" + str(response.status_message))
        return response.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('set_camera_info test completed')
