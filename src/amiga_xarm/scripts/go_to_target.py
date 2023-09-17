#!/usr/bin/env python3
import numpy as np
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Point

def mat_to_rpy(R):
    #  yaw=atan2(R(2,1),R(1,1));
    #  pitch=atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2)));
    #  roll=atan2(R(3,2),R(3,3));
    yaw = atan2(R[1,0],R[0,0])
    pitch = atan2(-R[2,0),sqrt(R[2,1]**2+R[2,2]**2)))
    roll = atan2(R[2,1],R[2,2])
    return roll, pitch, yaw


def callback(data):
    target = np.array([data.x, data.y, data.z])
    normal = np.array([-1.0, 0.0, 0.0])
    x0,y0,z0 = np.eye(3)
    dist = np.linalg.norm(target)
    rospy.loginfo(rospy.get_caller_id() + "> distance (to point): %s", dist)
    xT = z0
    zT = -normal
    yT = np.cross(xT, zT)
    rot = np.array([xT, yT, zT]).T
    r,p,y = mat_to_rpy(rot)
    rospy.loginfo(rospy.get_caller_id() + "> RPY: %s %s %s", r, p, y)

    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter_point", Point, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
