from hsrb_interface import Robot
from hsrb_interface import geometry
import roslib
roslib.load_manifest('mechknownet')
import rospy
import numpy as np
from mechknownet.srv import *


def demo():
    success = call_mechknownet_sys("bottle","cup","pour","three_tasks_demo",[])
    #success = call_mechknownet_sys("hammer","bottle","pound","three_tasks_demo",[])
    #success = call_mechknownet_sys("drawer2","glassbox","drawer_demo",[-0.3,0.0,-0.1,0.1,0.0,0.05])

    print "success?",success
    if success:
        print "success, ready to go"

def call_mechknownet_sys(tool,drawer,action,which_demo,boundingbox):
    got_result_flag = False
    rospy.wait_for_service('mechknownetsys')
    try:
        mechknownet_sys_srv = rospy.ServiceProxy('mechknownetsys', mechknownetsys)
        response = mechknownet_sys_srv(tool,drawer,action,which_demo,boundingbox)
        return response.Success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    try:
        demo()
    except rospy.ROSInterruptException:
        pass

    
