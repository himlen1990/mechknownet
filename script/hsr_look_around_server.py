#!/usr/bin/python


import roslib
roslib.load_manifest('mechknownet')
import rospy
from mechknownet.srv import *
from hsrb_interface import Robot

#import tf as ros_tf

class hsr_look_around_server:
    
    def __init__(self):
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.lookaroundSrv = rospy.Service("lookaround", lookaround, self.srvCb)
        rospy.loginfo("hsr_look_around_server: ready to serve")

    def srvCb(self,req):
        rospy.loginfo("hsr_look_around_server: got request")
        tilt_angle = req.TiltAngle
        pan_angle = req.PanAngle
        self.whole_body.move_to_joint_positions({'head_tilt_joint': tilt_angle})
        self.whole_body.move_to_joint_positions({'head_pan_joint': pan_angle})
        rospy.sleep(2)
        return lookaroundResponse(True)

def main(args):
  rospy.init_node('look_around_server', anonymous=True)
  la_server= hsr_look_around_server()

  #try:
  rospy.spin()
  #except KeyboardInterrupt:
  #  print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)




