from hsrb_interface import Robot
from hsrb_interface import geometry
import roslib
roslib.load_manifest('mechknownet')
import rospy
import numpy as np
from mechknownet.srv import *

robot = Robot()
whole_body = robot.try_get('whole_body')
base = robot.try_get('omni_base')
gripper = robot.try_get('gripper')
#whole_body.linear_weight = 30.0
whole_body.looking_hand_constraint = True




def demo():
    success = call_mechknownet_sys("drawer2","","grasp",[])
    print "success?",success
    step1_finish = False
    preposition = None
    if success:
        print "success, ready to go"
        pre_head_pan_joint = whole_body.joint_positions['head_pan_joint']
        pre_head_tilt_joint = whole_body.joint_positions['head_tilt_joint']
        preposition = base.pose
        gripper.command(1.0)
        rospy.sleep(3.0)        
        whole_body.move_end_effector_pose(geometry.pose(z=-0.08), 'grasp_pose')
        rospy.sleep(1.0)
        whole_body.move_end_effector_pose(geometry.pose(z=-0.02), 'grasp_pose')#drawer1 0.045, drawer2, 0.035
        rospy.sleep(1.0)
        gripper.grasp(-0.5)
        rospy.sleep(1.0)
        whole_body.move_end_effector_pose(geometry.pose(z=-0.3), 'grasp_pose')
        #base.go_rel(-0.2,0.0,0.0,100.0) #going toooooo fast
        rospy.sleep(1.0)
        gripper.command(1.3)
        rospy.sleep(0.8)
        whole_body.move_end_effector_pose(geometry.pose(z=-0.4), 'grasp_pose')
        rospy.sleep(0.8)
        whole_body.move_to_neutral()
        rospy.sleep(1.0)

        base.go_abs(preposition[0],preposition[1],preposition[2],500.0)

        rospy.sleep(1.0)
        whole_body.move_to_joint_positions({'head_pan_joint': pre_head_pan_joint, 'head_tilt_joint': pre_head_tilt_joint,'arm_roll_joint': -1.5})
        step1_finish = True
    rospy.sleep(10.0)

    if step1_finish:
        success = call_mechknownet_sys("drawer2","glassbox","drawer_demo",[-0.3,0.0,-0.1,0.1,0.0,0.05])
        if success:
            gripper.command(1.3)
            rospy.sleep(3.0)
            print "success, ready to go"
            whole_body.move_end_effector_pose(geometry.pose(y=-0.05,z=-0.04), 'grasp_pose')
            rospy.sleep(0.8)
            gripper.grasp(-0.5)
            rospy.sleep(0.8)
            whole_body.move_to_neutral()
            rospy.sleep(0.8)
            whole_body.move_end_effector_pose(geometry.pose(z=-0.03), 'execute1')
            rospy.sleep(0.8)
            whole_body.move_end_effector_pose(geometry.pose(z=-0.03), 'execute2')
            rospy.sleep(0.8)
            gripper.command(0.5)
            arm_lift_joint_position = whole_body.joint_positions['arm_lift_joint']
            rospy.sleep(0.8)
            whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_joint_position+0.2})
            rospy.sleep(0.8)
            base.go_abs(preposition[0],preposition[1],preposition[2],500.0)
            rospy.sleep(2.0)
            whole_body.move_to_neutral()
            rospy.sleep(1.0)
            gripper.grasp(-0.3)
            whole_body.move_end_effector_pose(geometry.pose(z=-0.4), 'prepose')
            rospy.sleep(1.0)
            whole_body.move_end_effector_pose(geometry.pose(z=-0.05), 'prepose')
            rospy.sleep(1.0)
            whole_body.move_to_neutral()
            rospy.sleep(1.0)
            base.go_abs(preposition[0],preposition[1],preposition[2],500.0)

def call_mechknownet_sys(tool,drawer,action,boundingbox):
    got_result_flag = False
    rospy.wait_for_service('mechknownetsys')
    try:
        mechknownet_sys_srv = rospy.ServiceProxy('mechknownetsys', mechknownetsys)
        response = mechknownet_sys_srv(tool,drawer,action,boundingbox)
        #response = mechknownet_sys_srv("drawer2","","debug",[])
        return response.Success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    try:
        demo()
    except rospy.ROSInterruptException:
        pass

    
'''    
whole_body.move_end_effector_pose(geometry.pose(z=-0.01), 'grasp_pose')
rospy.sleep(0.8)
gripper.grasp(-0.6)
rospy.sleep(0.8)
whole_body.move_to_joint_positions({'arm_lift_joint': 0.4})
rospy.sleep(0.8)
whole_body.move_to_neutral()
rospy.sleep(0.8)
whole_body.move_end_effector_pose(geometry.pose(y=0.08,ek=-0.8), 'execute1')
rospy.sleep(0.8)
whole_body.move_end_effector_pose(geometry.pose(y=0.08,ek=-0.8), 'execute2')
rospy.sleep(0.8)
gripper.command(1.0)
rospy.sleep(0.8)
whole_body.move_to_neutral()
'''
