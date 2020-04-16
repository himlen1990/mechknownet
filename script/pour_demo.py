from hsrb_interface import Robot
from hsrb_interface import geometry
import rospy
import roslib
roslib.load_manifest('mechknownet')
import rospy
import numpy as np
from mechknownet.srv import *


robot = Robot()
whole_body = robot.try_get('whole_body')
base = robot.try_get('omni_base')
gripper = robot.try_get('gripper')
whole_body.looking_hand_constraint = True

def cup_demo():
    response = call_mechknownet_sys("bottle","cup","pour","three_tasks_demo",[])

    success = response.Success
    print "success?", success
    if success:        
        pass
        whole_body.move_end_effector_pose(geometry.pose(z=-0.08), 'grasp_pose')
        rospy.sleep(0.8)
        whole_body.move_end_effector_pose(geometry.pose(), 'grasp_pose')
        rospy.sleep(0.8)
        gripper.grasp(-0.3)
        rospy.sleep(0.8)
        whole_body.move_end_effector_pose(geometry.pose(), 'execute1')
        #rospy.sleep(0.8)
        whole_body.move_end_effector_pose(geometry.pose(x=0.01), 'execute2')
        rospy.sleep(0.8)
    else:
        if response.Report == "target action not found":
            print "target action not found"
            print "trying to turn over the cup"
            response = call_mechknownet_sys("cup","","grasp","three_tasks_demo",[])
            success = response.Success
            if success:
                pre_arm_lift_joint = whole_body.joint_positions['arm_lift_joint']
                pre_head_pan_joint = whole_body.joint_positions['head_pan_joint']
                pre_head_tilt_joint = whole_body.joint_positions['head_tilt_joint']
                preposition = base.pose
                gripper.command(1.2)
                rospy.sleep(1.0)
                whole_body.move_end_effector_pose(geometry.pose(z=-0.1), 'grasp_pose')
                rospy.sleep(1.0)
                whole_body.move_end_effector_pose(geometry.pose(), 'grasp_pose')
                rospy.sleep(1.0)
                gripper.grasp(-0.3)
                arm_lift_joint_position = whole_body.joint_positions['arm_lift_joint']
                whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_joint_position+0.1})
                rospy.sleep(1.0)
                whole_body.move_to_joint_positions({'wrist_roll_joint': 3.0})
                rospy.sleep(1.0)
                whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_joint_position+0.03})
                gripper.command(0.1)
                rospy.sleep(1.0)
                gripper.command(0.3)
                rospy.sleep(1.0)
                gripper.command(0.5)
                rospy.sleep(1.0)
                gripper.command(1.2)
                rospy.sleep(1.0)
                whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_joint_position+0.15})
                rospy.sleep(1.0)
                whole_body.move_to_joint_positions({'wrist_roll_joint': -0.1})
                rospy.sleep(1.0)
                base.go_abs(preposition[0],preposition[1],preposition[2],500.0)
                rospy.sleep(1.0)
                whole_body.move_to_joint_positions({'head_pan_joint': pre_head_pan_joint, 'head_tilt_joint': pre_head_tilt_joint,'arm_lift_joint': pre_arm_lift_joint})
                rospy.sleep(10.0)
                step_2_response = call_mechknownet_sys("bottle","cup","pour","three_tasks_demo",[])
                step_2_success = step_2_response.Success
                print "success?", step_2_success
                if step_2_success:        
                    rospy.sleep(5.0)
                    whole_body.move_end_effector_pose(geometry.pose(z=-0.08), 'grasp_pose')
                    rospy.sleep(0.8)
                    whole_body.move_end_effector_pose(geometry.pose(), 'grasp_pose')
                    rospy.sleep(0.8)
                    gripper.grasp(-0.3)
                    rospy.sleep(0.8)
                    whole_body.move_end_effector_pose(geometry.pose(), 'execute1')
                    #rospy.sleep(0.8)
                    whole_body.move_end_effector_pose(geometry.pose(x=0.01), 'execute2')
                    rospy.sleep(0.8)
                    print "end of task"                    



def call_mechknownet_sys(tool,drawer,action,demo, boundingbox):
    rospy.wait_for_service('mechknownetsys')
    try:
        mechknownet_sys_srv = rospy.ServiceProxy('mechknownetsys', mechknownetsys)
        response = mechknownet_sys_srv(tool,drawer,action,demo,boundingbox)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    try:
        cup_demo()
    except rospy.ROSInterruptException:
        pass
