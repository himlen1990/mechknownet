from hsrb_interface import Robot
from hsrb_interface import geometry
import rospy
robot = Robot()
whole_body = robot.try_get('whole_body')
gripper = robot.try_get('gripper')
whole_body.linear_weight = 30.0
whole_body.looking_hand_constraint = True

whole_body.move_end_effector_pose(geometry.pose(z=-0.08), 'grasp_pose')
rospy.sleep(0.8)
whole_body.move_end_effector_pose(geometry.pose(), 'grasp_pose')
rospy.sleep(0.8)
gripper.grasp(-0.3)
rospy.sleep(0.8)
whole_body.move_end_effector_pose(geometry.pose(), 'execute1')
#rospy.sleep(0.8)
whole_body.move_end_effector_pose(geometry.pose(), 'execute2')
rospy.sleep(0.8)
gripper.command(1.0)
