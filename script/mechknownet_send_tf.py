#! /usr/bin/env python

import roslib
import tf
roslib.load_manifest('mechknownet')
import rospy
import numpy as np
from mechknownet.srv import *
from visualization_msgs.msg import MarkerArray

num_calls = 0 # one traj for one call
num_markers = 4
xyz = [None]*num_markers
rpy = [None]*num_markers
rospy.loginfo("mechknownet_send_tf: ready to serve")
got_result_flag = False
keep_pred_pose_flag = False

def srvCb(req):
    global xyz, rpy, got_result_flag, num_calls, num_markers, keep_pred_pose_flag
    print num_calls
    rospy.loginfo("mechknownet_send_tf: got message")
    num_calls = num_calls + 1
    keep_pred_pose_flag = req.KeepPrePose
    pose_arr = np.asarray(req.Poses)
    pose_arr = pose_arr.reshape([-1,7])   
    print "!!!!!!!!!!!!!!!",pose_arr.shape[0]
    if (pose_arr.shape[0] != num_markers):
        print "number of poses should == %d" %(num_markers)
        return False

    for i in range(num_markers):
        xyz[i] = [pose_arr[i][0],pose_arr[i][1],pose_arr[i][2]] #transition
        rpy[i] = tf.transformations.euler_from_quaternion([pose_arr[i][4],pose_arr[i][5],pose_arr[i][6],pose_arr[i][3]]) #rotation

    #xyz[0][2] = 0.07#for umbrella grasp

    
    if xyz[0] != None:
        got_result_flag = True
    return True

def send_tf():
    global xyz, rpy, got_result_flag, num_markers, num_calls, keep_pred_pose_flag
    got_result_flag = False
    rospy.init_node('mechknownet_sendtf')
    mechknownet_tfsrv = rospy.Service('mechknownet_sendtf', mechknownetsendtf, srvCb)
    rate = rospy.Rate(20)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    base_link_update_flag = False
    got_fix_frame_flag = False
    fix_to_map_frame = [None]*(num_markers)
    prepose = None
    prepose_ready_flag = False
    while not rospy.is_shutdown():
        if got_result_flag:
            try:
                (trans1,rot1) = listener.lookupTransform('/map','/base_link',rospy.Time(0))
                base_link_update_flag = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            if base_link_update_flag:
                trans1_mat = tf.transformations.translation_matrix(trans1)
                rot1_mat   = tf.transformations.quaternion_matrix(rot1)
                mat1 = np.dot(trans1_mat, rot1_mat)
                
                if keep_pred_pose_flag:
                    prepose = fix_to_map_frame[0]
                    print "!!!!!!!!!!!!!!!!!!!!!!!!!prepose ", prepose
                    prepose_ready_flag = True
                    if prepose == None:
                        rospy.loginfo("does not have prepose")                        

                for i in range(num_markers):
                    quat = tf.transformations.quaternion_from_euler(rpy[i][0],rpy[i][1],rpy[i][2])
                    trans = [xyz[i][0],xyz[i][1],xyz[i][2]]
                    trans2_mat = tf.transformations.translation_matrix(trans)
                    rot2_mat = tf.transformations.quaternion_matrix(quat)
                    mat2 = np.dot(trans2_mat, rot2_mat)
                    mat3 = np.dot(mat1, mat2)
                    trans3 = tf.transformations.translation_from_matrix(mat3)
                    rot3 = tf.transformations.quaternion_from_matrix(mat3)
                    pose = [trans3,rot3]
                    fix_to_map_frame[i] = pose

                got_fix_frame_flag = True
                got_result_flag = False
            
        if got_fix_frame_flag:
            br.sendTransform(fix_to_map_frame[0][0],
                             fix_to_map_frame[0][1],
                             rospy.Time.now(),
                             "grasp_pose",
                             "map")

            br.sendTransform(fix_to_map_frame[1][0],
                             fix_to_map_frame[1][1],
                             rospy.Time.now(),
                             "manipulation_pose0",
                             "map")

            for i in range(2, num_markers): 
                frame_name = "manipulation_pose%d" %(i-1)
                br.sendTransform(fix_to_map_frame[i][0],
                                 fix_to_map_frame[i][1],
                                 rospy.Time.now(),
                                 frame_name,
                                 "map")
            
            if keep_pred_pose_flag and prepose_ready_flag:
                frame_name = "prepose"
                br.sendTransform(prepose[0],
                                 prepose[1],
                                 rospy.Time.now(),
                                 frame_name,
                                 "map")
            try:
                (trans,rot) = listener.lookupTransform('/manipulation_pose0','/grasp_pose',rospy.Time(0))

                for i in range(1,3):

                    frame_name = "execute%d" %(i)
                    parent_frame_name = "manipulation_pose%d" %(i)
                    br.sendTransform(trans,
                                     rot,
                                     rospy.Time.now(),
                                     frame_name,
                                     parent_frame_name)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        rate.sleep()


if __name__ == '__main__':
    try:
        send_tf()
    except rospy.ROSInterruptException:
        pass

