#!/usr/bin/python
import roslib
roslib.load_manifest('mechknownet')
import rospy
import numpy as np
from plyfile import (PlyData, PlyElement, make2d, PlyParseError, PlyProperty)
import tensorflow as tf
import os
import sys
import h5py
import matplotlib.pyplot as plt

base_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(base_dir)
sys.path.append(base_dir)
sys.path.append(os.path.join(root_dir, 'model'))
sys.path.append(os.path.join(root_dir, 'pointnet_utils/show'))
import mechknownet_3states as model

import show3d_balls
import random
from mechknownet.srv import *
from visualization_msgs.msg import MarkerArray, Marker
#from itertools import permutations

class mechknownet_server:
    def __init__(self):
        self.drawer_demo = False
        self.num_points = 2400
        #tensorflow related
        self.sess = tf.Session()
        self.pointclouds_pl, clslabels_pl, reglabels_pl = model.placeholder_inputs(1,self.num_points)
        self.is_training_pl = tf.placeholder(tf.bool,shape=())
        self.clspred, self.regpred, end_points = model.get_model(self.pointclouds_pl, self.is_training_pl)
        #loss = model.get_loss(self.pred,clslabels_pl)
        saver = tf.train.Saver()

        if self.drawer_demo:
            saver.restore(self.sess, os.path.join(base_dir, "drawer_demo_log/log3/model.ckpt"))#retrain
        else:
            saver.restore(self.sess, os.path.join(base_dir, "log0224/model.ckpt"))
            #saver.restore(self.sess, os.path.join(base_dir, "log0223/model.ckpt"))

        #ROS related
        self.mechknownetsrv = rospy.Service('mechknownet_detect', mechknownetdetect, self.srvCb)
        self.grasp_marker_pub = rospy.Publisher("mechknownet_grasp_marker",MarkerArray, queue_size=1)
        self.manipulation_marker_pub = rospy.Publisher("mechknownet_manipulation_marker",MarkerArray, queue_size=1)
        self.function_text_marker_pub = rospy.Publisher("mechknownet_function_text_marker",MarkerArray, queue_size=1)
        rospy.loginfo("mechknownet detect: ready to serve")        
        self.first_obj_flag = False
        if self.drawer_demo:
            self.action_categories = ["","grasp","contain"]
        else:
            self.action_categories = ["","grasp","pour","pound","support"]


    def export_ply(self,pc, filename):
        vertex = np.zeros(pc.shape[0], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('red', 'u1'),('green', 'u1'),('blue', 'u1')])
        for i in range(pc.shape[0]):
            vertex[i] = (pc[i][0], pc[i][1], pc[i][2], pc[i][3],pc[i][4],pc[i][5])
        ply_out = PlyData([PlyElement.describe(vertex, 'vertex', comments=['vertices'])],text=True)
        ply_out.write(filename)

    def export_ply_xyz(self,pc, filename):
        vertex = np.zeros(pc.shape[0], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        for i in range(pc.shape[0]):
            vertex[i] = (pc[i][0], pc[i][1], pc[i][2])
        ply_out = PlyData([PlyElement.describe(vertex, 'vertex', comments=['vertices'])],text=True)
        ply_out.write(filename)

        
    def srvCb(self,req):

        rospy.loginfo("mechknownet: got request, num objs: %d", req.NumObjs )
        if req.NumObjs == 1:
            target_obj_arr = np.asarray(req.TargetObjectFlatPointCloud)
            target_obj_point_cloud = np.reshape(target_obj_arr, (-1,3))
            network_input_grasp = self.pointcloud_processor_grasp(target_obj_point_cloud)
            segpred_g, regpred_g = self.predict(np.expand_dims(network_input_grasp,0))
            grasp_result = np.c_[network_input_grasp, segpred_g[0]]
            recoverd_grasp_poses = self.recover_grasp_data(grasp_result, regpred_g[0])
            final_poses = np.r_[[recoverd_grasp_poses[0]], np.zeros((3,7))]
            resp = mechknownetdetectResponse()
            self.show_marker(final_poses,req.TrajID)

            #self.show_result(network_input_grasp,segpred_g[0],segpred_g[0])
            if(np.isnan(final_poses[0][0]) or np.isnan(final_poses[1][0])):
                print "nan detected"
                resp.Poses = []
                resp.PredictedPointLabel = []
                return resp
            else:                
                resp.Poses = final_poses.flatten().tolist()            
                resp.PredictedPointLabel = grasp_result.flatten().tolist()
                return resp

        if req.NumObjs == 2:
            target_obj_arr = np.asarray(req.TargetObjectFlatPointCloud)
            candidate_obj_arr = np.asarray(req.CandidateObjectFlatPointCloud)
            target_obj_point_cloud = np.reshape(target_obj_arr, (-1,3))
            candidate_obj_point_cloud = np.reshape(candidate_obj_arr, (-1,3))
            #for debug
            #cloud_pair = np.r_[target_obj_point_cloud,candidate_obj_point_cloud]

            network_input,seg_label = self.pointcloud_processor(target_obj_point_cloud, candidate_obj_point_cloud)        

            self.export_ply_xyz(network_input,"network_input.ply")
            print "saved ply file"
            segpred, regpred = self.predict(np.expand_dims(network_input,0))
            result = np.c_[network_input[:,:3], seg_label, segpred[0]] 
            #find the most frequent number in result
            
            nonzeros = np.where(result[:,4] > 0)
            label_no_background = result[:,4][nonzeros]
            function = None
            resp = mechknownetdetectResponse()

            if label_no_background.shape[0]>100: #has relation
                (labels,counts) = np.unique(label_no_background,return_counts=True)
                ind = np.argmax(counts)
                function = labels[ind]
                print "detected function", labels
                print "num of points", counts
            else:
                print "no relation"
                function = 0
                
            #self.show_result(network_input,segpred[0],seg_label)
            if self.drawer_demo:
                if function == 2:
                    recoverd_poses = self.recover_data(result, regpred[0], True)
                    network_input_grasp = self.pointcloud_processor_grasp(candidate_obj_point_cloud)
            else:
                if function == 0:
                    resp.Poses = []
                    resp.PredictedPointLabel = result.flatten().tolist()
                    return resp

                elif function == 3 or function == 2: 
                    recoverd_poses = self.recover_data(result, regpred[0], False)
                    network_input_grasp = self.pointcloud_processor_grasp(target_obj_point_cloud)
                else:
                    recoverd_poses = self.recover_data(result, regpred[0], True)
                    network_input_grasp = self.pointcloud_processor_grasp(candidate_obj_point_cloud)
                    
            segpred_g, regpred_g = self.predict(np.expand_dims(network_input_grasp,0))
            grasp_result = np.c_[network_input_grasp, segpred_g[0]]
            recoverd_grasp_poses = self.recover_grasp_data(grasp_result, regpred_g[0])
            final_poses = np.r_[[recoverd_grasp_poses[0]], recoverd_poses]       
            self.show_marker(final_poses,req.TrajID, action_category=self.action_categories[int(function)])

            print "????????", final_poses
            if(np.isnan(final_poses[0][0]) or np.isnan(final_poses[1][0])):
                print "nan detected"
                resp.Poses = []
                resp.PredictedPointLabel = []
                return resp

            else:
                resp.Poses = final_poses.flatten().tolist()
                resp.PredictedPointLabel = result.flatten().tolist()
            return resp


    def pointcloud_processor(self,obj1, obj2):

        #print "before ", obj1.shape
        #activate obj1 (add 1 to the last column)
        extended_obj1 = np.c_[obj1,np.ones(obj1.shape[0])]
        extended_obj2 = np.c_[obj2,np.zeros(obj2.shape[0])]

        #print "after ", extended_obj1.shape
        obj_pair_cloud = np.append(extended_obj1, extended_obj2, axis=0)        
        obj1_seg_label = np.ones(obj1.shape[0])
        obj2_seg_label = np.zeros(obj2.shape[0])
        seg_label = np.append(obj1_seg_label,obj2_seg_label, axis=0)
        '''
        if obj_pair_cloud.shape[0] > 8000:
            idx = random.sample(range(obj_pair_cloud.shape[0]), int(obj_pair_cloud.shape[0]*1/2))
            sampled_pc = obj_pair_cloud[idx,:]
            seg_label = seg_label[idx]
        else:
            sampled_pc = obj_pair_cloud
        '''
        sampled_pc = obj_pair_cloud
        if sampled_pc.shape[0] > self.num_points:
            idx2 = random.sample(range(sampled_pc.shape[0]), self.num_points)
            network_input = sampled_pc[idx2,:]
            seg_label = seg_label[idx2]        

        elif sampled_pc.shape[0] < self.num_points:
            num_padding = self.num_points - sampled_pc.shape[0]
            padding = np.zeros((num_padding, sampled_pc.shape[1]), dtype = sampled_pc.dtype)
            network_input = np.append(sampled_pc, padding, axis=0)
            seg_label = np.append(seg_label, np.zeros(num_padding), axis=0)
        return network_input, seg_label

    def pointcloud_processor_grasp(self,obj):

        extended_obj = np.c_[obj,np.ones(obj.shape[0])]

        if extended_obj.shape[0] > self.num_points:
            idx = random.sample(range(extended_obj.shape[0]), self.num_points)
            network_input = extended_obj[idx,:]

        elif extended_obj.shape[0] < self.num_points:
            num_padding = self.num_points - extended_obj.shape[0]
            padding = np.zeros((num_padding, extended_obj.shape[1]), dtype = extended_obj.dtype)
            network_input = np.append(extended_obj, padding, axis=0)
        return network_input

    def predict(self,points):
        feed_dict = {self.pointclouds_pl: points,
                     self.is_training_pl: False}
        logits, reglabel = self.sess.run([self.clspred, self.regpred],feed_dict=feed_dict)
        return np.argmax(logits,2), reglabel

    def show_result(self, pointcloud, label, gt_label=None):
        if gt_label is None:
            gt_label = np.zeros(self.num_points,dtype=int)
        
        else:
            gt_label = gt_label.astype(int)
        
        cmap = plt.cm.get_cmap("hsv", 5)
        cmap = np.array([cmap(i) for i in range(5)])[:,:3]        
        gt = cmap[gt_label, :]
        pred = cmap[label, :]
        ps = pointcloud[:,0:3]
        show3d_balls.showpoints(ps, gt, pred, ballradius=3)

    def recover_data(self, pc,label,support=False): 
        fake_color = np.zeros((pc.shape[0],1))
        recover_pc = np.c_[pc,fake_color]
        obj1 = recover_pc[(pc[:,3]==1).nonzero()]
        obj2 = recover_pc[(pc[:,3]==0).nonzero()]
        if not support:
            obj1[:,3] = 255
        else:
            obj2[:,3] = 255
        obj1_function_part = obj1[(obj1[:,4] > 0).nonzero()][:,0:3]
        obj2_function_part = obj2[(obj2[:,4] > 0).nonzero()][:,0:3]
        output_pc = np.r_[obj1,obj2]
        #self.export_ply(output_pc,"frame0000.ply")
        #print "save ply"
        #function_part = pc_void_to_array[:,:3]         
        centroid1 = np.mean(obj1_function_part, axis=0)
        centroid2 = np.mean(obj2_function_part, axis=0)

        if not support:
            label[:1,:3] =  label[:1,:3]+centroid1
            label[1:,:3] =  label[1:,:3]+centroid2
            rearrange_label = label[[0,2,1]]
            #rearrange_label[2][0] = rearrange_label[2][0] - 0.02 #for pour
            #rearrange_label[1][2] = rearrange_label[1][2] + 0.1 #for pound
        else:
            label[:2,:3] =  label[:2,:3]+centroid1
            label[2:,:3] =  label[2:,:3]+centroid2
            rearrange_label = label[[2,1,0]]

        np.savetxt("frame0000.txt",rearrange_label)
        print "save txt"
        return rearrange_label

    def recover_grasp_data(self, pc, label):
        fake_color = np.zeros((pc.shape[0],1))
        recover_pc = np.c_[pc,fake_color]
        function_part = recover_pc[(recover_pc[:,4]==1).nonzero()][:,:3]
        print "recover_pc ",recover_pc.shape
        recover_pc[recover_pc[:,4]==1,3]=255
        recover_pc[:,4]=0
        recover_pc[:,5]=0
        centroid = np.mean(function_part,axis=0)
        recover_label = label[:,:3] + centroid

        recover_label = np.c_[recover_label,label[:,3:]]
        self.export_ply(recover_pc,"frame0001.ply")
        np.savetxt("frame0001.txt",recover_label)
        return recover_label
    
    def show_marker(self,poses,TrajID, action_category=None):
        #grasp marker
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = "package://mechknownet/gripper.dae"
        marker.action = marker.ADD
        marker.id = TrajID*5 #grasp x 1 + traj X 3 + function X 1
        marker.pose.position.x = poses[0][0]
        marker.pose.position.y = poses[0][1]
        marker.pose.position.z = poses[0][2]
        marker.pose.orientation.w = poses[0][3]
        marker.pose.orientation.x = poses[0][4]
        marker.pose.orientation.y = poses[0][5]
        marker.pose.orientation.z = poses[0][6]
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)
        self.grasp_marker_pub.publish(marker_array)
        #manipulation marker
        marker_array = MarkerArray()
        for i in range(1,4):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.MESH_RESOURCE
            marker.mesh_resource = "package://mechknownet/arrow.dae"
            marker.action = marker.ADD
            marker.id = TrajID*5+i+1
            marker.pose.position.x = poses[i][0]
            marker.pose.position.y = poses[i][1]
            marker.pose.position.z = poses[i][2]
            marker.pose.orientation.w = poses[i][3]
            marker.pose.orientation.x = poses[i][4]
            marker.pose.orientation.y = poses[i][5]
            marker.pose.orientation.z = poses[i][6]
            marker.scale.x = 0.02
            marker.scale.y = 0.02 
            marker.scale.z = 0.02 
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.manipulation_marker_pub.publish(marker_array)
        # function text marker
        if action_category != None:
            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.TEXT_VIEW_FACING;
            marker.action = marker.ADD
            marker_text = "Detected Function : " + action_category
            marker.text = marker_text
            marker.id = TrajID*5+4
            marker.pose.position.x = poses[2][0]
            marker.pose.position.y = poses[2][1]
            marker.pose.position.z = poses[2][2] + 0.5
            marker.pose.orientation.w = poses[2][3]
            marker.pose.orientation.x = poses[2][4]
            marker.pose.orientation.y = poses[2][5]
            marker.pose.orientation.z = poses[2][6]
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            self.function_text_marker_pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node("mechknownet_server", anonymous=True)
    rs = mechknownet_server()
    rospy.spin()

