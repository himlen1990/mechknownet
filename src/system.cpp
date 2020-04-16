#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <mechknownet/mrcnn.h>
#include <mechknownet/lookaround.h>
#include <mechknownet/mechknownetdetect.h>
#include <mechknownet/mechknownetsendtf.h>
#include <mechknownet/mechknownetsys.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <cassert>
#include <time.h>

using namespace std;
using namespace cv;
typedef pcl::PointXYZ PCType;

class call_mechknownet
{
public:
  ros::NodeHandle nh_;
  ros::ServiceClient mrcnn_client;
  ros::ServiceClient lookaround_client;
  ros::ServiceClient mechknownet_client;
  ros::ServiceClient mechknownet_send_tf_client;
  mechknownet::mrcnn mrcnn_srv;
  mechknownet::lookaround lookaround_srv;
  mechknownet::mechknownetdetect mechknownet_detect_srv;
  mechknownet::mechknownetsendtf mechknownet_send_tf_srv;

  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image> >* sync_input_2_; 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  sensor_msgs::Image RGBImg_;
  sensor_msgs::Image DepthImg_;
  sensor_msgs::Image SynDepth_;
  sensor_msgs::Image SynRGB_;
  bool cam_info_ready;
  bool image_ready;
  boost::array<double,9> camera_info_;
  float cx;
  float cy;
  float invfx;
  float invfy;
  tf::TransformListener listener;

  ros::Subscriber camera_info_sub;
  ros::Publisher pointcloud_pub;
  ros::Publisher traj_pointcloud_pub;
  ros::ServiceServer systemService;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub;
  
  string tf_reference_frame_;
  //string objects[9] = {"","umbrella","chair1","chair2","drawer1","drawer2","banana","cup","glassbox"};
  //string actions[4] = {"no action","grasp","drawer_demo"};
  string objects[9] = {"","cup","bottle","bowl","plate","apple","eggplant","hammer","kettle"};
  string actions[5] = {"no action","grasp","pour","pound","support"};
  bool found_manipulate_obj_flag;
  bool found_associate_obj_flag;
  //bool found_relation_flag;
  bool search_finish_flag;
  bool task_finish_flag;
  bool visualize_flag;
  int look_around_counter;

  float look_around_tilt_angle;
  float look_around_pan_angle;
  bool look_around_ready;
  bool mechknownet_detect_ready;

  pcl::PointCloud<PCType>::Ptr manipulate_object_cloud;
  pcl::PointCloud<PCType>::Ptr associate_object_cloud;
  std::vector<pcl::PointCloud<PCType>::Ptr> found_object_clouds;
  std::vector<pcl::PointCloud<PCType>::Ptr> candidate_manipulate_object_clouds;
  std::vector<float> manipulate_obj_flat_pointCloud;
  string manipulate_object;
  string associate_object;
  string target_action;
  string which_demo;

  int numTargets; // one object scene(grasp) or two objects scene
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_show_final;

  call_mechknownet(int argc, char** argv): it_(nh_){

    rgb_sub.subscribe(nh_, "/hsrb/head_rgbd_sensor/rgb/image_rect_color", 1);
    depth_sub.subscribe(nh_, "/hsrb/head_rgbd_sensor/depth_registered/image", 1);
	

    sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,sensor_msgs::Image> >(1);//queue size = 1
    sync_input_2_->connectInput(rgb_sub,depth_sub);
    sync_input_2_->registerCallback(boost::bind(&call_mechknownet::callback, this, _1, _2));



	camera_info_sub = nh_.subscribe("/hsrb/head_rgbd_sensor/rgb/camera_info",1, &call_mechknownet::camera_info_cb,this);

    mrcnn_client = nh_.serviceClient<mechknownet::mrcnn>("mechknownet_mrcnn");
	lookaround_client = nh_.serviceClient<mechknownet::lookaround>("lookaround");
	mechknownet_client = nh_.serviceClient<mechknownet::mechknownetdetect>("mechknownet_detect");

	mechknownet_send_tf_client = nh_.serviceClient<mechknownet::mechknownetsendtf>("mechknownet_sendtf");

    pointcloud_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("mechknownettestoutput", 1);
    traj_pointcloud_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("mechknownettrajoutput", 1);

	image_pub = it_.advertise("/mechknownet/image_mask", 1);

	systemService = nh_.advertiseService("mechknownetsys", &call_mechknownet::srvCb, this);

	tf_reference_frame_ = "/base_link";// map or base_link

	found_manipulate_obj_flag = false;
	found_associate_obj_flag = false;
	//found_relation_flag = false;
	task_finish_flag = false;
	search_finish_flag = false;
	cam_info_ready = false;
	image_ready = false;
	mechknownet_detect_ready = false;
	look_around_ready = false;
	look_around_counter = 0;
	look_around_tilt_angle = 0;
	look_around_pan_angle = 0;
	visualize_flag = true;	
	numTargets = 2;
	ROS_INFO("mechknownetSys: Ready to serve");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_show (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_show_final = cloud_show;
  }

  void system_flag_recover()
  {
	found_manipulate_obj_flag = false;
	found_associate_obj_flag = false;
	//found_relation_flag = false;
	task_finish_flag = false;
	search_finish_flag = false;
	image_ready = false;
	mechknownet_detect_ready = false;
	look_around_ready = false;
	look_around_counter = 0;
	look_around_tilt_angle = 0;
	look_around_pan_angle = 0;
	//rememeber to clear vectors
  }

  bool srvCb(mechknownet::mechknownetsys::Request& req, mechknownet::mechknownetsys::Response& res)
  {
	task_finish_flag = false;
	ROS_INFO("mechknownetSys: got request");
	manipulate_object = req.ManipulateObj;
	associate_object = req.AssociateObj;
	target_action = req.Action;
	which_demo = req.WhichDemo;
	ROS_INFO("mechknownetSys: manipulate object : %s, associate object: %s, target action: %s", manipulate_object.c_str(), associate_object.c_str(), target_action.c_str());
	//check if target tool and object inside dataset
	bool manObj_in_dataset;
	bool assObj_in_dataset;
	int obj_in_dataset = sizeof(objects)/sizeof(*objects);
	string report = " ";
	for(int i=0; i< obj_in_dataset; i++)
	  {
		if(objects[i] == manipulate_object)
		  manObj_in_dataset = true;
		if(objects[i] == associate_object)
		  assObj_in_dataset = true;
	  }
	if(!manObj_in_dataset || !assObj_in_dataset)
	  {
		ROS_INFO("mechknownetSys: dataset has no tool or object information, please check the code");
		res.Success = false;	
	  }

	//if(associate_object=="")
	//numTargets = 1;

	if(target_action == "")
	  {
		ROS_INFO("mechknownetSys: target action can not be empty");
		res.Success = false;	
	  }
	while(!image_ready || !cam_info_ready)
	  {
		ros::Duration(1.0).sleep();
		ros::spinOnce();
		ROS_INFO("waiting for image and camera information ready");
	  }
	

	if(target_action == "grasp") // one step task, output proposal trajectories
	  {
		numTargets = 1;
		detect_grasp_pose();
	  }

	else if(which_demo == "drawer_demo") 
	  {
		ros::Rate r(10);	
		int image_update_counter=0;  		
		image_ready = false;
		while(!image_ready || image_update_counter<20)
		  {
			ros::spinOnce();
			r.sleep();
			cout<<"waiting for update"<<endl;
			image_update_counter++;
		  }
		std::vector<float> boundingbox = req.BoundingBox;
		update_target_pointcloud(boundingbox);
	  }
	else if(which_demo == "three_tasks_demo") // pour pound put
	  {
		numTargets = 2;
		report = three_tasks_demo();
		cout<<report<<endl;
	  }
	else 
	  {
		numTargets = 2;
		search_target();
	  }



	if(task_finish_flag)
	  {
		res.Success = true;	
		res.Report = report;
		//system_flag_recover();
	  }
	else
	  {
		res.Success = false;
		res.Report = report;
	  }


	return true; //return true means no error occurred in this program
  }

  void camera_info_cb(const sensor_msgs::CameraInfoPtr& camInfo)
  {

    camera_info_ = camInfo->K;
	cx = camera_info_[2];
	cy = camera_info_[5];
	invfx = 1.0f/camera_info_[0];
	invfy = 1.0f/camera_info_[4];
	camera_info_sub.shutdown();
    cam_info_ready = true;
  }

  void callback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
  {

    RGBImg_ = *msgRGB;
	DepthImg_ = *msgD;
    image_ready = true;

  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_processor(pcl::PointCloud <PCType>::Ptr &cloud, bool remove_table_top = true)
  {
	std::vector <pcl::PointIndices> clusters;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.06); // 4cm
	ec.setMinClusterSize (300);
	//ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (clusters);
	cout<<"cloud size "<<cloud->points.size()<<endl;
	cout<<"clusters size "<<clusters.size()<<endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	int largest_piece_index = 0;
	int cluster_size = clusters[0].indices.size();
	if(clusters.size()>1) //retrun the largest piece
	  {
		for (int i=0; i<clusters.size(); i++)
		  {
			if(clusters[i].indices.size() > cluster_size)
			  largest_piece_index = i;
		  }
	  }
	for (std::vector<int>::const_iterator pit = clusters[largest_piece_index].indices.begin (); pit != clusters[largest_piece_index].indices.end (); ++pit)
	  cloud_cluster->points.push_back (cloud->points[*pit]); //*
	cloud_cluster->width = cloud_cluster->points.size ();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;

	return cloud_cluster;
  }

  void detect_grasp_pose()
  {
    mrcnn_srv.request.Image = RGBImg_;
	SynDepth_ = DepthImg_;
	SynRGB_ = RGBImg_;
	ROS_INFO("waiting for mask rcnn service");
    if(mrcnn_client.call(mrcnn_srv))
      {
		std::vector<int32_t> cls_ids = mrcnn_srv.response.Class_ids;
		std::vector<int32_t> scores = mrcnn_srv.response.Scores;
		std::vector<sensor_msgs::Image> masks = mrcnn_srv.response.Masks;
		std::vector<std::vector<float> >candidate_trajectories;
		if(cls_ids.size()>0)
		  {
			show_mask_image(masks,SynRGB_);
			cout<<"I found "<<cls_ids.size()<<" objects"<<endl;
			for(int i=0; i<cls_ids.size(); i++) 
			  {
				cout<<"I found "<<objects[cls_ids[i]]<<endl;
				if(objects[cls_ids[i]] == manipulate_object)
				  {

					ROS_INFO("found manipualted object");
					found_manipulate_obj_flag = true;
					pcl::PointCloud<PCType>::Ptr temp_manipulate_object_cloud = mask_to_pointcloud(masks[i],SynDepth_);
					pcl::PointCloud<PCType>::Ptr empty_cloud;
					int TrajID = i;
					std::vector<float> trajectory;
					std::vector<float> predict_point_label;
					bool action_found = call_mechknownet_srv(numTargets, TrajID, target_action, temp_manipulate_object_cloud, empty_cloud, trajectory, predict_point_label);					
#if 0
					//for data collectiong
					bool action_found = false;
					pcl::PLYWriter ply_saver; 
					ply_saver.write("frame000.ply",*temp_manipulate_object_cloud);    
					// end of data collectiong
#endif
					if(action_found)
					  {
						candidate_trajectories.push_back(trajectory);
						candidate_manipulate_object_clouds.push_back(temp_manipulate_object_cloud);
					  }
				  }
			  }			

			if(!found_manipulate_obj_flag)
			  {
				ROS_INFO("I can not found the manipulable object, finish");
				return;
			  }			

			if(candidate_trajectories.size()>0) // valid trajectory detected, choose the first one and send tf;
			  {
				ROS_INFO("call mechknownet send tf server");
				if(candidate_trajectories.size()>1)
				  {
					srand (time(NULL));
					int random_traj = rand() % candidate_trajectories.size();
					mechknownet_send_tf_srv.request.Poses = candidate_trajectories[random_traj];
					mechknownet_send_tf_srv.request.KeepPrePose = false;
					manipulate_object_cloud = candidate_manipulate_object_clouds[random_traj];
				  }
				else
				  {
					mechknownet_send_tf_srv.request.Poses = candidate_trajectories[0];
					mechknownet_send_tf_srv.request.KeepPrePose = false;
					manipulate_object_cloud = candidate_manipulate_object_clouds[0];
				  }
				pcl::PLYWriter ply_saver; 
				ply_saver.write("manipulate_cloud.ply",*manipulate_object_cloud);    
	
				if(mechknownet_send_tf_client.call(mechknownet_send_tf_srv))
				  {
					task_finish_flag = true;
					return;
				  }
				else
				  ROS_INFO("Fail to call mechknownet send tf service");				
			  }
		  } // end of if(cls_ids.size()>0)
		else
		  {
			//look around or directly end the program
			cout<<"no target found by mask rcnn, finish"<<endl;
			return;
		  }
      } // end of if(mrcnn_client.call(mrcnn_srv))
	else
	  ROS_INFO("Fail to call mask rcnn service");
  } //end of function
  
  string three_tasks_demo()
  {
    mrcnn_srv.request.Image = RGBImg_;
	SynDepth_ = DepthImg_;
	SynRGB_ = RGBImg_;
	std::vector<pcl::PointCloud<PCType>::Ptr > manipulate_object_candidates;
	std::vector<pcl::PointCloud<PCType>::Ptr > associate_object_candidates;
	std::vector<std::vector<float> >candidate_trajectories;
	//first call
    if(mrcnn_client.call(mrcnn_srv))
      {
		std::vector<int32_t> cls_ids = mrcnn_srv.response.Class_ids;
		std::vector<int32_t> scores = mrcnn_srv.response.Scores;
		std::vector<sensor_msgs::Image> masks = mrcnn_srv.response.Masks;
		if(cls_ids.size()>0)
		  {
			show_mask_image(masks,SynRGB_);
			cout<<"I found "<<cls_ids.size()<<" objects"<<endl;
			for(int i=0; i<cls_ids.size(); i++) 
			  {
				cout<<"I found "<<objects[cls_ids[i]]<<endl;
				if(objects[cls_ids[i]] == manipulate_object)
				  {

					ROS_INFO("found manipualted object");
					found_manipulate_obj_flag = true;
					pcl::PointCloud<PCType>::Ptr detected_manipulate_object_cloud = mask_to_pointcloud(masks[i],SynDepth_,true);
					manipulate_object_candidates.push_back(detected_manipulate_object_cloud);
				  }
				if(objects[cls_ids[i]] == associate_object)
				  {

					ROS_INFO("found associate_object");
					found_associate_obj_flag = true;
					pcl::PointCloud<PCType>::Ptr detected_associate_object_cloud = mask_to_pointcloud(masks[i],SynDepth_,true);
					associate_object_candidates.push_back(detected_associate_object_cloud);
				  }
			  }			
			if(found_manipulate_obj_flag && found_associate_obj_flag)
			  {
				ROS_INFO(" manipulable object and associate object found, call mechknownet");
				bool action_found = false;
				for(int i=0; i<manipulate_object_candidates.size();i++)
				  {
					for(int j=0; j<associate_object_candidates.size();j++)
					  {
						int TrajID = i*associate_object_candidates.size() + j;
						numTargets = 2;
						std::vector<float> trajectory;
						std::vector<float> predict_point_label;
						action_found = call_mechknownet_srv(numTargets, TrajID, target_action, manipulate_object_candidates[i], associate_object_candidates[j], trajectory, predict_point_label);
						if(action_found)
						  {
							ROS_INFO(" found target action");

							candidate_trajectories.push_back(trajectory);
							//record object pair for display
							//one traj for testing
							trajectory_visualize(trajectory,manipulate_object_candidates[i], associate_object_candidates[j],target_action);
						  }
					  }
				  }
				if(!action_found)
					return "target action not found";

				if(candidate_trajectories.size()>0) 
				  {
					ROS_INFO("call mechknownet send tf server");
					if(candidate_trajectories.size()>1)
					  {
						srand (time(NULL));
						int random_traj = 0;//rand() % candidate_trajectories.size();
						mechknownet_send_tf_srv.request.Poses = candidate_trajectories[random_traj];
						mechknownet_send_tf_srv.request.KeepPrePose = false;
						trajectory_visualize(candidate_trajectories[0],manipulate_object_candidates[0], associate_object_candidates[0],target_action);
					  }
					else
					  {
						mechknownet_send_tf_srv.request.Poses = candidate_trajectories[0];
						mechknownet_send_tf_srv.request.KeepPrePose = false;
					  }					  
					if(mechknownet_send_tf_client.call(mechknownet_send_tf_srv))
					  {
						ROS_INFO("Success to call mechknownet send tf service");
						task_finish_flag = true;
						return "success";
					  }
					else
					  {
						ROS_INFO("Fail to call mechknownet send tf service");
						return "Fail to call mechknownet send tf service";
					  }
				  }
			  }// end of if(found_manipulate_obj_flag && found_associate_obj_flag)
			else
			  {
				ROS_INFO("I can not found the manipulable object or associate object, finish");
				return "target object not found";
			  }
		  } // end of if(cls_ids.size()>0)
		else
		  {
			//look around or directly end the program
			cout<<"no target found by mask rcnn, finish"<<endl;
			return "no object found";
		  }
      } // end of if(mrcnn_client.call(mrcnn_srv))
	else
	  {
		ROS_INFO("Fail to call mask rcnn service");
		return "Fail to call mask rcnn service";
	  }
  }

  void search_target()
  {
    mrcnn_srv.request.Image = RGBImg_;
	SynDepth_ = DepthImg_;
	SynRGB_ = RGBImg_;
	std::vector<pcl::PointCloud<PCType>::Ptr > manipulate_object_candidates;
	std::vector<pcl::PointCloud<PCType>::Ptr > associate_object_candidates;
	std::vector<std::vector<float> >candidate_trajectories;
	//first call
    if(mrcnn_client.call(mrcnn_srv))
      {
		std::vector<int32_t> cls_ids = mrcnn_srv.response.Class_ids;
		std::vector<int32_t> scores = mrcnn_srv.response.Scores;
		std::vector<sensor_msgs::Image> masks = mrcnn_srv.response.Masks;
		if(cls_ids.size()>0)
		  {
			show_mask_image(masks,SynRGB_);
			cout<<"I found "<<cls_ids.size()<<" objects"<<endl;
			for(int i=0; i<cls_ids.size(); i++) 
			  {
				cout<<"I found "<<objects[cls_ids[i]]<<endl;
				if(objects[cls_ids[i]] == manipulate_object)
				  {

					ROS_INFO("found manipualted object");
					found_manipulate_obj_flag = true;
					pcl::PointCloud<PCType>::Ptr detected_manipulate_object_cloud = mask_to_pointcloud(masks[i],SynDepth_,true);
					manipulate_object_candidates.push_back(detected_manipulate_object_cloud);
				  }
				if(objects[cls_ids[i]] == associate_object)
				  {

					ROS_INFO("found associate_object");
					found_associate_obj_flag = true;
					pcl::PointCloud<PCType>::Ptr detected_associate_object_cloud = mask_to_pointcloud(masks[i],SynDepth_,true);
					associate_object_candidates.push_back(detected_associate_object_cloud);
				  }
			  }			
			if(!found_manipulate_obj_flag)
			  {
				ROS_INFO("I can not found the manipulable object, finish");
				return;
			  }			
			if(found_manipulate_obj_flag && found_associate_obj_flag)
			  {
				ROS_INFO(" manipulable object and associate object found, call mechknownet");
				for(int i=0; i<manipulate_object_candidates.size();i++)
				  {
					for(int j=0; j<associate_object_candidates.size();j++)
					  {
						int TrajID = i*associate_object_candidates.size() + j;
						numTargets = 2;
						std::vector<float> trajectory;
						std::vector<float> predict_point_label;
						bool action_found = call_mechknownet_srv(numTargets, TrajID, target_action, manipulate_object_candidates[i], associate_object_candidates[j], trajectory, predict_point_label);
						if(action_found)
						  {
							ROS_INFO(" found target action");

							candidate_trajectories.push_back(trajectory);
							//record object pair for display
							//one traj for testing
							trajectory_visualize(trajectory,manipulate_object_candidates[i], associate_object_candidates[j],target_action);
						  }
					  }
				  }

				if(candidate_trajectories.size()>0) 
				  {
					ROS_INFO("call mechknownet send tf server");
					if(candidate_trajectories.size()>1)
					  {
						srand (time(NULL));
						int random_traj = 0;//rand() % candidate_trajectories.size();
						mechknownet_send_tf_srv.request.Poses = candidate_trajectories[random_traj];
						mechknownet_send_tf_srv.request.KeepPrePose = false;
						trajectory_visualize(candidate_trajectories[0],manipulate_object_candidates[0], associate_object_candidates[0],target_action);
					  }
					else
					  {
						mechknownet_send_tf_srv.request.Poses = candidate_trajectories[0];
						mechknownet_send_tf_srv.request.KeepPrePose = false;
					  }					  
					if(mechknownet_send_tf_client.call(mechknownet_send_tf_srv))
					  {
						ROS_INFO("Success to call mechknownet send tf service");
						return;
					  }
					else
					  ROS_INFO("Fail to call mechknownet send tf service");
				  }
			  }
			if(found_manipulate_obj_flag && !found_associate_obj_flag) //look around to search target
			  {
				ros::Rate r(10);	
				int image_update_counter;  // the image will delay sometimes, so we need a counter;		
				mechknownet_detect_ready = true;
				while(!search_finish_flag)
				  {
					if(mechknownet_detect_ready)
					  {
						look_around();
						image_ready = false;
						image_update_counter = 0;
					  }
					ros::spinOnce();
					r.sleep();
					image_update_counter++;
					if(look_around_ready && image_ready && image_update_counter > 10)
					  {
						find_candidates_while_look_around(manipulate_object_candidates);// find candidate tools
					  }
				  }				
			  }
		  } // end of if(cls_ids.size()>0)
		else
		  {
			//look around or directly end the program
			cout<<"no target found by mask rcnn, finish"<<endl;
			return;
		  }
      } // end of if(mrcnn_client.call(mrcnn_srv))
	else
	  ROS_INFO("Fail to call mask rcnn service");
  } //end of function

  void look_around()	
  {
	mechknownet_detect_ready = false;	
	look_around_pan_angle = look_around_pan_angle - 0.5;	
	lookaround_srv.request.TiltAngle = look_around_tilt_angle;
	lookaround_srv.request.PanAngle = look_around_pan_angle;
	cout<<"look around"<<look_around_counter<<endl;	  

	if(look_around_counter < 2)//4
	  {
		look_around_pan_angle = -0.5*look_around_counter;
		lookaround_srv.request.TiltAngle = look_around_tilt_angle;
		lookaround_srv.request.PanAngle = look_around_pan_angle;
		if(lookaround_client.call(lookaround_srv))
		  {
			cout<<"looking r"<<endl;
		  }
		else
		  ROS_INFO("Fail to call hsr look around service");
	  }
	if(look_around_counter >= 2)//4
	  {
		look_around_pan_angle = 0.5*(look_around_counter-1);
		lookaround_srv.request.TiltAngle = look_around_tilt_angle;
		lookaround_srv.request.PanAngle = look_around_pan_angle;
		if(lookaround_client.call(lookaround_srv))
		  {
			cout<<"looking l"<<endl;
		  }
	  }
	look_around_ready = true;
  }
  
  void find_candidates_while_look_around(std::vector<pcl::PointCloud<PCType>::Ptr > manipulate_object_candidates)
  {
	look_around_ready = false;
	ROS_INFO("find candidates while looking around");
#if 1
	//for debug
	cv_bridge::CvImageConstPtr cv_ptrRGB;
	try
	  {
		cv_ptrRGB = cv_bridge::toCvCopy(RGBImg_, sensor_msgs::image_encodings::BGR8);
	  }
	catch (cv_bridge::Exception& e)
	  {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	  }
	
	cv::Mat RGBImage;
	cv_ptrRGB->image.copyTo(RGBImage);	
	imshow("debug_img",RGBImage);
	waitKey(0);
	//end of debug
#endif  
	ROS_INFO("call mrcnn_srv while looking around");
    mrcnn_srv.request.Image = RGBImg_;
	SynDepth_ = DepthImg_;
	SynRGB_ = RGBImg_;
    if(mrcnn_client.call(mrcnn_srv))
      {
		std::vector<int32_t> cls_ids = mrcnn_srv.response.Class_ids;
		std::vector<int32_t> scores = mrcnn_srv.response.Scores;
		std::vector<sensor_msgs::Image> masks = mrcnn_srv.response.Masks;
		if(cls_ids.size()>0)
		  {
			for(int i=0; i<cls_ids.size(); i++) 
			  {
				if(objects[cls_ids[i]] == associate_object)
				  {

					found_associate_obj_flag = true;
					associate_object_cloud = mask_to_pointcloud(masks[i],SynDepth_);
					cout<<"[while looking around] I found the associate object "<< objects[cls_ids[i]]<<endl;
					break;
				  }
				else
				  {
					cout<<"[while looking around] I found candidate object "<< objects[cls_ids[i]]<<endl;					
					//store its point cloud for later process
					pcl::PointCloud<PCType>::Ptr found_object_cloud = mask_to_pointcloud(masks[i],  SynDepth_);
					found_object_clouds.push_back(found_object_cloud);
					//std::vector<float> CandidateFlatPointCloud = pointCloud_to_FlatPointCloud(candidate_cloud);
				  }
			  }
		  }
		else
		  ROS_INFO("[while looking around] I found nothing this time, keep founding");
	  }
	else
	  ROS_INFO("failed to call mrcnn while looking around");

	mechknownet_detect_ready = true;
	look_around_counter++;

	if(look_around_counter == 4)//6 left twice and right twice
	  {
		search_finish_flag = true;
		lookaround_srv.request.TiltAngle = 0.0;
		lookaround_srv.request.PanAngle = 0.0;
		if(lookaround_client.call(lookaround_srv))
		  {
			cout<<"search finish, move to neutral state"<<endl;
		  }		

		sensor_msgs::PointCloud2 pc2;
		pcl::PCLPointCloud2::Ptr pcl_pc_2(new pcl::PCLPointCloud2());
		pcl::PointCloud<PCType>::Ptr combined_cloud (new pcl::PointCloud<PCType>) ;
		pcl::copyPointCloud(*manipulate_object_cloud, *combined_cloud);
		for(int i=0; i<found_object_clouds.size(); i++)
		  *combined_cloud += *(found_object_clouds[i]);
		pcl::toPCLPointCloud2 (*combined_cloud, *pcl_pc_2);
		pcl_conversions::fromPCL( *pcl_pc_2, pc2 );
		pc2.header =  DepthImg_.header;
		pc2.header.frame_id = tf_reference_frame_;
		pointcloud_pub.publish(pc2);

	  }		
  }

	  
  std::vector<float> pointCloud_to_FlatPointCloud(pcl::PointCloud<PCType>::Ptr pointCloud)
  {
	std::vector<float> FlatPointCloud;
	for (int i=0; i<pointCloud->points.size(); i++)
	  {
		FlatPointCloud.push_back(pointCloud->points[i].x);
		FlatPointCloud.push_back(pointCloud->points[i].y);
		FlatPointCloud.push_back(pointCloud->points[i].z);
	  }
	return FlatPointCloud;
  }

  bool call_mechknownet_srv(int numObjs, int trajID, string target_action, pcl::PointCloud<PCType>::Ptr target_object_cloud, pcl::PointCloud<PCType>::Ptr candidate_object_cloud, std::vector<float> &trajectory, std::vector<float> &predicted_label)
  {
	std::vector<float> TargetFlatPointCloud = pointCloud_to_FlatPointCloud(target_object_cloud);

	mechknownet_detect_srv.request.TargetObjectFlatPointCloud = TargetFlatPointCloud;

	if (numObjs==1)
	  {
		std::vector<float> EmptyPointCloud;
		mechknownet_detect_srv.request.CandidateObjectFlatPointCloud = EmptyPointCloud;
	  }
	else
	  {
		std::vector<float> CandidateFlatPointCloud = pointCloud_to_FlatPointCloud(candidate_object_cloud);	
		mechknownet_detect_srv.request.CandidateObjectFlatPointCloud = CandidateFlatPointCloud;
	  }
	mechknownet_detect_srv.request.NumObjs = numObjs;
	mechknownet_detect_srv.request.TrajID = trajID;


	ROS_INFO("call mechknownet server");
	if(mechknownet_client.call(mechknownet_detect_srv))
	  {
		std::vector<float> point_label = mechknownet_detect_srv.response.PredictedPointLabel;
		//predicted_label = point_label;
		ROS_INFO("call mechknownet success");
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_show (new pcl::PointCloud<pcl::PointXYZRGB>);
		int actions_in_dataset = sizeof(actions)/sizeof(*actions);
		std::vector<int> label_counter(actions_in_dataset, 0); //background not included

		for(int i=0; i<point_label.size(); i=i+5)
		  {
			pcl::PointXYZRGB point;			
			point.x = point_label[i];
			point.y = point_label[i+1];
			point.z = point_label[i+2];			
			int Label = point_label[i+4];
			if(Label==1.0)
			  {
				  point.r = 255;
				  point.g = 0;
				  point.b = 0;				  
			  }
			else if(Label==2.0)
			  {
				point.r = 0;
				point.g = 255;
				point.b = 0;				  
			  }			
			else if(Label==3.0)
			  {
				point.r = 0;
				point.g = 255;
				point.b = 0;				  
			  }			
			else if(Label==4.0)
			  {
				point.r = 0;
				point.g = 255;
				point.b = 0;				  
			  }			
			else
			  {
				point.r = 255;
				point.g = 255;
				point.b = 255;				  				
			  }
			cloud_show->points.push_back(point);
			label_counter[Label]++;
		  }
		*cloud_show_final += *cloud_show;
		publish_pointcloud(cloud_show_final);
		// for debug
		pcl::PLYWriter ply_saver;
		ply_saver.write("abc.ply",*cloud_show);
		cout<<"background points: "<<label_counter[0]<<endl;
		int largest_number = 100; //at least 100 points
		int largest_number_index = 0;
		for(int i=1; i<actions_in_dataset; i++)
		  {
			cout<<"points in "<<actions[i]<<" : "<<label_counter[i]<<endl;
			if(label_counter[i] > largest_number)
			  largest_number_index = i;  
		  }

		if(actions[largest_number_index] == target_action)
		  {
			ROS_INFO("Found corresponding action: %s", target_action.c_str());
			trajectory = mechknownet_detect_srv.response.Poses;
			return true;
		  }
		else
		  {
			ROS_INFO("the detected action is not the desired action");			
			return false; // or we can return true and a proposal action
		  }
	  }
	else
	  ROS_INFO("Fail to call mechknownet detect service");
	return false;
  }


  void publish_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool is_trajectory = false)
  {
	sensor_msgs::PointCloud2 pc2;
	pcl::PCLPointCloud2::Ptr pcl_pc_2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2 (*cloud, *pcl_pc_2);
	pcl_conversions::fromPCL( *pcl_pc_2, pc2 );
	pc2.header =  DepthImg_.header;
	pc2.header.frame_id = tf_reference_frame_;
	if(is_trajectory)
	  traj_pointcloud_pub.publish(pc2);		
	else
	  pointcloud_pub.publish(pc2);	
  }

  void trajectory_visualize(std::vector<float> trajectory, pcl::PointCloud<PCType>::Ptr manipulate_object_cloud, pcl::PointCloud<PCType>::Ptr associate_object_cloud, string target_action)
  {
	std::vector <Eigen::Matrix4f> frames;
	for(int i=7; i<trajectory.size(); i=i+7) // the first 7th pose is grasp pose, we do not need that
	  {
		Eigen::Quaternionf q;
		q.w() = trajectory[i+3];
		q.x() = trajectory[i+4];
		q.y() = trajectory[i+5];
		q.z() = trajectory[i+6];
		Eigen::Matrix3f RM = q.normalized().toRotationMatrix();
		Eigen::Matrix4f frame = Eigen::Matrix4f::Identity();
		frame.block<3,3>(0,0) = RM;
		frame(0,3) = trajectory[i];
		frame(1,3) = trajectory[i+1];
		frame(2,3) = trajectory[i+2];
		frames.push_back(frame);
	  }
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obj1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obj2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr traj_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if(target_action=="drawer_demo" || target_action=="support")// || target_action=="pour")
	  {
		pcl::copyPointCloud(*associate_object_cloud, *cloud_obj1);		
		pcl::copyPointCloud(*manipulate_object_cloud, *cloud_obj2);
	  }
	else
	  {
		pcl::copyPointCloud(*manipulate_object_cloud, *cloud_obj1);
		pcl::copyPointCloud(*associate_object_cloud, *cloud_obj2);		
	  }
	for(int i=1; i<frames.size(); i++)
	  {
		Eigen::Matrix4f trans =  frames[i]*frames[0].inverse();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud (*cloud_obj1, *cloud_trans, trans);
		//add color
		for(int i=0; i<cloud_trans->points.size(); i++)
		  {
			cloud_trans->points[i].r = 255;
			cloud_trans->points[i].g = 80;
			cloud_trans->points[i].b = 80;
		  }
		*traj_cloud += *cloud_trans;
	  }	  
	publish_pointcloud(traj_cloud,true);
  }

  void show_mask_image(std::vector<sensor_msgs::Image> masks, sensor_msgs::Image syn_RGB)
  {


	cv_bridge::CvImageConstPtr cv_ptrRGB;
	try
	  {
		cv_ptrRGB = cv_bridge::toCvCopy(syn_RGB, sensor_msgs::image_encodings::BGR8);
	  }
	catch (cv_bridge::Exception& e)
	  {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	  }

	cv::Mat ColorImage;
	cv_ptrRGB->image.copyTo(ColorImage);

	cv_bridge::CvImageConstPtr cv_ptrMONO;
	std::vector<cv::Mat> maskImgs;
	for(int i=0; i<masks.size(); i++)
	  {
		try
		  {
			cv_ptrMONO = cv_bridge::toCvCopy(masks[i], sensor_msgs::image_encodings::MONO8);
		  }
		catch (cv_bridge::Exception& e)
		  {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		  }
		cv::Mat BiImage;		
		cv_ptrMONO->image.copyTo(BiImage);
		maskImgs.push_back(BiImage);
	  }
	
	cv::Mat combine_mask = maskImgs[0];
	if(maskImgs.size()>1)
	  {
		for(int i=1; i<maskImgs.size(); i++)
		  cv::add(combine_mask,maskImgs[i] ,combine_mask);
	  }


	cv::Mat color_mask;
	bitwise_and(ColorImage,ColorImage,color_mask,combine_mask);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_mask).toImageMsg();
	image_pub.publish(msg);
  }

  void update_target_pointcloud(std::vector<float> boundingbox)
  {
	SynDepth_ = DepthImg_;
	cv_bridge::CvImageConstPtr cv_ptrD;
	try
	  {
		cv_ptrD = cv_bridge::toCvCopy(SynDepth_);
	  }
	catch (cv_bridge::Exception& e)
	  {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	  }

	cv::Mat depth;
	cv::Mat depth_f;
	cv_ptrD->image.copyTo(depth);
	if (depth.type()==2)
	  depth.convertTo(depth_f,CV_32FC1, 1.0/1000);
	else if (depth.type()==5)
	  depth_f = depth;
	else
	  {
		cout<<"unknown depth Mat type"<<endl;
	  }
    pcl::PointCloud<PCType>::Ptr cloud = depth_to_pointcloud(depth_f);

	//add header for transformation
	cloud->header.frame_id = SynDepth_.header.frame_id;
	//transform point cloud
	try{
	  listener.waitForTransform(tf_reference_frame_, SynDepth_.header.frame_id, ros::Time(0), ros::Duration(3.0));
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	}
	pcl::PointCloud<PCType>::Ptr cloud_trans(new pcl::PointCloud<PCType>());
	pcl_ros::transformPointCloud(tf_reference_frame_, *cloud, *cloud_trans, listener);

	pcl::PointCloud<PCType>::Ptr pre_manipulate_object_cloud = manipulate_object_cloud;
	//general 3d bounding box from the pre_manipulate_object_cloud
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D (*pre_manipulate_object_cloud, minPt, maxPt);

	//crop cloud
	pcl::search::Search <PCType>::Ptr tree (new pcl::search::KdTree<PCType>);
	pcl::PointCloud <PCType>::Ptr cloud_cropped (new pcl::PointCloud <PCType>);
	pcl::ConditionAnd<PCType>::Ptr range_cond (new pcl::ConditionAnd<PCType> ());
	range_cond->addComparison (pcl::FieldComparison<PCType>::ConstPtr (new pcl::FieldComparison<PCType> ("x", pcl::ComparisonOps::GT, minPt.x+boundingbox[0])));
	range_cond->addComparison (pcl::FieldComparison<PCType>::ConstPtr (new pcl::FieldComparison<PCType> ("x", pcl::ComparisonOps::LT, maxPt.x+boundingbox[1])));
	range_cond->addComparison (pcl::FieldComparison<PCType>::ConstPtr (new pcl::FieldComparison<PCType> ("y", pcl::ComparisonOps::GT, minPt.y+boundingbox[2])));
	range_cond->addComparison (pcl::FieldComparison<PCType>::ConstPtr (new pcl::FieldComparison<PCType> ("y", pcl::ComparisonOps::LT, maxPt.y+boundingbox[3])));
	range_cond->addComparison (pcl::FieldComparison<PCType>::ConstPtr (new pcl::FieldComparison<PCType> ("z", pcl::ComparisonOps::GT, minPt.z+boundingbox[4])));
	range_cond->addComparison (pcl::FieldComparison<PCType>::ConstPtr (new pcl::FieldComparison<PCType> ("z", pcl::ComparisonOps::LT, maxPt.z+boundingbox[5])));
	pcl::ConditionalRemoval<PCType> condrem;
	condrem.setCondition (range_cond);
	condrem.setInputCloud (cloud_trans);
	condrem.setKeepOrganized(true);
	condrem.filter (*cloud_cropped);

	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(*cloud_cropped, *cloud_cropped, nan_indices);

	pcl::PLYWriter ply_saver; 
	ply_saver.write("frame_updated_cloud.ply",*cloud_cropped);    

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud_cropped, *cloud_pub);
	for(int i=0; i<cloud_pub->points.size();i++)
	  {
		cloud_pub->points[i].r = 130;
		cloud_pub->points[i].g = 0;
		cloud_pub->points[i].b = 130;
	  }
	publish_pointcloud(cloud_pub);
	//update associate object

	if(mrcnn_client.call(mrcnn_srv))
      {
		std::vector<int32_t> cls_ids = mrcnn_srv.response.Class_ids;
		std::vector<int32_t> scores = mrcnn_srv.response.Scores;
		std::vector<sensor_msgs::Image> masks = mrcnn_srv.response.Masks;
		if(cls_ids.size()>0)
		  {
			for(int i=0; i<cls_ids.size(); i++) 
			  {
				if(objects[cls_ids[i]] == associate_object)
				  {
					ROS_INFO("[drawer_demo] found associate object");
					found_associate_obj_flag = true;
					associate_object_cloud = mask_to_pointcloud(masks[i],SynDepth_);
					break;
				  }
			  }
			if(found_associate_obj_flag = true)
			  {

				manipulate_object_cloud = cloud_cropped;			
				int TrajID = 0;
				numTargets = 2;
				std::vector<float> trajectory;
				std::vector<float> predict_point_label;
				bool action_found = call_mechknownet_srv(numTargets, TrajID, target_action, manipulate_object_cloud, associate_object_cloud, trajectory, predict_point_label);
				if(action_found)
				  {

					trajectory_visualize(trajectory,manipulate_object_cloud, associate_object_cloud, target_action);
					mechknownet_send_tf_srv.request.Poses = trajectory;
					mechknownet_send_tf_srv.request.KeepPrePose = true;
					if(mechknownet_send_tf_client.call(mechknownet_send_tf_srv))
					  {
						task_finish_flag = true;
						ROS_INFO("[drawer_demo] publish trajectory");
					  }
				  }
			  }
			else
			  ROS_INFO("[drawer_demo] I can not found the associate_object");
		  }
		else
		  ROS_INFO("[drawer_demo] I found nothing this time, keep founding");
	  }
	else
	  ROS_INFO("[drawer_demo] failed to call mrcnn while looking around");	
  }

  pcl::PointCloud<PCType>::Ptr depth_to_pointcloud(cv::Mat depth_f, cv::Mat mask=cv::Mat{})
  {
    pcl::PointCloud<PCType>::Ptr cloud(new pcl::PointCloud<PCType>());	
	if(mask.empty())
	  {
		for(int i = 0; i < depth_f.cols; i++) {
		  for(int j = 0; j < depth_f.rows; j++) {
			float z = depth_f.at<float>(j,i);
			if(z>0)
			  {
				pcl::PointXYZ point;
				point.x = (i-cx)*z*invfx;
				point.y = (j-cy)*z*invfy;
				point.z = z;
				cloud->points.push_back(point);
			  }    
		  }
		}
	  }
	else
	  {
		for(int i = 0; i < depth_f.cols; i++) {
		  for(int j = 0; j < depth_f.rows; j++) {
			float z = depth_f.at<float>(j,i);
			if(z>0 && mask.at<uchar>(j,i)==255)
			  {
				pcl::PointXYZ point;
				point.x = (i-cx)*z*invfx;
				point.y = (j-cy)*z*invfy;
				point.z = z;
				cloud->points.push_back(point);
			  }    
		  }
		}
	  }
	return cloud;
  }

  pcl::PointCloud<PCType>::Ptr mask_to_pointcloud(sensor_msgs::Image mask, sensor_msgs::Image syn_depth, bool remove_table_top = false)
  {
	cv_bridge::CvImageConstPtr cv_ptrMONO;
	try
	  {
		cv_ptrMONO = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::MONO8);
	  }
	catch (cv_bridge::Exception& e)
	  {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	  }

	cv_bridge::CvImageConstPtr cv_ptrD;
	try
	  {
		cv_ptrD = cv_bridge::toCvCopy(syn_depth);
	  }
	catch (cv_bridge::Exception& e)
	  {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	  }

	cv::Mat BiImage;
	cv_ptrMONO->image.copyTo(BiImage);


	cv::Mat depth;
	cv::Mat depth_f;
	cv_ptrD->image.copyTo(depth);
	if (depth.type()==2)
	  depth.convertTo(depth_f,CV_32FC1, 1.0/1000);
	else if (depth.type()==5)
	  depth_f = depth;
	else
	  {
		cout<<"unknown depth Mat type"<<endl;
	  }
    pcl::PointCloud<PCType>::Ptr cloud = depth_to_pointcloud(depth_f, BiImage);

	pcl::PointCloud<PCType>::Ptr processed_cloud = pointcloud_processor(cloud);
	//add header for transformation
	processed_cloud->header.frame_id = syn_depth.header.frame_id;
	//transform point cloud
	try{
	  listener.waitForTransform(tf_reference_frame_, syn_depth.header.frame_id, ros::Time(0), ros::Duration(3.0));
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	}
	pcl::PointCloud<PCType>::Ptr cloud_trans(new pcl::PointCloud<PCType>());
	pcl_ros::transformPointCloud(tf_reference_frame_, *processed_cloud, *cloud_trans, listener);
	pcl::PLYWriter ply_saver; 
	ply_saver.write("frame000.ply",*cloud_trans);    


	if(remove_table_top)
	  {
		pcl::search::Search <PCType>::Ptr tree (new pcl::search::KdTree<PCType>);
		pcl::PointCloud <PCType>::Ptr cloud_cropped (new pcl::PointCloud <PCType>);
		pcl::ConditionAnd<PCType>::Ptr range_cond (new pcl::ConditionAnd<PCType> ());
		range_cond->addComparison (pcl::FieldComparison<PCType>::ConstPtr (new pcl::FieldComparison<PCType> ("z", pcl::ComparisonOps::GT, 0.73)));
		pcl::ConditionalRemoval<PCType> condrem;
		condrem.setCondition (range_cond);
		condrem.setInputCloud (cloud_trans);
		condrem.setKeepOrganized(true);
		condrem.filter (*cloud_cropped);
		std::vector<int> nan_indices;
		pcl::removeNaNFromPointCloud(*cloud_cropped, *cloud_cropped, nan_indices);
		return cloud_cropped;
	  }
	return cloud_trans;

  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mechknownet_system");
  call_mechknownet call(argc,argv);

  ros::Rate r(10);

  ros::spin();
	
  return 0;
}

