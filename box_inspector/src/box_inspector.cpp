//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file

#include <algorithm>
#include <list>
#include <string>

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box (actual and desired)
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
  void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {

  ROS_WARN("HIT update_inspection() ");
  got_new_snapshot_=false;
  while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
  }
  ROS_INFO("got new image");
  int num_parts_seen =  box_inspector_image_.models.size();
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();

  int desired_length = desired_models_wrt_world.size();
  bool satisfied = false;

  //define types and how many exist of each type
  int types_qtys_actual[5] = {0, 0, 0, 0, 0};
  int types_qtys_desired[5] = {0, 0, 0, 0, 0};
  //differences (actual - desired) so positive means there are orphans and negative missing
  int types_qtys_differences[5] = {0, 0, 0, 0, 0};
  
  //FIND HOW MANY OF EACH ONE WE ARE LOOKING FOR
  //for each object in desired
    //find the type
    //add it to list of types if it doesn't exist
    //if it does then increase the count

  //use the defined mapping in the header file
  int mapping_i;
  int this_type;

  //get a count of each type of mapping
  for(mapping_i = 0; mapping_i < num_parts_seen; mapping_i++){
       this_type = mappings[box_inspector_image_.models[mapping_i].type];
       types_qtys_actual[this_type - 1] += 1;
  }

  //check how many are desired for each type
  for(mapping_i = 0; mapping_i < desired_length; mapping_i++){
       this_type = mappings[desired_models_wrt_world[mapping_i].type];
       types_qtys_desired[this_type - 1] += 1;
  }

  //calculate the differences between actual and desired
  int i = 0;
  for(i = 0; i < 5; i++){
    types_qtys_differences[i] = types_qtys_actual[i] - types_qtys_desired[i];
  }

  //PRINT VALUES OF THE LISTS
  /*for(int i = 0; i < 5; i ++){
    ROS_INFO("Actual is %d, Desired is %d, Diff is %d", types_qtys_actual[i], types_qtys_desired[i], types_qtys_differences[i]);
  }*/
  //Store the models that are not satisfied in two new lists
  vector<osrf_gear::Model> desired_models_wrt_world_wo_satisfied;
  vector<osrf_gear::Model> box_inspector_image_models_wo_satisfied;
  //FIND THE SATISFIED MODELS AND REMOVE THE SATISFIED ONES FROM BOTH LISTS
  for (i=0;i<box_inspector_image_.models.size(); i++) {
    //compare all the desired models with the actual and determine which ones belong in their current position
    for(int j = 0; j < desired_models_wrt_world.size(); j++) { 
          geometry_msgs::PoseStamped model_pose_from_image_wrt_world=compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
         //compare the poses
         bool poses_same = comparePoses(model_pose_from_image_wrt_world.pose, desired_models_wrt_world[j].pose);
         //if the image is in thee desired models (presume same order)
         if(poses_same && box_inspector_image_.models[i].type == desired_models_wrt_world[j].type){
	    //the model does not need to be changed
            //ROS_WARN("SATISFIED");
            //then it should be added to the satisfied list
            satisfied_models_wrt_world.push_back(box_inspector_image_.models[i]);
            satisfied = true;
         } else {
	    //otherwise go to the next one
         }
    }
    //if the box image model is a desired model
    if(!satisfied){
      //add remaining into new list
      box_inspector_image_models_wo_satisfied.push_back(box_inspector_image_.models[i]);

      //remove it from the actual and desired list
    } 
    satisfied = false;
  }

  //ROS_WARN("GOT BOX");
  //do this again to get a list of all the desireds that weren't touched
  for (i=0;i<desired_models_wrt_world.size(); i++) {
    //compare all the desired models with the actual and determine which ones belong in their current position
    for(int j = 0; j < box_inspector_image_.models.size(); j++) { 
         //compare the poses
         bool poses_same = comparePoses(compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[j].pose).pose, desired_models_wrt_world[i].pose);
         //if the image is in the desired models (presume same order)
         if(poses_same && box_inspector_image_.models[j].type == desired_models_wrt_world[i].type){
	    //the model does not need to be changed
            //DONT DO ANYTHING THIS TIME
            satisfied = true;
         } else {
	    //otherwise go to the next one
         }
    }
    //if the box image model is a desired model
    if(!satisfied){
      //add remaining into new list
      desired_models_wrt_world_wo_satisfied.push_back(desired_models_wrt_world[i]);

      //remove it from the actual and desired list
    } 
    satisfied = false;
  }

  //ROS_WARN("GOT DESIRED");
  //differences (actual - desired) so positive means there are orphans and negative missing
  //FIND THE ORPHAN MODELS - find models that are in the box that shouldn't be
  //FIND THE MISSING MODELS - find models that are not in the box that are desired
  //FIND THE MISPLACED MODELS - find models that are in the box but use the wrong position
  for(i = 0; i < 5; i++) {
    int type_counter = 0;
    //determine for the particular type whether there are orphan, missing, or misplaced model(s)
    if(types_qtys_differences[i] > 0) {
      //these are orphans
      ROS_INFO("ORPHANS");
      for(int j = 0; j < box_inspector_image_models_wo_satisfied.size(); j++){
        //if the mapped value equals the current index
        if(mappings[box_inspector_image_models_wo_satisfied[j].type] == i + 1 && type_counter >= types_qtys_differences[i]){
          orphan_models_wrt_world.push_back(box_inspector_image_.models[j]);
        } else if (mappings[box_inspector_image_models_wo_satisfied[j].type] == i + 1){
          type_counter += 1;
        } else {
          //do nothing
        }
      }
    } else if (types_qtys_differences[i] < 0) {
      //these are missing
      ROS_INFO("MISSING");
      for(int j = 0; j < desired_models_wrt_world_wo_satisfied.size(); j++){
        //if the mapped value equals the current index
        if(mappings[desired_models_wrt_world_wo_satisfied[j].type] == i + 1 && type_counter <= types_qtys_differences[i]){
          missing_models_wrt_world.push_back(desired_models_wrt_world_wo_satisfied[j]);
        } else if (mappings[desired_models_wrt_world_wo_satisfied[j].type]) {
          type_counter -= 1;
        } else {
          //do nothing
        }
      }
    } else {
      //these are misplaced
      ROS_INFO("MISPLACED");
      for(int j = 0; j < desired_models_wrt_world_wo_satisfied.size(); j++){
        if(mappings[desired_models_wrt_world_wo_satisfied[j].type] == i + 1){
          misplaced_models_desired_coords_wrt_world.push_back(desired_models_wrt_world_wo_satisfied[j]);
        }
      }
      for(int j = 0; j < box_inspector_image_models_wo_satisfied.size(); j++){
        if(mappings[box_inspector_image_models_wo_satisfied[j].type] == i + 1){
          misplaced_models_actual_coords_wrt_world.push_back(box_inspector_image_models_wo_satisfied[j]);
        }
      }
    }
  }
}

//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/
