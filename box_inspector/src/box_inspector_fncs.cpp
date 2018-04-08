#include <std_msgs/Float64.h>

bool BoxInspector::comparePoses(geometry_msgs::Pose a, geometry_msgs::Pose b){
    double pos_variance = 0.03; //3 cm variance toleration
    float quat_variance = 0.1; //.1 radian variance toleration
    //check within 3 cm variance between poses a and b's position
    double dist;
    double x_start, x_goal, y_start, y_goal, z_start, z_goal;
    x_start = a.position.x;
    y_start = a.position.y;
    z_start = a.position.z;
    x_goal = b.position.x;
    y_goal = b.position.y;
    z_goal = b.position.z;
    double dx,dy,dz;
    dx = x_goal - x_start;
    dy = y_goal - y_start;
    dz = z_goal - z_start;
    dist = sqrt(dx*dx +dy*dy + dz*dz);
    
    double phi_a = xformUtils_.convertPlanarQuat2Phi(a.orientation);
    double phi_b = xformUtils_.convertPlanarQuat2Phi(b.orientation);
    double phi_diff = phi_a-phi_b;
    if(dist > pos_variance){
        //ROS_INFO("POSITION VARIANCE TOO MUCH: %f, %f, %f, %f", dist, dx, dy, dz);
	return false;
    } else if ((phi_diff > quat_variance && phi_diff > 0) || (phi_diff < quat_variance * (-1) && phi_diff < 0)){
        //ROS_INFO("ORIENTATION VARIANCE TOO MUCH %f", phi_diff);
        return false;
    } else {
        //ROS_INFO("WITHIN TOLERATION");
        return true;
    }
}


//given an image, compute all models w/rt box and return in a "Shipment" object
bool BoxInspector::model_poses_wrt_box(osrf_gear::Shipment &shipment_status) {
    ROS_INFO("model_poses_wrt_box()");
    geometry_msgs::Pose cam_pose, box_pose_wrt_cam, model_pose_wrt_cam, part_pose_wrt_box;
    geometry_msgs::PoseStamped box_pose_wrt_world, part_pose_wrt_world;
    Eigen::Affine3d affine_cam_wrt_world, affine_part_wrt_cam, affine_part_wrt_box,
            affine_box_pose_wrt_world, affine_part_wrt_world;

    get_new_snapshot_from_box_cam();
    bool found_box = false;
    int i_box = 0;
    int num_models;

    num_models = box_inspector_image_.models.size();
    //parse the  image and compute model poses w/rt box; ignore  the box itself
    if (num_models == 0) {
        ROS_WARN("model_poses_wrt_box(): image has zero models");
        return false;
    }
    ROS_INFO("box cam sees %d models", num_models);

    //look for the box:
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;

    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose_wrt_cam = model.pose;
            ROS_INFO_STREAM("box pose w/rt camera: " << box_pose_wrt_cam << endl);
            found_box = true;
            i_box = imodel; //remember where box is in the list of models
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose_wrt_cam);
            affine_box_pose_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world);
            break;
        }
    }
    if (!found_box) {
        ROS_WARN("model_poses_wrt_box(): did not find box in view");
        return false;
    }

    osrf_gear::Product product;
    //if here, image contains box, and box pose w/rt world is in affine_box_pose_wrt_world
    for (int imodel = 0; imodel < num_models; imodel++) {
        if (imodel != i_box) { //if here, have a model NOT the box
            model = box_inspector_image_.models[imodel];
            string model_name(model.type);
            ROS_INFO_STREAM("model: " << model << endl);
            model_pose_wrt_cam = model.pose;
            part_pose_wrt_world = compute_stPose(cam_pose, model_pose_wrt_cam);
            ROS_INFO_STREAM("part pose wrt world: " << part_pose_wrt_world << endl);
            
            ROS_WARN("model_poses_wrt_box(): FINISHED! compute part_pose_wrt_box");
            //Use the following defined above Eigen::Affine3d affine_cam_wrt_world, affine_part_wrt_cam, affine_part_wrt_box, affine_box_pose_wrt_world, affine_part_wrt_world;

            //transformPoseToEigenAffine3d for converting a geometry_msgs::Pose to Eigen::Affine
            //get the values for part_pose with respect to world
            affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
            //get the inverse of the box pose wrt the world and multiply by the part position wrt world
            Eigen::Affine3d affine_box_pose_wrt_world_inverse = affine_box_pose_wrt_world.inverse();
            affine_part_wrt_box = affine_box_pose_wrt_world_inverse * affine_part_wrt_world;
            //convert back to gemoetry_msgs::Pose
            part_pose_wrt_box = xformUtils_.transformEigenAffine3dToPose(affine_part_wrt_box);
            //put this into "shipment"  object:
            //string shipment_type
            //box_inspector/Product[] products
            //  string type
            //  geometry_msgs/Pose pose    
            product.type = model.type;
            product.pose = part_pose_wrt_box;
            shipment_status.products.push_back(product);
        }
    }
    ROS_INFO_STREAM("resulting part poses w/rt box: " << shipment_status << endl);

}



//given a shipment description that specifies desired parts and  poses with respect to box,
//convert this to poses of parts w/rt world;
//robot needs to know current and desired part poses  w/rt world

void BoxInspector::compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box,
        geometry_msgs::PoseStamped box_pose_wrt_world,
        vector<osrf_gear::Model> &desired_models_wrt_world) {

    //Function started
    ROS_INFO("BEGIN COMPUTE SHIMPMENT");

    //number of products in the shipment
    int num_products = shipment_wrt_box.products.size();
     
    if(num_products > 0){
        //current desired model to add to list of the desired models and its pose initialized
    	geometry_msgs::PoseStamped curr_desired_model_wrt_world;
        geometry_msgs::Pose model_wrt_box_pose;

        //hold current model information to pass into desired models wrt world
        osrf_gear::Model model;

        //compute and fill in terms in desired_models_wrt_world
        for(int i = 0; i < num_products; i++){
            ROS_INFO("WORKING ON PRODUCT %d of %d", i, num_products);
            
            //get the pose of the current product
            geometry_msgs::Pose model_box_wrt_pose = shipment_wrt_box.products[i].pose;

            //helper function from box_inspector
            curr_desired_model_wrt_world = compute_stPose(box_pose_wrt_world.pose, model_box_wrt_pose);

            //add current product information to the model object
            model.type = shipment_wrt_box.products[i].type;
            model.pose = curr_desired_model_wrt_world.pose;

            //push it to the list of desired models
            desired_models_wrt_world.push_back(model);
        }
    } else {
        //if there are no products there is nothing we need to do
        ROS_INFO("NO PRODUCTS");
    }

    //Function is completed
    ROS_INFO("compute_shipment_poses_wrt_world() completed");
}


