#include "BotControl.hpp"
#include <ros/ros.h>

using namespace botcontrol;

BotControl::BotControl(ros::NodeHandle& nh) : nodehandle_(nh){

	//load the param
	if(!loadParam()){
		ROS_ERROR("Error in loading the parameters.");
		// ros::requestShutdown();
	}

	// declare all the subscriber and publisher
	scan_sub_ = nodehandle_.subscribe("/scan", 1, &BotControl::scanCallBack, this);
	odom_sub_ = nodehandle_.subscribe("/husky_velocity_controller/odom", 1, &BotControl::odomCallBack, this);

	vel_pub_ = nodehandle_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 200);
	error_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_forward", 1); 
	error_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_angle", 1);
	control_signal_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_forward", 1);
	control_signal_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_angle", 1);

	//initialize variables
	//ENTER YOUR CODE HERE TO INITIALISE
	error_forward_ 		= 0;
	error_forward_prev_ = 0;
	error_angle_ 		= 0;
	error_angle_prev_ 	= 0;
	P_forward_			= 0;
	P_angle_			= 0;
	I_forward_ 			= 0;
	I_angle_ 			= 0;
	D_forward_ 			= 0;
	D_angle_ 			= 0;

	//END OF INITIALISATION
	ROS_INFO("Node Initialized");
}

BotControl::~BotControl(){}

void BotControl::odomCallBack(const nav_msgs::OdometryConstPtr& odomMsg){
	pos_x_ = odomMsg->pose.pose.position.x;
	pos_y_ = odomMsg->pose.pose.position.y;
	q_z_ = odomMsg->pose.pose.orientation.z;
	ang_z_ = q_z_ * 2.19;
}

void BotControl::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scanMsg){
	scan_data_ = scanMsg->ranges;
	int arr_size = scan_data_.size();
	float smallest_dist = 100;

	for(int i = 0; i<arr_size; i++){
		if(scan_data_[i] < smallest_dist) {
			smallest_dist = scan_data_[i];
			scan_ang_ = scanMsg->angle_min + scanMsg->angle_increment*i;
		}
	}
	scan_range_ = smallest_dist;

	pidAlgorithm();
}

void BotControl::pidAlgorithm(){

	std_msgs::Float32 linear_error;
	std_msgs::Float32 angle_error;
	std_msgs::Float32 linear_velocity;
	std_msgs::Float32 angle_velocity;

	//ENTER YOUR CODE HERE FOR PID COMPUTATION

	// update the PID-related error states
	error_forward_ = scan_range_  - target_distance;	
	error_angle_ =  scan_ang_ - target_angle;

	// normalise the error_angle_ within [-PI, PI]
	error_angle_ = normalizeAngle(error_angle_);

	//define proportional term
	P_angle_ 	= Kp_a * error_angle_;
	P_forward_ 	= Kp_f * error_forward_;

	// define integral term
	I_angle_ 	+= error_angle_ * dt;
	I_forward_ 	+= error_forward_ * dt;

	// define derivative term
	D_angle_ 	= Kd_a * normalizeAngle(error_angle_ - error_angle_prev_) / dt;
	D_forward_ 	= Kd_f * (error_forward_ - error_forward_prev_) / dt;

	// compute PID
	trans_forward_ 	= P_forward_ + Ki_f*I_forward_ + D_forward_;
	trans_angle_ 	= -(P_angle_ + Ki_a*I_angle_ + D_angle_);

	error_forward_prev_ = error_forward_;
	error_angle_prev_ 	= error_angle_;

	// set threshold (optional)
	trans_forward_ = std::max(0.0, std::min(trans_forward_, 7.0));


	//END OF PID COMPUTATION
	ROS_INFO("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Distance: %f", 
		trans_forward_, trans_angle_, error_angle_, scan_range_);
	
	//publish all
	vel_cmd_.linear.x = trans_forward_;
	vel_cmd_.angular.z = trans_angle_; 
	vel_pub_.publish(vel_cmd_);

	linear_error.data = error_forward_;
	error_forward_pub_.publish(linear_error);

	linear_velocity.data = trans_forward_;
	control_signal_angle_pub_.publish(linear_velocity);

	angle_error.data = error_angle_;
	error_angle_pub_.publish(angle_error);

	angle_velocity.data = trans_angle_;
	control_signal_angle_pub_.publish(angle_velocity);

}

void BotControl::spin(){
	ros::Rate loop_rate(1/dt);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

}

bool BotControl::loadParam(){


	if(!nodehandle_.getParam("/pid_control_node/Kp_f", Kp_f)){
		ROS_ERROR("Kp_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/Ki_f", Ki_f)){
		ROS_ERROR("Ki_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/Kd_f", Kd_f)){
		ROS_ERROR("Kd_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/Kp_a", Kp_a)){
		ROS_ERROR("Kp_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/Ki_a", Ki_a)){
		ROS_ERROR("Ki_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/Kd_a", Kd_a)){
		ROS_ERROR("Kd_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/target_distance", target_distance)){
		ROS_ERROR("target_distance Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/target_angle", target_angle)){
		ROS_ERROR("target_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pid_control_node/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}

double BotControl::normalizeAngle(double angle){
	if (angle > PI){
		angle -= 2*PI;
		return normalizeAngle(angle);
	}
	else if(angle < -PI){
		angle += 2*PI;
		return normalizeAngle(angle);
	}
	return angle;
}
