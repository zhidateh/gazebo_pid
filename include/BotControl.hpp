#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Float32.h> 
#include <ros/console.h>

static const double PI = 3.1415;

namespace botcontrol{
class BotControl{

private:

	ros::NodeHandle nodehandle_;

	// topics to be subscribed
	ros::Subscriber scan_sub_; // laser scan
	ros::Subscriber odom_sub_; // odometry

	// topics to be published
	ros::Publisher vel_pub_; 
	ros::Publisher error_forward_pub_; 
	ros::Publisher error_angle_pub_; 
	ros::Publisher control_signal_forward_pub_; 
	ros::Publisher control_signal_angle_pub_;

	std::vector<float> scan_data_; // from laser scan
	float scan_range_;
	float scan_ang_;

	double pos_x_, pos_y_, q_z_; // from odometry, the robot moves flat, the robot will not turn along x axis or y axis
	double ang_z_; // eular angle from quaternion q_z

	geometry_msgs::Twist vel_cmd_; // control the robot msgs
	double trans_forward_, trans_angle_; // pid output

	// PID-related states
	double error_forward_, error_angle_, error_forward_prev_, error_angle_prev_;
	double P_forward_, P_angle_; // proportional part
	double I_forward_, I_angle_; // integral part
	double D_forward_, D_angle_; // derivative part

	void odomCallBack(const nav_msgs::OdometryConstPtr& odomMsg);
	void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
	void pidAlgorithm();
	bool loadParam();

	double normalizeAngle(double angle);
	double init_x,init_y,init_ang;

public:

	double dt;
	double target_distance, target_angle;
	double Kp_f, Ki_f, Kd_f;
	double Kp_a, Ki_a, Kd_a;

	BotControl(ros::NodeHandle& nh);
	virtual ~BotControl();

	void spin();

};

}
