#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>


class TurtlePractice
{
	ros::NodeHandle nh;
	ros::Subscriber TurtlePosSub;
	ros::Publisher TurtleDesTwistPub;
	geometry_msgs::Twist velCmd;
	
	double x_c,y_c,theta,x_rc,y_rc,theta_rc;
	double x_tilde,y_tilde,theta_tilde,v1r,v2r,v1,v2;
	double e1,e2,e3;
	ros::Time init_t,t;
	
public:
	
	TurtlePractice()
{
	//initialize
	TurtlePosSub = nh.subscribe("turtlebot3/ground_pose",1,&TurtlePractice::positionCB,this);
	TurtleDesTwistPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi",1);
	init_t = ros::Time::now();

}
	
	void positionCB(const geometry_msgs::Pose2DConstPtr& posePtr)
	{
		double r = 0.8; //radius(m)
		// double r = 0.6; //radius(m)
		double f = 0.05; //frequency
		double k1 = 0.5;
		double k2 = 3;
		double pi = M_PI;

		t = ros::Time::now();
		
		x_c = posePtr->x;
		y_c = posePtr->y;
		theta = posePtr->theta;

		if (theta < 0)
		{
			theta = theta + 2*pi;
		}
		
		// std::cout << x_c << std::endl;
		// std::cout << y_c << std::endl;
		// std::cout << theta << std::endl;

		double dt = double((t-init_t).toSec());
		
		x_rc = 0.0 + r * cos(2*pi*dt*f);
		y_rc = 0.0 + r * sin(2*pi*dt*f);
		theta_rc = 2*pi*dt*f + pi/2;
		
		x_tilde = x_c - x_rc;
		y_tilde = y_c - y_rc;
		theta_tilde = theta - theta_rc;
		
		while (fabs(theta_tilde) >= 2*pi)
		{
			if (theta_tilde >= 2*pi)
			{
				theta_tilde = theta_tilde - 2*pi;
			}
			else
			{
				theta_tilde = theta_tilde + 2*pi;
			}
		}
		
		if (theta_tilde >= pi)
		{
			theta_tilde = -1 * (2*pi - theta_tilde);
		}
		else if (theta_tilde <= -pi)
		{
			theta_tilde = 2*pi + theta_tilde;
		}
		
		e1 = cos(theta) * x_tilde + sin(theta) * y_tilde;
		e2 = -sin(theta) * x_tilde + cos(theta) * y_tilde;
		e3 = theta_tilde;
		
		v1r = sqrt(pow((-r*sin(dt*f*2*pi)*f*2*pi),2) + pow((r*cos(dt*f*2*pi)*f*2*pi),2));
		v2r = f*2*pi;
		
		v1 = -k1*e1 + v1r*cos(e3); //linear velocity
		v2 = -v1r*sin(e3)*e2/e3 - k2*e3 + v2r; //angular velocity
		
		velCmd.linear.x = v1;
		velCmd.linear.y = 0;
		velCmd.linear.z = 0;
		velCmd.angular.x = 0;
		velCmd.angular.y = 0;
		velCmd.angular.z = v2;
		TurtleDesTwistPub.publish(velCmd);

	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot_practice");
	
	TurtlePractice turtle_practice;
	ros::spin();
	return 0;
	
}
	
