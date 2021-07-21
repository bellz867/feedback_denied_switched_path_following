#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
/*
class Primary_trajectory
{
	ros::NodeHandle nh;
    ros::Timer controlLoop;
    ros::Publisher primaryPosePub;
    ros::Publisher primaryTwistPub;
    tf::TransformListener tfl;
    geometry_msgs::Twist primaryTwist;
	geometry_msgs::PoseStamped primaryPose;
    
    double loopRate;
    int trajectory_type;
    
    ros::Time time_init;
    
	public:
		Primary_trajectory()
		{
			ros::NodeHandle nhp("~");
			nhp.param<double>("loopRate",loopRate,300); //Hz
			nhp.param<int>("trajectory_type",trajectory_type,0); //which type of trajectory to use
			primaryTwistPub = nh.advertise<geometry_msgs::Twist>("bebop0/primaryTwist",1);
			primaryPosePub = nh.advertise<geometry_msgs::PoseStamped>("bebop0/primaryPose",1);
			
			time_init = ros::Time::now();
			
			
			controlLoop = nh.createTimer(ros::Duration(1.0/loopRate),&Primary_trajectory::trajectoryCB,this);
		}
		
		void trajectoryCB(const ros::TimerEvent& )
		{
			ros::Time time_now = ros::Time::now();
			double dT = time_now.toSec()-time_init.toSec();
			
			Eigen::Vector3d xd,xdDot;
			switch(trajectory_type){
				
				case 0: //Circular 
				{	double radius = 1.5; //radius of circle
					double sec = 20; //time to complete one revolution
					double w = 2*M_PI/sec;
					
					xdDot << -radius*sin(w*dT), radius*cos(w*dT),w;
					xd << radius*cos(w*dT),radius*sin(w*dT),atan2f(xdDot(1),xdDot(0));
				}
					break;
				default:
					std::cout << "No trajectory type found under type " << trajectory_type << std::endl;
					xdDot << 0,0,0;
					xd << 0,0,0;
					break;
			}
			
			// publish primaryPose	
			Eigen::Quaterniond q(Eigen::AngleAxisd(xd(2),Eigen::Vector3d::UnitZ()));
				
			primaryPose.header.stamp = time_now;
			primaryPose.header.frame_id = "world";
			primaryPose.pose.position.x = xd(0);
			primaryPose.pose.position.y = xd(1);
			primaryPose.pose.position.z = 1;
			primaryPose.pose.orientation.x = q.x();
			primaryPose.pose.orientation.y = q.y();
			primaryPose.pose.orientation.z = q.z();
			primaryPose.pose.orientation.w = q.w();
		
			primaryPosePub.publish(primaryPose);
		
			// publish vd	
				
			primaryTwist.linear.x = xdDot(0);
			primaryTwist.linear.y = xdDot(1);
			primaryTwist.linear.z = 0;
			primaryTwist.angular.x = 0;
			primaryTwist.angular.y = 0;
			primaryTwist.angular.z = xdDot(2);
			
			primaryTwistPub.publish(primaryTwist);
		}
};*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Primary_trajectory_node");
    
    //Primary_trajectory obj;
    
	ros::spin();
    return 0;
}

