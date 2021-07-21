#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Primary_trajectory
{
	ros::NodeHandle nh;
    ros::Timer controlLoop;
    ros::Publisher primaryPosePub;
    ros::Publisher primaryTwistPub;
    ros::Subscriber advanceSub;
    tf::TransformListener tfl;
    geometry_msgs::Twist primaryTwist;
	geometry_msgs::PoseStamped primaryPose;
    std::string bebopName;//namespace for the bebop    
    double loopRate,radius,sec,go_timer;
    int trajectory_type;
    bool advancePath;
    
    ros::Time time_init, time_last;
    
	public:
		Primary_trajectory()
		{
			ros::NodeHandle nhp("~");
			nhp.param<double>("loopRate",loopRate,300); //Hz
			nhp.param<int>("trajectory_type",trajectory_type,0); //which type of trajectory to use
			nhp.param<double>("primary_radius",radius,1.5); 
			nhp.param<double>("sec",sec,40);
			nhp.param<std::string>("bebopName",bebopName,"bebop1");
			primaryTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/primaryTwist",1);
			primaryPosePub = nh.advertise<geometry_msgs::PoseStamped>(bebopName+"/primaryPose",1);

			time_init = ros::Time::now();
			
			advanceSub = nh.subscribe(bebopName+"/advancePath",1,&Primary_trajectory::advanceCB,this);
			controlLoop = nh.createTimer(ros::Duration(1.0/loopRate),&Primary_trajectory::trajectoryCB,this);
			
			advancePath = false;
			time_init = ros::Time::now();
			time_last = time_init;
			go_timer = 0;
		}
		void advanceCB(const std_msgs::Bool::ConstPtr& advancePathMsg)
		{
			advancePath = advancePathMsg->data;
		}		
		void trajectoryCB(const ros::TimerEvent& )
		{
			ros::Time time_now = ros::Time::now();
			
			if(advancePath)
			{
				go_timer +=  time_now.toSec()-time_last.toSec();
			}			
			time_last = time_now;
			
			Eigen::Vector3d xd,xdDot;
			switch(trajectory_type){
				
				case 0: //Circular 
				{	
					double w = 2*M_PI/sec;
					if(advancePath)
					{
						xdDot << -radius*w*sin(w*go_timer), radius*w*cos(w*go_timer),w;
						xd << radius*cos(w*go_timer),radius*sin(w*go_timer),atan2f(xdDot(1),xdDot(0));
					}
					else
					{
						xdDot << 0,0,0;
						xd << radius*cos(w*go_timer),radius*sin(w*go_timer),atan2f(radius*w*cos(w*go_timer),-radius*w*sin(w*go_timer));
					}
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Primary_trajectory_node");
    
    Primary_trajectory obj;
    
	ros::spin();
    return 0;
}
