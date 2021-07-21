#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Feedback_boundary
{
	ros::NodeHandle nh;
    ros::Publisher boundaryPosePub;
    ros::Publisher boundaryTwistPub;
    ros::Subscriber primaryPoseSub,advanceSub;
    tf::TransformListener tfl;
    geometry_msgs::Twist boundaryTwist;
	geometry_msgs::PoseStamped boundaryPose;
    std::string bebopName;//namespace for the bebop    
    double loopRate,radius,sec,go_timer;
    int boundary_type;
    bool advancePath;
    ros::Time time_init,time_last;
    
	public:
		Feedback_boundary()
		{
			ros::NodeHandle nhp("~");
			nhp.param<double>("loopRate",loopRate,300); //Hz
			nhp.param<int>("boundary_type",boundary_type,0); //which type of trajectory to use
			nhp.param<double>("boundary_radius",radius,1); 
			nhp.param<double>("sec",sec,40);
			nhp.param<std::string>("bebopName",bebopName,"bebop1");			
			primaryPoseSub = nh.subscribe(bebopName+"/primaryPose",1,&Feedback_boundary::poseCB,this);
			advanceSub = nh.subscribe(bebopName+"/advancePath",1,&Feedback_boundary::advanceCB,this);
			boundaryTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/boundaryTwist",1);
			boundaryPosePub = nh.advertise<geometry_msgs::PoseStamped>(bebopName+"/boundaryPose",1);
			advancePath = false;
			time_init = ros::Time::now();
			time_last = time_init;
			go_timer = 0;
		}
		void advanceCB(const std_msgs::Bool::ConstPtr& advancePathMsg)
		{
			advancePath = advancePathMsg->data;
		}
		void poseCB(const geometry_msgs::PoseStampedPtr& priPose)
		{
			ros::Time time_now = priPose->header.stamp;
			
			if(advancePath)
			{
				go_timer +=  time_now.toSec()-time_last.toSec();
			}			
			time_last = time_now;
			
			Eigen::Vector3d xd,xdDot;
			switch(boundary_type){
				
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
					std::cout << "No trajectory type found under type " << boundary_type << std::endl;
					xdDot << 0,0,0;
					xd << 0,0,0;
					break;
			}
			
			// publish boundaryPose	
			Eigen::Quaterniond q(Eigen::AngleAxisd(xd(2),Eigen::Vector3d::UnitZ()));
				
			boundaryPose.header.stamp = time_now;
			boundaryPose.header.frame_id = "world";
			boundaryPose.pose.position.x = xd(0);
			boundaryPose.pose.position.y = xd(1);
			boundaryPose.pose.position.z = 1;
			boundaryPose.pose.orientation.x = q.x();
			boundaryPose.pose.orientation.y = q.y();
			boundaryPose.pose.orientation.z = q.z();
			boundaryPose.pose.orientation.w = q.w();
		
			boundaryPosePub.publish(boundaryPose);
		
			// publish vd	
				
			boundaryTwist.linear.x = xdDot(0);
			boundaryTwist.linear.y = xdDot(1);
			boundaryTwist.linear.z = 0;
			boundaryTwist.angular.x = 0;
			boundaryTwist.angular.y = 0;
			boundaryTwist.angular.z = xdDot(2);
			
			boundaryTwistPub.publish(boundaryTwist);
		}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Feedback_boundary_node");
    
    Feedback_boundary obj;
    
	ros::spin();
    return 0;
}
