#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Secondary_trajectory
{
	ros::NodeHandle nh;
    ros::Publisher secondaryPosePub;
    ros::Publisher secondaryTwistPub;
    ros::Subscriber primaryPoseSub, advanceSub;
    tf::TransformListener tfl;
    geometry_msgs::Twist secondaryTwist;
	geometry_msgs::PoseStamped secondaryPose;
    std::string bebopName;//namespace for the bebop    
    double loopRate, e1_max,e2_max,radius,sec,go_timer;
    int secondary_type;
    bool advancePath;
    
    ros::Time time_init, time_last;
    
	public:
		Secondary_trajectory()
		{
			ros::NodeHandle nhp("~");
			nhp.param<double>("loopRate",loopRate,300); //Hz
			nhp.param<int>("secondary_type",secondary_type,0); //which type of trajectory to use
			nhp.param<double>("e1_max",e1_max,0.1); 
			nhp.param<double>("e2_max",e2_max,0.8); 
			nhp.param<double>("feedback_radius",radius,1); 
			nhp.param<double>("sec",sec,40); 
			nhp.param<std::string>("bebopName",bebopName,"bebop1");
			
			primaryPoseSub = nh.subscribe(bebopName+"/primaryPose",1,&Secondary_trajectory::poseCB,this);
			advanceSub = nh.subscribe(bebopName+"/advancePath",1,&Secondary_trajectory::advanceCB,this);

			secondaryTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/secondaryTwist",1);
			secondaryPosePub = nh.advertise<geometry_msgs::PoseStamped>(bebopName+"/secondaryPose",1);
			
			bool advancePath = false;
			radius = radius - e1_max - e2_max;
			go_timer = 0;
			time_init = ros::Time::now();
			
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
			switch(secondary_type){
				
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
					std::cout << "No trajectory type found under type " << secondary_type << std::endl;
					xdDot << 0,0,0;
					xd << 0,0,0;
					break;
			}
			
		
			// publish secondaryPose	
			Eigen::Quaterniond q(Eigen::AngleAxisd(xd(2),Eigen::Vector3d::UnitZ()));
			
			secondaryPose.header.stamp = time_now;
			secondaryPose.header.frame_id = "world";
			secondaryPose.pose.position.x = xd(0);
			secondaryPose.pose.position.y = xd(1);
			secondaryPose.pose.position.z = 1;
			secondaryPose.pose.orientation.x = q.x();
			secondaryPose.pose.orientation.y = q.y();
			secondaryPose.pose.orientation.z = q.z();
			secondaryPose.pose.orientation.w = q.w();
		
			secondaryPosePub.publish(secondaryPose);
		
			// publish vd	
				
			secondaryTwist.linear.x = xdDot(0);
			secondaryTwist.linear.y = xdDot(1);
			secondaryTwist.linear.z = 0;
			secondaryTwist.angular.x = 0;
			secondaryTwist.angular.y = 0;
			secondaryTwist.angular.z = xdDot(2);
			
			secondaryTwistPub.publish(secondaryTwist);
		}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Secondary_trajectory_node");
    
    Secondary_trajectory obj;
    
	ros::spin();
    return 0;
}
