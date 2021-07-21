#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Switched_trajectory
{
	ros::NodeHandle nh;
    ros::Publisher switchedPosePub;
    ros::Publisher switchedTwistPub;
    ros::Publisher advancePub;
    ros::Subscriber mocapSub, primaryPoseSub,primaryTwistSub, secondaryPoseSub, secondaryTwistSub, boundaryPoseSub, boundaryTwistSub, predictorPoseSub;
    tf::TransformListener tfl;
    geometry_msgs::Twist primaryTwist,secondaryTwist,boundaryTwist,switchedTwist;
	geometry_msgs::PoseStamped primaryPose,secondaryPose,boundaryPose,switchedPose,bebopPose, predictorPose;
	double dwell_time_max, dwell_time_min;
    Eigen::Vector3d e_last,eHat_last,zeta_last,e,eHat,zeta;
    double loopRate, e_max,e_min,lu,ls,epsilon,v_max,v_min;
    std::vector<double> k1,dmax;
    int switched_type,stage;
    std::string bebopName;
    bool recordE,feedback,pp,pt,sp,st,bp,bt,pdp,dataSaved,advancePath;
    
    Eigen::Vector3d e_reentry,eHat_reentry,zeta_reentry;
    ros::Timer controlLoop;
    ros::Time time_init,stage_init;
    double init;
	std::vector<double> maxDTdata, minDTdata;

	public:
		Switched_trajectory()
		{
			ros::NodeHandle nhp("~");
			nhp.param<double>("loopRate",loopRate,300); //Hz
			nhp.param<int>("switched_type",switched_type,0); //which type of trajectory to use
			nhp.param<double>("e_max",e_max,1); 
			nhp.param<double>("e_min",e_min,0.1);
			nhp.param<double>("lu",lu,1); 
			nhp.param<std::vector<double>>("k1",k1,{1,1,1}); 
			nhp.param<std::vector<double>>("dmax",dmax,{0.05,0.05,0.05});
			nhp.param<std::string>("bebopName",bebopName,"bebop1");


			mocapSub = nh.subscribe(bebopName+"/mocapPose",1,&Switched_trajectory::mocapCB,this);
			primaryPoseSub = nh.subscribe(bebopName+"/primaryPose",1,&Switched_trajectory::priPoseCB,this);
			primaryTwistSub = nh.subscribe(bebopName+"/primaryTwist",1,&Switched_trajectory::priTwistCB,this);
			secondaryPoseSub = nh.subscribe(bebopName+"/secondaryPose",1,&Switched_trajectory::secPoseCB,this);
			secondaryTwistSub = nh.subscribe(bebopName+"/secondaryTwist",1,&Switched_trajectory::secTwistCB,this);
			boundaryPoseSub = nh.subscribe(bebopName+"/boundaryPose",1,&Switched_trajectory::boundPoseCB,this);
			boundaryTwistSub = nh.subscribe(bebopName+"/boundaryTwist",1,&Switched_trajectory::boundTwistCB,this);
			predictorPoseSub = nh.subscribe(bebopName+"/predictorPose",1,&Switched_trajectory::predictorPoseCB,this);

			switchedTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/switchedTwist",1);
			switchedPosePub = nh.advertise<geometry_msgs::PoseStamped>(bebopName+"/switchedPose",1);
			advancePub = nh.advertise<std_msgs::Bool>(bebopName+"/advancePath",1);
			
			
			controlLoop = nh.createTimer(ros::Duration(1.0/loopRate),&Switched_trajectory::switchedCB,this);
			
			v_max = (1.0/2)*pow(e_max,2);
			v_min = (1.0/2)*pow(e_min,2);
			ls = 2*(*min_element(k1.begin(),k1.end()));
			epsilon = (1.0/2)*(dmax.at(0)*dmax.at(0)+dmax.at(1)*dmax.at(1)+dmax.at(2)*dmax.at(2));
			stage = 0;
			feedback = true;
			init = false;
			pp = false;
			pt = false;
			sp = false;
			st = false;
			bp = false;
			bt = false;
			pdp = false;
			dataSaved = false;
			recordE = false;
			advancePath = false;
			time_init = ros::Time::now();
			stage_init = time_init;

			switchedPose.pose.position.x = 1-e_max;
			switchedPose.pose.position.y = 0;
			switchedPose.pose.position.z = 1;
			switchedPose.pose.orientation.z = M_PI/2;
			
			switchedPose.header.stamp = time_init;
			switchedPose.header.frame_id = "world";
		}
		
		void mocapCB(const geometry_msgs::PoseStampedPtr& pose)
		{

			bebopPose = *pose;
			feedback = checkFeedback(bebopPose);
			if(pp && pt && sp && st && bp && bt && pdp)
			{
				tf::Transform tfPose, tfRot;
				tfPose.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
				tfPose.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w));
		
				tfRot.setOrigin(tf::Vector3(0,0,0));
				tfRot.setRotation(tfPose.getRotation());
			
				tf::Quaternion rot(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
				tf::Quaternion rotd(switchedPose.pose.orientation.x, switchedPose.pose.orientation.y, switchedPose.pose.orientation.z, switchedPose.pose.orientation.w);
				tf::Quaternion rotp(predictorPose.pose.orientation.x, predictorPose.pose.orientation.y, predictorPose.pose.orientation.z, predictorPose.pose.orientation.w);
			
				tf::Matrix3x3 m(rot);
				tf::Matrix3x3 md(rotd);
				tf::Matrix3x3 mp(rotp);
			
			
				double roll,pitch,yaw,yawd,yawp;
			
				m.getRPY(roll,pitch,yaw);
				md.getRPY(roll,pitch,yawd);
				mp.getRPY(roll,pitch,yawp);
				
				
				
				e << bebopPose.pose.position.x - switchedPose.pose.position.x, bebopPose.pose.position.y - switchedPose.pose.position.y, yawp - yawd; 
				eHat << predictorPose.pose.position.x - switchedPose.pose.position.x, predictorPose.pose.position.y - switchedPose.pose.position.y, yawp - yawd; 
				zeta << bebopPose.pose.position.x-predictorPose.pose.position.x, bebopPose.pose.position.y-predictorPose.pose.position.y, yaw - yawp;
				

				
				// make sure theta tilde is between 0 and 2*pi
				double theta_tilde1 = fmod(e(2),2.0*M_PI);
				double theta_tilde2 = fmod(eHat(2),2.0*M_PI);
				double theta_tilde3 = fmod(zeta(2),2.0*M_PI);
		
				// flips theta tilde to negative if > pi and vice versa
				if (theta_tilde1 >= M_PI){
					theta_tilde1 = -1 * (2*M_PI - theta_tilde1);
				}
				else if (theta_tilde1 <= -M_PI)
				{
					theta_tilde1 = 2*M_PI + theta_tilde1;
				}
		
				if (theta_tilde2 >= M_PI)
				{
					theta_tilde2 = -1 * (2*M_PI - theta_tilde2);
				}
				else if (theta_tilde2 <= -M_PI)
				{
					theta_tilde2 = 2*M_PI + theta_tilde2;
				}
				
				if (theta_tilde3 >= M_PI)
				{
					theta_tilde3 = -1 * (2*M_PI - theta_tilde3);
				}
				else if (theta_tilde3 <= -M_PI)
				{
					theta_tilde3 = 2*M_PI + theta_tilde3;
				}
		
				e(2) = theta_tilde1;
				eHat(2) = theta_tilde2;
				zeta(2) = theta_tilde3;
				
				if(!init)
				{
					e_last = e;
					eHat_last = eHat;
					zeta_last = zeta;
					double v_sigma = (1.0/2.0)*(e_last(0)*e_last(0) + e_last(1)*e_last(1));
					dwell_time_min = -(1.0/ls)*log((v_min)/(v_sigma));
					std::cout << dwell_time_min << std::endl;
				
					if(dwell_time_min < 3.5)
					{ 
						dwell_time_min = 3.5; // Lower bounding dt_min by 2 secs;
						std::cout << "Lower bounding dt_min by 3.5 secs." << std::endl;
					}
					minDTdata.push_back(dwell_time_min);
					//dwell_time_min = 10;
				}
				init = true;
			}
			else return;		
		}
		void switchedCB(const ros::TimerEvent& )
		{
			bool linear_switching = 1;
			ros::Time time_now = ros::Time::now();
			
			if((time_now-time_init).toSec() > 200 && !dataSaved)
			{
			
				std::ofstream maxDTfile("/home/ncr/experiment_data/bebop_sw/saved_data/maxDT.txt");
				if (maxDTfile.is_open())
				{
					maxDTfile << "max_DT" << "\n";
					for (int i = 0; i < maxDTdata.size(); i++)
					{
						maxDTfile << maxDTdata.at(i) << "\n";
					}
				maxDTfile.close();
				}
				std::ofstream minDTfile("/home/ncr/experiment_data/bebop_sw/saved_data/minDT.txt");
				if (minDTfile.is_open())
				{
					minDTfile << "min_DT" << "\n";
					for (int i = 0; i < minDTdata.size(); i++)
					{
						minDTfile << minDTdata.at(i) << "\n";
					}
				minDTfile.close();
				}
				dataSaved = true;
			}
			
			double time_stage = time_now.toSec() - stage_init.toSec();

			if(!init)
			{ 
				return;
			}
			if(feedback)
			{
				e_last = e; 
				eHat_last = eHat;
				zeta_last = zeta;
			}
			if(!recordE && stage == 1 && feedback && time_stage > dwell_time_max*0.5)
			{
				e_reentry = e_last;
				eHat_reentry = eHat_last;
				zeta_reentry = zeta_last;
				//std::cout << "e1" << std::endl << e1_reentry << std::endl;
				//std::cout << "e2" << std::endl << e2_reentry << std::endl;
				recordE = true;
			}
			if(stage == 0 && time_stage > dwell_time_min)
			{
				stage = 1;
				stage_init = ros::Time::now();
				time_stage = time_now.toSec() - stage_init.toSec();
								
				double v_sigma = (1.0/2.0)*(e_last(0)*e_last(0) + e_last(1)*e_last(1));
				
				// calculate max. dwell time condition
				
				// exponential
				//dwell_time_max = (1.0/lu)*log((v_max+(epsilon/lu))/(v_sigma+(epsilon/lu)));
				
				// linear
				double ep1 = epsilon;
				double ep2 = sqrt(epsilon*2.0)*sqrt(e_last(0)*e_last(0) + e_last(1)*e_last(1));
				dwell_time_max = (-ep2+sqrt(pow(ep2,2.0)-4.0*ep1*(v_sigma-v_max)))/(2.0*ep1);
				
				std::cout << "Switching to stage 1" << std::endl; 
				std::cout << dwell_time_max << std::endl;
				std::cout << "time:" << (time_now-time_init).toSec() << std::endl;
				if((time_now-time_init).toSec() < 400)
				{
					//std::vector<double> maxDTdata minDTdata;
					maxDTdata.push_back(dwell_time_max);
				}

			}
			else if(stage == 1 && feedback && time_stage > dwell_time_max)
			{
				stage = 0;
				stage_init = ros::Time::now();
				time_stage = time_now.toSec() - stage_init.toSec();
								
				//std::cout << "e1" << std::endl << e1_reentry << std::endl;
				//std::cout << "e2" << std::endl << e2_reentry << std::endl;
				
				double v_sigma = (1.0/2.0)*(e_reentry(0)*e_reentry(0) + e_reentry(1)*e_reentry(1));
				dwell_time_min = -(1.0/ls)*log((v_min)/(v_sigma));
				
				std::cout << "Switching to stage 0" << std::endl; 
				std::cout << dwell_time_min << std::endl;
				std::cout << "time:" << (time_now-time_init).toSec() << std::endl;
				
				if(dwell_time_min < 3.5) {
					std::cout << "Max dwell time less than 3.5 sec... something's wrong... setting to 3.5 sec to avoid issue..." << std::endl;
					//dwell_time_min = 3.5; 
				}
				
				if((time_now-time_init).toSec() < 400)
				{
					//std::vector<double> maxDTdata minDTdata;
					minDTdata.push_back(dwell_time_min);
				}
				recordE = false;
			}
					
			if(stage == 0)
			{
				double ratio = time_stage/dwell_time_min;
				
				if (ratio > 1) ratio = 1;
				
				double ss, ssDot;
				smoother_step(ratio,&ss,&ssDot,dwell_time_min);
				if (linear_switching){ ss = ratio; ssDot = 1/dwell_time_min;}
				
				switchedPose.pose.position.x = ss*boundaryPose.pose.position.x+(1-ss)*secondaryPose.pose.position.x;
				switchedPose.pose.position.y = ss*boundaryPose.pose.position.y+(1-ss)*secondaryPose.pose.position.y;
				switchedPose.pose.position.z = ss*boundaryPose.pose.position.z+(1-ss)*secondaryPose.pose.position.z;
				switchedPose.pose.orientation.z = ss*boundaryPose.pose.orientation.z+(1-ss)*secondaryPose.pose.orientation.z;
				switchedPose.pose.orientation.w = ss*boundaryPose.pose.orientation.w+(1-ss)*secondaryPose.pose.orientation.w;
				
				switchedTwist.linear.x = ssDot*boundaryPose.pose.position.x + ss*boundaryTwist.linear.x - ssDot*secondaryPose.pose.position.x + (1-ss)*secondaryTwist.linear.x;
				switchedTwist.linear.y = ssDot*boundaryPose.pose.position.y + ss*boundaryTwist.linear.y - ssDot*secondaryPose.pose.position.y + (1-ss)*secondaryTwist.linear.y;
				switchedTwist.linear.z = ssDot*boundaryPose.pose.position.z + ss*boundaryTwist.linear.z - ssDot*secondaryPose.pose.position.z + (1-ss)*secondaryTwist.linear.z;
				switchedTwist.angular.z = ssDot*boundaryPose.pose.orientation.z + ss*boundaryTwist.angular.z - ssDot*secondaryPose.pose.orientation.z + (1-ss)*secondaryTwist.angular.z;
				
				advancePath = false;
			}
			else
			{
				double phase0 = 0.4;
				double phase1 = 0.7;
				double phase2 = 1;
				
				double ratio = time_stage/dwell_time_max;
				if(ratio > 1) ratio = 1;
				
				if(ratio <= phase0)
				{
					double phase_ratio = time_stage/(dwell_time_max*phase0);
					double ss, ssDot;
					smoother_step(phase_ratio,&ss,&ssDot,dwell_time_max*phase0);
					if (linear_switching){ ss = phase_ratio; ssDot = 1/(dwell_time_max*phase0);};
					
					switchedPose.pose.position.x = ss*primaryPose.pose.position.x+(1-ss)*boundaryPose.pose.position.x;
					switchedPose.pose.position.y = ss*primaryPose.pose.position.y+(1-ss)*boundaryPose.pose.position.y;
					switchedPose.pose.position.z = ss*primaryPose.pose.position.z+(1-ss)*boundaryPose.pose.position.z;
					switchedPose.pose.orientation.z = ss*primaryPose.pose.orientation.z+(1-ss)*boundaryPose.pose.orientation.z;
					switchedPose.pose.orientation.w = ss*primaryPose.pose.orientation.w+(1-ss)*boundaryPose.pose.orientation.w;
				
					switchedTwist.linear.x = ssDot*primaryPose.pose.position.x + ss*primaryTwist.linear.x - ssDot*boundaryPose.pose.position.x + (1-ss)*boundaryTwist.linear.x;
					switchedTwist.linear.y = ssDot*primaryPose.pose.position.y + ss*primaryTwist.linear.y - ssDot*boundaryPose.pose.position.y + (1-ss)*boundaryTwist.linear.y;
					switchedTwist.linear.z = ssDot*primaryPose.pose.position.z + ss*primaryTwist.linear.z - ssDot*boundaryPose.pose.position.z + (1-ss)*boundaryTwist.linear.z;
					switchedTwist.angular.z = ssDot*primaryPose.pose.orientation.z + ss*primaryTwist.angular.z - ssDot*boundaryPose.pose.orientation.z + (1-ss)*boundaryTwist.angular.z;
					advancePath = false;
				}
				else if(ratio <= phase1)
				{
					double phase_ratio = 1;
					double ss, ssDot;
					smoother_step(phase_ratio,&ss,&ssDot,dwell_time_max*phase0);
					if (linear_switching){ ss = phase_ratio; ssDot = 0;}
					
					switchedPose.pose.position.x = ss*primaryPose.pose.position.x+(1-ss)*boundaryPose.pose.position.x;
					switchedPose.pose.position.y = ss*primaryPose.pose.position.y+(1-ss)*boundaryPose.pose.position.y;
					switchedPose.pose.position.z = ss*primaryPose.pose.position.z+(1-ss)*boundaryPose.pose.position.z;
					switchedPose.pose.orientation.z = ss*primaryPose.pose.orientation.z+(1-ss)*boundaryPose.pose.orientation.z;
					switchedPose.pose.orientation.w = ss*primaryPose.pose.orientation.w+(1-ss)*boundaryPose.pose.orientation.w;
				
					switchedTwist.linear.x = ssDot*primaryPose.pose.position.x + ss*primaryTwist.linear.x - ssDot*boundaryPose.pose.position.x + (1-ss)*boundaryTwist.linear.x;
					switchedTwist.linear.y = ssDot*primaryPose.pose.position.y + ss*primaryTwist.linear.y - ssDot*boundaryPose.pose.position.y + (1-ss)*boundaryTwist.linear.y;
					switchedTwist.linear.z = ssDot*primaryPose.pose.position.z + ss*primaryTwist.linear.z - ssDot*boundaryPose.pose.position.z + (1-ss)*boundaryTwist.linear.z;
					switchedTwist.angular.z = ssDot*primaryPose.pose.orientation.z + ss*primaryTwist.angular.z - ssDot*boundaryPose.pose.orientation.z + (1-ss)*boundaryTwist.angular.z;
					advancePath = true;
				}
				else if(ratio <= phase2)
				{
					double phase_ratio = (time_stage-dwell_time_max*phase1)/(dwell_time_max*(phase2-phase1));
					if(phase_ratio > 1) phase_ratio = 1;
					double ss, ssDot;
					smoother_step(phase_ratio,&ss,&ssDot,dwell_time_max*(phase2-phase1));
					if (linear_switching){ ss = phase_ratio; ssDot = 1/(dwell_time_max*(phase2-phase1));}
					
					switchedPose.pose.position.x = ss*secondaryPose.pose.position.x+(1-ss)*primaryPose.pose.position.x;
					switchedPose.pose.position.y = ss*secondaryPose.pose.position.y+(1-ss)*primaryPose.pose.position.y;
					switchedPose.pose.position.z = ss*secondaryPose.pose.position.z+(1-ss)*primaryPose.pose.position.z;
					switchedPose.pose.orientation.z = ss*secondaryPose.pose.orientation.z+(1-ss)*primaryPose.pose.orientation.z;
					switchedPose.pose.orientation.w = ss*secondaryPose.pose.orientation.w+(1-ss)*primaryPose.pose.orientation.w;
				
					switchedTwist.linear.x = ssDot*secondaryPose.pose.position.x + ss*secondaryTwist.linear.x - ssDot*primaryPose.pose.position.x + (1-ss)*primaryTwist.linear.x;
					switchedTwist.linear.y = ssDot*secondaryPose.pose.position.y + ss*secondaryTwist.linear.y - ssDot*primaryPose.pose.position.y + (1-ss)*primaryTwist.linear.y;
					switchedTwist.linear.z = ssDot*secondaryPose.pose.position.z + ss*secondaryTwist.linear.z - ssDot*primaryPose.pose.position.z + (1-ss)*primaryTwist.linear.z;
					switchedTwist.angular.z = ssDot*secondaryPose.pose.orientation.z + ss*secondaryTwist.angular.z - ssDot*primaryPose.pose.orientation.z + (1-ss)*primaryTwist.angular.z;
					advancePath = false;
				}
			}
/*
			switchedPose.pose.position.x = 0;
			switchedPose.pose.position.y = 0;
			switchedPose.pose.position.z = 1;
			switchedPose.pose.orientation.x = 0;
			switchedPose.pose.orientation.y = 0;
			switchedPose.pose.orientation.z = 1;
			switchedPose.pose.orientation.w = 1;
				
			switchedTwist.linear.x = 0;
			switchedTwist.linear.y = 0;
			switchedTwist.linear.z = 0;
			switchedTwist.angular.x = 0;
			switchedTwist.angular.y = 0;
			switchedTwist.angular.z = 0;*/
			switchedPose.header.stamp = time_now;
			switchedPose.header.frame_id = "world";
			switchedPosePub.publish(switchedPose);
			switchedTwistPub.publish(switchedTwist);
			std_msgs::Bool advanceBool;
			advanceBool.data = advancePath;
			advancePub.publish(advanceBool);
		}

		void priPoseCB(const geometry_msgs::PoseStampedPtr& priPose)
		{
			primaryPose = *priPose;
			//primaryPose.pose.orientation.z += sin(M_PIl/4.0);
			pp = true;
			
		}
		void priTwistCB(const geometry_msgs::TwistPtr& priTwist)
		{
			primaryTwist = *priTwist;
			pt = true;
		}
		void secPoseCB(const geometry_msgs::PoseStampedPtr& secPose)
		{
			secondaryPose = *secPose;
			//secondaryPose.pose.orientation.z += sin(M_PIl/4.0);
			sp = true;
		}
		void secTwistCB(const geometry_msgs::TwistPtr& secTwist)
		{
			secondaryTwist = *secTwist;
			st = true;
		}
		void boundPoseCB(const geometry_msgs::PoseStampedPtr& boundPose)
		{
			boundaryPose = *boundPose;
			//boundaryPose.pose.orientation.z += sin(M_PIl/4.0);
			bp = true;
		}
		void boundTwistCB(const geometry_msgs::TwistPtr& boundTwist)
		{
			boundaryTwist = *boundTwist;
			bt = true;
		}
		void predictorPoseCB(const geometry_msgs::PoseStampedPtr& predPose)
		{
			predictorPose = *predPose;
			pdp = true;
		}
		bool checkFeedback(geometry_msgs::PoseStamped pose)
		{
			double boundary_radius = 1;
			if(pow(pose.pose.position.x,2) + pow(pose.pose.position.y,2) < 1) return true;
			else return false; 
		}
		void smoother_step(double ratio, double* step, double* stepDot,double dwell_time)
		{
			
			*step = 6*pow(ratio,5)-15*pow(ratio,4)+10*pow(ratio,3);
			*stepDot = (1.0/dwell_time)*(30*pow(ratio,4)-60*pow(ratio,3)+30*pow(ratio,2));
		}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Switched_trajectory_node");
    
    Switched_trajectory obj;
    
	ros::spin();
    return 0;
}
