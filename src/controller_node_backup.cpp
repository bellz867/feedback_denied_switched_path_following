#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//differential q matrix
Eigen::MatrixXd getqDiff(Eigen::Vector4d q)
{
	Eigen::MatrixXd qDiff(4,3);
	qDiff << -q(1), -q(2), -q(3),
			  q(0), -q(3),  q(2),
			  q(3),  q(0), -q(1),
			 -q(2),  q(1),  q(0);
	return qDiff;
}

//q as matrix
Eigen::Matrix4d getqMat(Eigen::Vector4d q)
{
	Eigen::Matrix4d qMat;
	qMat << q(0), -q(1), -q(2), -q(3),
			q(1),  q(0), -q(3),  q(2),
			q(2),  q(3),  q(0), -q(1),
			q(3), -q(2),  q(1),  q(0);
	return qMat;
}

//q inverse
Eigen::Vector4d getqInv(Eigen::Vector4d q)
{
	Eigen::Vector4d qInv;
	qInv << q(0), -q(1), -q(2), -q(3);
	return qInv;
}

/***************************Copied Classes******************************/
// LS estimator for a first order approximatoion of the derivative of a state vector wrt time, thanks Anup
class DerivativeEstimator
{    
    int bufferSize; //Number of data points to store
    int stateSize; //Number of elements for the state
    bool bufferFull; //Estimation will start after buffer is full for first time
    Eigen::VectorXd timeBuff; //ring buffer for time data
    Eigen::VectorXd stateBuff; //ring buffer for position data
    int timeInd; //index of oldest time data. Data at this index gets replaced with new data
    int stateInd; //index of oldest position data Data at this index gets replaced with new data
    bool firstUpdate;//indicates first update has not happened
    
public:
	DerivativeEstimator()
	{}

    DerivativeEstimator(int bufferSizeInit, int stateSizeInit)
    {
        //Initialize buffers
        bufferSize = bufferSizeInit;
        stateSize = stateSizeInit;
        timeInd = 0;
        stateInd = 0;
        timeBuff = Eigen::VectorXd::Zero(bufferSize);
        stateBuff = Eigen::VectorXd::Zero(stateSize*bufferSize);
        bufferFull = false;
        firstUpdate = true;
    }
    
    DerivativeEstimator(const DerivativeEstimator& derivativeEstimatorNew)
    {
        //Initialize buffers
        bufferSize = derivativeEstimatorNew.bufferSize;
        stateSize = derivativeEstimatorNew.stateSize;
        timeInd = 0;
        stateInd = 0;
        timeBuff = Eigen::VectorXd::Zero(bufferSize);
        stateBuff = Eigen::VectorXd::Zero(stateSize*bufferSize);
        bufferFull = false;
        firstUpdate = true;
    }
    
    DerivativeEstimator operator=(const DerivativeEstimator& derivativeEstimatorNew)
    {
        //Initialize buffers
        bufferSize = derivativeEstimatorNew.bufferSize;
        stateSize = derivativeEstimatorNew.stateSize;
        timeInd = 0;
        stateInd = 0;
        timeBuff = Eigen::VectorXd::Zero(bufferSize);
        stateBuff = Eigen::VectorXd::Zero(stateSize*bufferSize);
        bufferFull = false;
        firstUpdate = true;
        return *this;
    }
    
    Eigen::VectorXd update(Eigen::VectorXd newMeasure, ros::Time newTime)
    {
		// Picture courtesy of Anup
        // Setting up least squares problem A*theta = P. theta is made up of the coefficients for the best fit line,
        // e.g., X = Mx*T + Bx, Y = My*t + By, Z = Mz*t + Bz. Velocity is estimated as the slope of the best fit line, i.e., Vx = Mx, Vy = My, Vz = Mz. 
        // Each block of data is arranged like this:
        // [Xi]     [1, Ti,  0,  0,  0,  0] * [Bx]
        // [Yi]  =  [0,  0,  1, Ti,  0,  0]   [Mx]
        // [Zi]     [0,  0,  0,  0,  1, Ti]   [By]
        //  \/      \_____________________/   [My]
        //  Pi                 \/             [Bz]
        //                     Ai             [Mz]
        //                                     \/
        //                                   theta
        //
        // and then data is all stacked like this, where n is the buffer size:
        // [P1]     [A1] * [Bx]
        // [P2]  =  [A2]   [Mx]
        //  :        :     [By]
        // [Pn]     [An]   [My]
        //                 [Bz]
        //                 [Mz]

		firstUpdate = false;

        //Fill buffers
        timeBuff(timeInd) = newTime.toSec();
        stateBuff.segment(stateInd,stateSize) = newMeasure;
        
        //Increment index, roll back over
        timeInd = (timeInd+1)%bufferSize;
        stateInd = (stateInd + stateSize)%(stateSize*bufferSize);

        //If the index has rolled over once, the buffer is full
        if (timeInd == 0)
        {
            bufferFull = true;
        }

        Eigen::VectorXd stateDerivative = Eigen::VectorXd::Zero(stateSize,1);//initialize state derivative
        if (bufferFull)
        {
            // normalize time for numerical stability/accuracy of subsequent matrix inversion
            double delT = timeBuff.maxCoeff() - timeBuff.minCoeff();
            Eigen::VectorXd timeNorm = (timeBuff.array() - timeBuff.minCoeff())/delT;
		
			clock_t startTime = clock();
            // Solve LLS for best fit line parameters
            Eigen::MatrixXd stateA(stateSize*bufferSize,2*stateSize);
            for (int ii = 0; ii < bufferSize; ii++)
            {
				Eigen::MatrixXd newA = Eigen::MatrixXd::Zero(stateSize,2*stateSize);
				for (int jj = 0; jj < stateSize; jj++)
				{
					int thisColStart = 2*jj;
					newA.block(jj,thisColStart,1,2) << 1,timeNorm(ii);
				}
				
				stateA.block(ii*stateSize,0,stateSize,2*stateSize) = newA;
            }
            //ROS_INFO("time for here6 %3.7f",double(clock()-startTime)/CLOCKS_PER_SEC);
            startTime = clock();
            //Eigen::VectorXd theta = stateA.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(stateBuff);
			//Eigen::VectorXd theta = stateA.colPivHouseholderQr().solve(stateBuff);
			//Eigen::VectorXd theta = stateA.householderQr().solve(stateBuff);
			Eigen::MatrixXd ATA = stateA.transpose()*stateA;
			Eigen::MatrixXd ATB = stateA.transpose()*stateBuff;
			Eigen::VectorXd theta = ATA.ldlt().solve(ATB);
			//Eigen::VectorXd theta = ATA.llt().solve(ATB);
			
			
			
			//ROS_INFO("time for here7 %3.7f",double(clock()-startTime)/CLOCKS_PER_SEC);

			// Get state derivatives
            for (int ii = 0; ii < stateSize; ii++)
            {
				int oddElement = ii*2+1;
				stateDerivative(ii) = theta(oddElement)/delT;// rescaled to account for time normalization
			}
        }
        
        return stateDerivative;//return state derivative
    }

	//return the current index in the state
	bool isfirstUpdate()
	{
		return firstUpdate;
	}

	//return if the buffer is full indicating the estimate is good
	bool isbufferFull()
	{
		return bufferFull;
	}
	
	//reset the estimator
	void reset()
	{
		timeInd = 0;
        stateInd = 0;
        bufferFull = false;
        firstUpdate = true;
	}
};



class Predictor
{
	ros::NodeHandle nh;
    ros::Timer controlLoop;
    ros::Publisher predictorPosePub;
    ros::Publisher predictorTwistPub;
    ros::Publisher velCmdPub;
    
    geometry_msgs::Twist predictorTwist, switchedTwist;
	geometry_msgs::PoseStamped predictorPose,bebopPose, switchedPose;
    geometry_msgs::Twist uTwist;
    nav_msgs::Odometry odom;
    
    ros::Subscriber mocapSub, switchedPoseSub, switchedTwistSub, uSub;
    
    std::string bebopName;//namespace for the bebop
    Eigen::Vector3d e1,e2,k1,k2,dmax,e1_prev,e2_prev;
    Eigen::Vector3d statesP,statesHatP,statesDesP,statesHatL,statesHatA,desL,desA,e1P,e2P,rL,rA,uL,uA;
    Eigen::Vector4d statesQ,statesHatQ,statesDesQ,lastQ,e1Q,e2Q;
    
    DerivativeEstimator hatQdot;
    std::vector<double> k1_gain,k2_gain,d_max;
    bool sp,st,init,feedback,mocap;
    double loopRate,boundary_radius,feedback_radius;
    
    ros::Time time_init,time_last,time_last_mocap;
    
	public:
		Predictor()
		{
			// node handle
			ros::NodeHandle nhp("~");
			
			// parameters
			nhp.param<double>("loopRate",loopRate,300); //Hz
			nhp.param<double>("boundary_radius",boundary_radius,1);
			nhp.param<double>("feedback_radius",feedback_radius,1);
			nhp.param<std::vector<double>>("k1",k1_gain,{1,1,1}); 
			nhp.param<std::vector<double>>("k2",k2_gain,{3,3,3}); 
			nhp.param<std::vector<double>>("dmax",d_max,{0.05,0.05,0.05});
			nhp.param<std::string>("bebopName",bebopName,"bebop1");
			
			// parse gains and parameters
			k2 << k2_gain.at(0),k2_gain.at(1),k2_gain.at(2);
			k1 << k1_gain.at(0),k1_gain.at(1),k1_gain.at(2);
			dmax << d_max.at(0),d_max.at(1),d_max.at(2);
			
			// initialize states and controls
			statesHatL << 0,0,0;
			statesHatA << 0,0,0;
			uL << 0,0,0;
			uA << 0,0,0;
			rL << 0,0,0;
			rA << 0,0,0;
			
			e1_prev << 0,0,0;
			e2_prev << 0,0,0;			
			
			hatQdot = DerivativeEstimator(3,4);
			
			// set initial boolean
			sp = false;
			st = false;
			mocap = false;
			
			// publishers
			predictorTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/predictorTwist",1);
			predictorPosePub = nh.advertise<geometry_msgs::PoseStamped>(bebopName+"/predictorPose",1);
			velCmdPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/desTwist",1);
			
			// subscribers
			mocapSub = nh.subscribe(bebopName+"/mocapPose",1,&Predictor::mocapCB,this);
			switchedPoseSub = nh.subscribe(bebopName+"/switchedPose",1,&Predictor::switchedPoseCB,this);
			switchedTwistSub = nh.subscribe(bebopName+"/switchedTwist",1,&Predictor::switchedTwistCB,this);
			controlLoop = nh.createTimer(ros::Duration(1.0/loopRate),&Predictor::predictorCB,this);
			uSub = nh.subscribe("bebop1/odom",1,&Predictor::uCB,this);
			//feedbackSub = nh.subscribe(bebopName+"/feedback",1,&Predictor::feedbackCB,this);
			
			// set time
			time_init = ros::Time::now();
			time_last = time_init;
			time_last_mocap = time_init;
		}
		void mocapCB(const geometry_msgs::PoseStampedPtr& pose)
		{
			
			// Set current frame time
			ros::Time time_now = ros::Time::now();
			
			// Get dt
			double dt = (time_now - time_last_mocap).toSec();
			
			// Store time and states
			time_last_mocap = time_now;
			bebopPose = *pose;
			statesP << bebopPose.pose.position.x, bebopPose.pose.position.y, bebopPose.pose.position.z;
			statesQ << bebopPose.pose.orientation.w, bebopPose.pose.orientation.x, bebopPose.pose.orientation.y, bebopPose.pose.orientation.z;
	
			// First iteration
			if(!mocap)
			{	
				statesHatP = statesP;
				statesHatQ = statesQ;
				lastQ = statesQ;
				std::cout << "setting statesHatQ to statesQ" << std::endl;
				mocap = true;
			}

			if(sp && st) init = true;
			else return;
			
			if ((statesHatQ-(-1.0*statesQ)).norm() < (statesHatQ-statesQ).norm())//check if it flipped
			{
				statesQ *= -1.0;
			}
			
			feedback = checkFeedback(bebopPose);
			//feedback = true;
	
		
			e1P = statesHatP - statesDesP; 
			e2P = statesP - statesHatP;
			
			/*e1Q = (getqMat(getqInv(statesHatQ))*statesDesQ).segment(1,3).norm() < (getqMat(getqInv(-1.0*statesHatQ))*statesDesQ).segment(1,3).norm() ? 
					getqMat(getqInv(statesHatQ))*statesDesQ : getqMat(getqInv(-1.0*statesHatQ))*statesDesQ;
			e2Q = (getqMat(getqInv(statesQ))*statesHatQ).segment(1,3).norm() < (getqMat(getqInv(-1.0*statesQ))*statesHatQ).segment(1,3).norm() ? 
					getqMat(getqInv(statesQ))*statesHatQ : getqMat(getqInv(-1.0*statesQ))*statesHatQ;*/
			
			tf::Quaternion hatQ(statesHatQ(1),statesHatQ(2),statesHatQ(3),statesHatQ(0));  
			tf::Quaternion desQ(statesDesQ(1), statesDesQ(2), statesDesQ(3), statesDesQ(0));
			tf::Quaternion truQ(statesQ(1), statesQ(2), statesQ(3), statesQ(0));
				
			
			tf::Matrix3x3 hatM(hatQ);
			tf::Matrix3x3 desM(desQ);
			tf::Matrix3x3 truM(truQ);
			Eigen::Vector3d hatRPY;
			Eigen::Vector3d desRPY;
			Eigen::Vector3d truRPY;
			hatRPY << 0,0,0;
			desRPY << 0,0,0;
			truRPY << 0,0,0;
			hatM.getRPY(hatRPY(0),hatRPY(1),hatRPY(2));
			desM.getRPY(desRPY(0),desRPY(1),desRPY(2));
			truM.getRPY(truRPY(0),truRPY(1),truRPY(2));
						
			e1Q << hatRPY -  desRPY;
			e2Q << truRPY - hatRPY;
			
			
			//tf::Quaternion e1Qtf = desQ.inverse()*hatQ;
			//tf::Quaternion e2Qtf = hatQ.inverse()*truQ;
			//e1Q << e1Qtf.getW(),e1Qtf.getX(),e1Qtf.getY(),e1Qtf.getZ();
			//e2Q << e2Qtf.getW(),e2Qtf.getX(),e2Qtf.getY(),e2Qtf.getZ();
			

					
			// make sure theta tilde is between 0 and 2*pi
			double theta_tilde1 = fmod(e1Q(2),2.0*M_PI);
			double theta_tilde2 = fmod(e2Q(2),2.0*M_PI);
			
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
			
			e1Q(2) = theta_tilde1;
			e2Q(2) = theta_tilde2;
				
			
			if(feedback)
			{
				//rL = k2.cwiseProduct(e2P);
				//rA = k2.cwiseProduct(e2Q);
				
				//rL = k2.cwiseProduct(e2P);
				//rA = k2.cwiseProduct(e2Q.segment(1,3));
			}
			else
			{
				 rL << 0,0,0;
				 rA << 0,0,0;
			}
			
			tf::Quaternion swA_quat(switchedTwist.angular.x,switchedTwist.angular.y,switchedTwist.angular.z,0.0);
			swA_quat = hatQ*swA_quat*hatQ.inverse();
			
			Eigen::Vector3d swL(switchedTwist.linear.x,switchedTwist.linear.y,switchedTwist.linear.z);
			Eigen::Vector3d swA(swA_quat.getX(),swA_quat.getY(),swA_quat.getZ());

			uL = -k1.cwiseProduct(e1P) + swL - rL;
			uA = -2.0*k1.cwiseProduct(e1Q.segment(1,3)) + swA - rA;
										

			//std::cout << "u2: " << std::endl << swDot << std::endl;
			//std::cout << "rA: " << std::endl << rA << std::endl;
			//std::cout << "tf: " << std::endl << hatQ.getX() << std::endl;
			//std::cout << "e1Q: " << std::endl << e1Q << std::endl;
			//std::cout << "e2Q: " << std::endl << e2Q << std::endl;
			//std::cout << "uL: " << std::endl << uL << std::endl;
			//std::cout << "uA: " << std::endl << uA << std::endl;
			
			
			// Rotate to body frame
			Eigen::Vector3d bebopUL,bebopUA;
			
			if(feedback)
			{	
				bebopUL = (getqMat(getqInv(statesQ))*getqMat(Eigen::Vector4d(0.0,uL(0),uL(1),uL(2)))*statesQ).block(1,0,3,1);
				//bebopUA = (getqMat(getqInv(statesQ))*getqMat(Eigen::Vector4d(0.0,uA(0),uA(1),uA(2)))*statesQ).block(1,0,3,1);
			}
			else
			{
				bebopUL = (getqMat(getqInv(statesHatQ))*getqMat(Eigen::Vector4d(0.0,uL(0),uL(1),uL(2)))*statesHatQ).block(1,0,3,1);
				//bebopUA = (getqMat(getqInv(statesHatQ))*getqMat(Eigen::Vector4d(0.0,uA(0),uA(1),uA(2)))*statesHatQ).block(1,0,3,1);
			}


			geometry_msgs::Twist velCmd;
			velCmd.linear.x = bebopUL(0);
			velCmd.linear.y = bebopUL(1);
			velCmd.linear.z = bebopUL(2);
			velCmd.angular.x = bebopUA(0);
			velCmd.angular.y = bebopUA(1);
			velCmd.angular.z = bebopUA(2);
			velCmdPub.publish(velCmd);
			
			//std::cout << "velCmd: " << std::endl << velCmd << std::endl;
		
		}
		void uCB(const nav_msgs::Odometry::ConstPtr& odomPtr)
		{
			//std::cout << "get u" << std::endl;
			odom = *odomPtr;
			uTwist = odomPtr->twist.twist;
		}
		void predictorCB(const ros::TimerEvent& )
		{
			
			ros::Time time_now = ros::Time::now();
			double dt = (time_now - time_last).toSec();
			time_last = time_now;

			if(!mocap) return;
			
			// update statesHat using latest dt
			//tf::Quaternion hatQ(statesHatQ(1),statesHatQ(2),statesHatQ(3),statesHatQ(0));
			
			//tf::Matrix3x3 hatM(hatQ);
			//Eigen::Vector3d hatRPY;
			//hatM.getRPY(hatRPY(0),hatRPY(1),hatRPY(2));
			//tf::Quaternion newHatQ;
			//newHatQ.setRPY(hatRPY(0),hatRPY(1),hatRPY(2));
			//statesHatQ += statesHatA*dt;
			
			//tf::Quaternion Q(newOdomQ(1),newOdomQ(2),newOdomQ(3),newOdomQ(0));
			//tf::Matrix3x3 M(Q);
			//Eigen::Vector3d RPY;
			//M.getRPY(RPY(0),RPY(1),RPY(2));
						
			//Eigen::Vector3d uAa_world = hatQdot.update(RPY,time_now);
			
			//Eigen::MatrixXd Bq = getqDiff(statesHatQ);//get the differential matrix for new orientation
			//Eigen::Vector3d uAa_world = 2*(Bq.transpose())*uA_actual;//get the angular velocity in the body frame
			
			
			statesHatP += statesHatL*dt;
			
			tf::Quaternion hatQ(statesHatQ(1),statesHatQ(2),statesHatQ(3),statesHatQ(0));
			tf::Quaternion dQ;
			dQ.setRPY(statesHatA(0),statesHatA(1),statesHatA(2));
			tf::Quaternion newHatQ = dQ*hatQ;
			
			statesHatQ(0) = newHatQ.getW();
			statesHatQ(1) = newHatQ.getX();
			statesHatQ(2) = newHatQ.getY();
			statesHatQ(3) = newHatQ.getZ();
			statesHatQ.normalize();	
			
				
			Eigen::Vector4d uLa_body;
			uLa_body << 0.0,uTwist.linear.x,uTwist.linear.y,uTwist.linear.z;
			Eigen::Vector4d uLa_world = getqMat(getqMat(statesHatQ)*uLa_body)*getqInv(statesHatQ);
			

			Eigen::Vector4d newOdomQ(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);
			if(newOdomQ.norm()<0.5) return;
						
			if ((statesHatQ-(-1.0*newOdomQ)).norm() < (statesHatQ-newOdomQ).norm())//check if it flipped
			{
				newOdomQ *= -1.0;
			}
			
			// Update derivative estimator
			Eigen::Vector4d hatA_world = hatQdot.update(newOdomQ,time_now);
			
			// Convert to tf::Quaternion
			tf::Quaternion odomQ_quat(newOdomQ(1),newOdomQ(2),newOdomQ(3),newOdomQ(0));
			tf::Quaternion hatA_quat(hatA_world(1),hatA_world(2),hatA_world(3),hatA_world(0));


			Eigen::Vector4d debug;
			debug << hatQ.inverse().getW(),hatQ.inverse().getX(),hatQ.inverse().getY(),hatQ.inverse().getZ();
			std::cout << "debug : " <<  debug << std::endl;

			// Find body angular velocity w(t) = 2*(dq/dt)*q.conj(); q.conj() = q.inv() for unit quaternions
			tf::Quaternion uA_quat = (hatQ.inverse()*2.0)*hatA_quat;
			uA_quat.setW(0.0); // ignore scalar part
			//uA_quat.normalize(); // normalize since scalar part is now zero
		
			
			// Convert to world (inertial) frame
			//uA_quat = hatQ*uA_quat*hatQ.inverse();
			uA_quat.setW(0.0); // ignore scalar part
			//uA_quat.normalize(); // normalize since scalar part is now zero
			
			Eigen::Vector3d uAa_world;
			uAa_world << uA_quat.getX(),uA_quat.getY(),uA_quat.getZ();

			statesHatL = uLa_world.segment(1,3)+rL;
			statesHatA = uAa_world;//+rA;

			//std::cout << "uAa_world : " << uAa_world << std::endl;
			//std::cout << "statesHatA: " << std::endl << statesHatA << std::endl;
			//std::cout << "statesHatQ: " << std::endl << statesHatQ << std::endl;
			////std::cout << "uA: " << std::endl << uA << std::endl;
			//std::cout << "rA: " << std::endl << rA << std::endl;
		
			// publish predictorPose	
				
			predictorPose.header.stamp = time_now;
			predictorPose.header.frame_id = "world";
			predictorPose.pose.position.x = statesHatP(0);
			predictorPose.pose.position.y = statesHatP(1);
			predictorPose.pose.position.z = statesHatP(2);
			predictorPose.pose.orientation.x = statesHatQ(1);
			predictorPose.pose.orientation.y = statesHatQ(2);
			predictorPose.pose.orientation.z = statesHatQ(3);
			predictorPose.pose.orientation.w = statesHatQ(0);
		
			predictorPosePub.publish(predictorPose);
		
			// publish vd	
				
			predictorTwist.linear.x = statesHatL(0);
			predictorTwist.linear.y = statesHatL(1);
			predictorTwist.linear.z = statesHatL(2);
			predictorTwist.angular.x = statesHatA(0);
			predictorTwist.angular.y = statesHatA(1);
			predictorTwist.angular.z = statesHatA(2);
			
			predictorTwistPub.publish(predictorTwist);
			
		}
		void switchedPoseCB(const geometry_msgs::PoseStampedPtr& swiPose)
		{
			switchedPose = *swiPose;
			statesDesP << switchedPose.pose.position.x, switchedPose.pose.position.y, switchedPose.pose.position.z;
			statesDesQ << switchedPose.pose.orientation.w,switchedPose.pose.orientation.x,switchedPose.pose.orientation.y,switchedPose.pose.orientation.z;
			sp = true;
		}
		void switchedTwistCB(const geometry_msgs::TwistPtr& swiTwist)
		{
			switchedTwist = *swiTwist;
			desL << switchedTwist.linear.x,switchedTwist.linear.y,switchedTwist.linear.z;
			desA << switchedTwist.angular.x,switchedTwist.angular.y,switchedTwist.angular.z;
			st = true;
		}
		
		bool checkFeedback(geometry_msgs::PoseStamped pose)
		{
			if(pow(pose.pose.position.x,2) + pow(pose.pose.position.y,2) < feedback_radius) return true;
			else return false; 
		}
		Eigen::Vector3d sgn(Eigen::Vector3d e2)
		{
			return Eigen::Vector3d((e2(0) > 0) - (e2(0) < 0), (e2(1) > 0) - (e2(1) < 0), (e2(2) > 0) - (e2(2) < 0));
		}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Predictor_node");
    
    Predictor obj;
    
	ros::spin();
    return 0;
}

