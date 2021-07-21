#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

/***************************Copied Classes******************************/

// trapizoidal rule integral estimator of a state vector wrt time
class IntegralEstimator
{
	double Deltat;                           // time window of data to hold
	std::deque<ros::Time> timeBuffer;        // time data
	std::deque<Eigen::Vector3d> stateBuffer; // state data
	int stateSize;                           // size of the state
	
public:
	IntegralEstimator()
	{}
	
	IntegralEstimator(double DeltatInit, int stateSizeInit)
	{
		Deltat = DeltatInit;
		stateSize = stateSizeInit;
	}
	
	IntegralEstimator(const IntegralEstimator& integralEstimatorNew)
	{
		Deltat = integralEstimatorNew.Deltat;
		stateSize = integralEstimatorNew.stateSize;
	}
	
	IntegralEstimator operator =(const IntegralEstimator& integralEstimatorNew)
	{
		Deltat = integralEstimatorNew.Deltat;
		stateSize = integralEstimatorNew.stateSize;
		return *this;
	}
	
	// update the buffers with new data
	Eigen::Vector3d update(Eigen::Vector3d stateNew, ros::Time timeNew)
	{
		timeBuffer.push_back(timeNew);   // save the time
		stateBuffer.push_back(stateNew); // save the state
		
		Eigen::Vector3d stateBufferIntegral;                    // integral of the buffer
		stateBufferIntegral = Eigen::Vector3d::Zero(stateSize); // initialize to 0
		
		// use greater than 3 because trapezoidal rule for 2 data points doesnt make sense
		if (timeBuffer.size() >= 3)
		{
			// while the buffer is too big pop off the oldest data as long as it wont make 
			// the time on the buffer too small. compare with the second oldest data to ensure
			// the buffer stays large enough
			while ((timeBuffer.at(timeBuffer.size()-1) - timeBuffer.at(1)).toSec() >= Deltat)
			{
				timeBuffer.pop_front();
				stateBuffer.pop_front();
			}
			
			// if the buffer has enough time worth of data on it then calculate the 
			// integral of Y, and calculate the new Dx
			if ((timeBuffer.at(timeBuffer.size()-1) - timeBuffer.at(0)).toSec() >= Deltat)
			{
				for (int i = 0; i < timeBuffer.size()-1; i++)
				{
					stateBufferIntegral += 0.5*(timeBuffer.at(i+1) - timeBuffer.at(i)).toSec()*(stateBuffer.at(i+1) + stateBuffer.at(i));
				}
			}
		}
		
		return stateBufferIntegral;
	}
};

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

// PID controller for a state vector error
class PID
{
	double kP;
	double kD;
	double kI;
	int derivativeBufferSize;
	double integralBufferSize;
	DerivativeEstimator derivativeEstimator;
	IntegralEstimator integralEstimator;
	int stateSize;

public:
	PID()
	{}
	
	PID(double kPInit, double kDInit, double kIInit, int derivativeBufferSizeInit, double integralBufferSizeInit, int stateSizeInit)
	{
		kP = kPInit;
		kD = kDInit;
		kI = kIInit;
		stateSize = stateSizeInit;
		derivativeBufferSize = derivativeBufferSizeInit;
		integralBufferSize = integralBufferSizeInit;
		derivativeEstimator = DerivativeEstimator(derivativeBufferSize,stateSize);
		integralEstimator = IntegralEstimator(integralBufferSize,stateSize);
	}
	
	PID(const PID& pidNew)
	{
		kP = pidNew.kP;
		kD = pidNew.kD;
		kI = pidNew.kI;
		stateSize = pidNew.stateSize;
		derivativeBufferSize = pidNew.derivativeBufferSize;
		integralBufferSize = pidNew.integralBufferSize;
		derivativeEstimator = DerivativeEstimator(derivativeBufferSize,stateSize);
		integralEstimator = IntegralEstimator(integralBufferSize,stateSize);
	}
	
	PID operator =(const PID& pidNew)
	{
		kP = pidNew.kP;
		kD = pidNew.kD;
		kI = pidNew.kI;
		stateSize = pidNew.stateSize;
		derivativeBufferSize = pidNew.derivativeBufferSize;
		integralBufferSize = pidNew.integralBufferSize;
		derivativeEstimator = DerivativeEstimator(derivativeBufferSize,stateSize);
		integralEstimator = IntegralEstimator(integralBufferSize,stateSize);
		return *this;
	}
	
	Eigen::VectorXd update(Eigen::Vector3d errorNew, ros::Time timeNew)
	{
		Eigen::Matrix3d kScale;
		kScale << 1.0,  0,  0,
				    0,1.0,  0,
				    0,  0,0.5;
		Eigen::Vector3d kPu = kP*kScale*errorNew;
		Eigen::Vector3d kDu = kD*kScale*derivativeEstimator.update(errorNew, timeNew);
		Eigen::Vector3d kIu = kI*kScale*integralEstimator.update(errorNew, timeNew);
		return kPu+kDu+kIu;
	}
	
};

class VelocityCommand
{
	std::vector<double> linVelGains,angVelGains;
	int errorDerivativeBufferSize;
	double errorIntegralBufferSize;
	PID linVelPID;
	PID angVelPID;
	
public:
	VelocityCommand()
	{}

	VelocityCommand(std::vector<double> linVelGainsInit, std::vector<double> angVelGainsInit, int errorDerivativeBufferSizeInit, double errorIntegralBufferSizeInit)
	{
		linVelGains = linVelGainsInit;
		angVelGains = angVelGainsInit;
		errorDerivativeBufferSize = errorDerivativeBufferSizeInit;
		errorIntegralBufferSize = errorIntegralBufferSizeInit;
		linVelPID = PID(linVelGains.at(0),linVelGains.at(1),linVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		angVelPID = PID(angVelGains.at(0),angVelGains.at(1),angVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
	}
	
	VelocityCommand(const VelocityCommand& VelocityCommandN)
	{
		linVelGains = VelocityCommandN.linVelGains;
		angVelGains = VelocityCommandN.angVelGains;
		errorDerivativeBufferSize = VelocityCommandN.errorDerivativeBufferSize;
		errorIntegralBufferSize = VelocityCommandN.errorIntegralBufferSize;
		linVelPID = PID(linVelGains.at(0),linVelGains.at(1),linVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		angVelPID = PID(angVelGains.at(0),angVelGains.at(1),angVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
	}
	
	VelocityCommand operator=(const VelocityCommand& VelocityCommandN)
	{
		linVelGains = VelocityCommandN.linVelGains;
		angVelGains = VelocityCommandN.angVelGains;
		errorDerivativeBufferSize = VelocityCommandN.errorDerivativeBufferSize;
		errorIntegralBufferSize = VelocityCommandN.errorIntegralBufferSize;
		linVelPID = PID(linVelGains.at(0),linVelGains.at(1),linVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		angVelPID = PID(angVelGains.at(0),angVelGains.at(1),angVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		return *this;
	}
	
	geometry_msgs::Twist update(Eigen::Vector3d linVelError, Eigen::Vector3d angVelError)
	{
		Eigen::Vector3d linVelCmd = linVelPID.update(linVelError,ros::Time::now());
		Eigen::Vector3d angVelCmd = angVelPID.update(angVelError,ros::Time::now());
		geometry_msgs::Twist u;
		u.linear.x = linVelCmd(0);
		u.linear.y = linVelCmd(1);
		u.linear.z = linVelCmd(2);
		u.angular.x = angVelCmd(0);
		u.angular.y = angVelCmd(1);
		u.angular.z = angVelCmd(2);
		return u;
	}

	
};

class BebopControl
{
	ros::NodeHandle nh;
	ros::Subscriber udSub, uSub;
	ros::Publisher uPub;
	geometry_msgs::Twist udTwist;
	VelocityCommand velocityCommand;
	int errorDerivativeBufferSize;
	double errorIntegralBufferSize;
	std::vector<double> linVelGains,angVelGains;
		
public:
	
	BebopControl()
	{
		//initialize
		ros::NodeHandle nhp("~");
		nhp.param<std::vector<double>>("linVelGains",linVelGains,{0,0,0});
		nhp.param<std::vector<double>>("angVelGains",angVelGains,{0,0,0});
		nhp.param<int>("errorDerivativeBufferSize",errorDerivativeBufferSize,3);
		nhp.param<double>("errorIntegralBufferSize",errorIntegralBufferSize,0.5);
		udSub = nh.subscribe("bebop4/desTwist",1,&BebopControl::udCB,this);
		uSub = nh.subscribe("bebop4/odom",1,&BebopControl::uCB,this);
		uPub = nh.advertise<geometry_msgs::Twist>("bebop4/cmd_vel",1);  // previously turtlebot0/cmd_vel_mux/input/navi
		udTwist.linear.x = 0;
		udTwist.linear.y = 0;
		udTwist.linear.z = 0;
		udTwist.angular.x = 0;
		udTwist.angular.y = 0;
		udTwist.angular.z = 0;
		VelocityCommand velocityCommandNew(linVelGains, angVelGains, errorDerivativeBufferSize, errorIntegralBufferSize);
		velocityCommand = velocityCommandNew;
	}
	void udCB(const geometry_msgs::TwistPtr& udTwistPtr)
	{
		udTwist = *udTwistPtr;
	}
	
	void uCB(const nav_msgs::Odometry::ConstPtr& odomPtr)
	{
		//std::cout << "get u" << std::endl;
		geometry_msgs::Twist uTwist = odomPtr->twist.twist;
		double linx = udTwist.linear.x - uTwist.linear.x;
		double liny = udTwist.linear.y - uTwist.linear.y;
		double linz = udTwist.linear.z - uTwist.linear.z;
		double angx = udTwist.angular.x - uTwist.angular.x;
		double angy = udTwist.angular.y - uTwist.angular.y;
		double angz = udTwist.angular.z - uTwist.angular.z;
		Eigen::Vector3d linVelError(linx,liny,linz);
		Eigen::Vector3d angVelError(angx,angy,angz);
		geometry_msgs::Twist uError = velocityCommand.update(linVelError,angVelError);
		geometry_msgs::Twist ucmd;
		ucmd.linear.x = uError.linear.x;// + udTwist.linear.x;
		ucmd.linear.y = uError.linear.y;// + udTwist.linear.y;
		ucmd.linear.z = uError.linear.z;// + udTwist.linear.z;
		ucmd.angular.x = uError.angular.x;// + udTwist.angular.x;
		ucmd.angular.y = uError.angular.y;// + udTwist.angular.y;
		ucmd.angular.z = uError.angular.z;// + udTwist.angular.z;
		uPub.publish(ucmd);
	}
		
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bebop_control_node");
	
	BebopControl bebop_practice;
	ros::spin();
	return 0;
	
}
	


//class Bebop_control
//{
    //// ROS stuff
    //ros::NodeHandle nh;
    //ros::Publisher velCmdPub;
    //ros::Subscriber mocapSub_bbp,mocapSub_ttb, desPoseSub, desTwistSub;
	
    //// Topic prefix
    //std::string bebopName;//namespace for the bebop
    
    
    //// Parameters
	//std::vector<double> kv;//lin command gain
	//std::vector<double> kw;//ang command gain
    //// States
    //tf::Vector3 desPoseLin,ttbPoseLin;
    //tf::Quaternion desPoseAng, lastDesPoseAng;
    //geometry_msgs::TwistStamped desTwist;
    //bool autonomy;
    //ros::Time time_now,time_last,time_start;
    //tf::Vector3 mLin_last,mCmd;
    //tf::Transform lastPose;

//public:
    //Bebop_control()
    //{
        //// Initialize parameters
        //ros::NodeHandle nhp("~");
		//autonomy = false;
		//nhp.param<std::vector<double>>("kv",kv,{0.3,0.2,0.2});
		//double kvz = 100;
		//nhp.param<std::vector<double>>("kw",kw,{1});
		//nhp.param<std::string>("bebopName",bebopName,"NONE");
		//time_now = ros::Time::now();
		//time_last = ros::Time::now();
		//time_start = time_now;
		//std::cout << bebopName << std::endl;
        //// Initialize states
		//desPoseLin.setX(1.5);
		//desPoseLin.setY(0);
		//desPoseLin.setZ(1.0);
		//desPoseAng.setX(0);
		//desPoseAng.setY(0);
		//desPoseAng.setZ(0);
		//desPoseAng.setW(1);
		//ttbPoseLin = tf::Vector3(0,0,0);
		//mLin_last = tf::Vector3(0,0,0);
		//mCmd = tf::Vector3(0,0,0);
		//lastDesPoseAng = desPoseAng;
            
        //// Subscribers
        //mocapSub_bbp = nh.subscribe(bebopName+"/mocapPose",1,&Bebop_control::mocapCB,this);
        //desTwistSub = nh.subscribe(bebopName+"/desTwist",1,&Bebop_control::desTwistCB,this);
        //desPoseSub = nh.subscribe(bebopName+"/desPose",1,&Bebop_control::desPoseCB,this);
        
        //// Publishers
        //velCmdPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/cmd_vel",1);
        
    //}
    //void mocapCB(const geometry_msgs::PoseStampedConstPtr& pose)
    //{
		//time_now = ros::Time::now();
		//double posX = pose->pose.position.x;
		//double posY = pose->pose.position.y;
		//double posZ = pose->pose.position.z;
		//double mx = (desPoseLin.getX()-posX);
		//double my = (desPoseLin.getY()-posY);
		//double mz = (desPoseLin.getZ()-posZ);

		//tf::Vector3 mLin(mx,my,mz);
				
		//tf::Vector3 mDeriv = (mLin-mLin_last)/(time_now-time_last).toSec();
		
		//mCmd = kv.at(0)*mLin+kv.at(2)*mDeriv;
		
		
		
		
		////std::cout << kv.at(1) << std::endl;
		
		
		////tf::Vector3 bebop_x(1,0,0);
		////tf::Vector3 bebop_heading(mx,my,mz);
		////double angle = acos(bebop_x.dot(bebop_heading)/(bebop_x.length()*bebop_heading.length()));
		
		////std::cout << angle << std::endl;
		////tf::Quaternion q;
		////q.setRPY(0,0,angle);
		
		//tf::Transform tfPose, tfRot;
		//tfPose.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
		//tfPose.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w));
		
		//tfRot.setOrigin(tf::Vector3(0,0,0));
		//tfRot.setRotation(tfPose.getRotation());
		
		//tf::Quaternion rot(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
		
		
		
		//tf::Vector3 vLin = tfRot.inverse()*mCmd;
		
		//bool sat = false;
		
		//if(sat)
		//{
			//if(vLin.getX()>0.1) vLin.setX(0.1);
			//if(vLin.getY()>0.1) vLin.setY(0.1);
			//if(vLin.getZ()>0.1) vLin.setZ(0.1);
			//if(vLin.getX()<-0.1) vLin.setX(-0.1);
			//if(vLin.getY()<-0.1) vLin.setY(-0.1);
			//if(vLin.getZ()<-0.1) vLin.setZ(-0.1);
		//}
		
		
		//tf::Quaternion qTilde = desPoseAng.inverse()*rot;
		//tf::Quaternion wd = tf::Quaternion(desTwist.twist.angular.x,desTwist.twist.angular.y,desTwist.twist.angular.z,0);
		//tf::Quaternion vAng = qTilde*(-kw.at(0))+qTilde.inverse()*wd*qTilde;
				
		
		//updateVel(vLin,vAng);
		
		//bool debug = false;
		//if(debug)
		//{
			//std::cout << "mX: " << mx << std::endl;
			//std::cout << "mY: " << my << std::endl;
			//std::cout << "mZ: " << mz << std::endl << std::endl;
			
			//std::cout << "X: " << posX << std::endl;
			//std::cout << "Y: " << posY << std::endl;
			//std::cout << "Z: " << posZ << std::endl<<std::endl;
			//std::cout << "vX: " << vLin.getX() << std::endl;
			//std::cout << "vY: " << vLin.getY() << std::endl;
			//std::cout << "vZ: " << vLin.getZ() << std::endl << std::endl;
		//}
		//time_last = time_now;
		//mLin_last = mLin;
		//lastDesPoseAng = desPoseAng;
		//lastPose = tfPose;
	//}
	
	//void updateVel(tf::Vector3 vLin, tf::Quaternion w)// update this bebops velocity command
	//{
		//double vx = vLin.getX();
		//double vy = vLin.getY();
		//double vz = vLin.getZ();
		//double wx = w.getX();
		//double wy = w.getY();
		//double wz = w.getZ();
		
		//geometry_msgs::Twist velCmd;
		//velCmd.linear.x = vx;
		//velCmd.linear.y = vy;
		//velCmd.linear.z = vz;
		//velCmd.angular.x = wx;
		//velCmd.angular.y = wy;
		//velCmd.angular.z = wz;
		//velCmdPub.publish(velCmd);
	//}
	//void desPoseCB(const geometry_msgs::PoseStampedConstPtr& desPose)
	//{
		//desPoseLin.setX(desPose->pose.position.x);
		//desPoseLin.setY(desPose->pose.position.y);
		//desPoseLin.setZ(desPose->pose.position.z);
		//desPoseAng.setX(desPose->pose.orientation.x);
		//desPoseAng.setY(desPose->pose.orientation.y);
		//desPoseAng.setZ(desPose->pose.orientation.z);
		//desPoseAng.setW(desPose->pose.orientation.w);
		
	//}
	//void desTwistCB(const geometry_msgs::TwistStampedConstPtr& twist)
	//{
		//desTwist.twist.linear.x = twist->twist.linear.x;
		//desTwist.twist.linear.y = twist->twist.linear.y;
		//desTwist.twist.linear.z = twist->twist.linear.z;
		//desTwist.twist.angular.x = twist->twist.angular.x;
		//desTwist.twist.angular.y = twist->twist.angular.y;
		//desTwist.twist.angular.z = twist->twist.angular.z;
	//}
        
    
//}; // end Bebop_control

//int main(int argc, char** argv)
//{
    //ros::init(argc, argv, "Bebop_control");
    
    
    //Bebop_control obj;
    
    //ros::spin();
    //return 0;
//}

