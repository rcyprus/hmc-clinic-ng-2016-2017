/**
 * Subsriber to sensor topics (specifically the laser scanner and IMU)
 * 		Stores data in sensor structs, and posts to rostopic 'sensorData'
 * 		Deals with data parsing (acceleration -> velocity)
 * 
 * Austin Chun
 * Northrop Clinic Spring 2017
 * 
 *************************** 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/RCOut.h"
#include "mavros_msgs/State.h"

#include <math.h>
#include <RTIMULib.h>
#include <string>

//////// Might need to fix ///
#include "beginner_tutorials/SensorData.h"
// #include "beginner_tutorials/MotorInputs.h"

#define PI 3.14159265
#define LASER_OFFSET  0.05

class SensorMonitor{
   public:
	// Add whatever parameters the SSE would want/need
	
	struct laserScanner {
		double cur_dist; 
		double last_min;
		int cur_time;
		int last_time;
		double v_z;
	} laser0, laser1; 

	struct imu {
		int last_time;
		int cur_time;
		int duration;
		int numSamples;

		double ang_x; 
		double ang_y; 
		double ang_z; 

		double acc_x;
		double acc_y;
		double acc_z;

		double vel_x; 
		double vel_y; 
		double vel_z; 

		double v0_x;
		double v0_y;
		double v0_z;

		double roll; 
		double pitch; 
		double yaw; 

		double xScale;
		double yScale;
		double zScale;

	} imu1, imu2, imu3, imu4; 

	struct motorInputs {
		int numSamples;

		double motor1;
		double motor2;
		double motor3;
		double motor4;
	} u;

	double vz_1;
	double vz_2;
	double vz_3;
	double vz_4;

	std::string pixState;

	// Useful struct for converting Quaternion to Eulerian
   	struct eulerAngles {
   		double roll; 
   		double pitch;
   		double yaw; 
   	};

   	struct vector3{
   		double x;
   		double y;
   		double z;
   	};

   	// Flags for publishing (start as true, to initialize publishing)
   	bool gotlaser0;
   	bool gotlaser1;
   	bool justSentImu1;
   	bool justSentImu2;
   	bool justSentImu3;
   	bool justSentImu4;
   	bool justSentMotor;


	/* 
	* Constructor (initialize parameters
	*/
	SensorMonitor(){
		gotlaser0 = false;
	   	gotlaser1 = false;
	   	justSentImu1 = true;
	   	justSentImu2 = true;
	   	justSentImu3 = true;
	   	justSentImu4 = true;
	   	justSentMotor = true;


	   	imu1.v0_x = 0;
	   	imu1.v0_y = 0;
	   	imu1.v0_z = 0;

	   	imu2.v0_x = 0;
	   	imu2.v0_y = 0;
	   	imu2.v0_z = 0;

	   	imu3.v0_x = 0;
	   	imu3.v0_y = 0;
	   	imu3.v0_z = 0;

	   	imu4.v0_x = 0;
	   	imu4.v0_y = 0;
	   	imu4.v0_z = 0;

	   	// Scale Accelerometer data
	   	imu1.xScale = (0.966086 + 1.033639)/2.1;
	   	imu1.yScale = (0.988546 + 1.008764)/2.1;
	   	imu1.zScale = (1.050018 + 1.018457)/2.1;

	   	imu2.xScale = (1.031485 + 0.957328)/2.1;
	   	imu2.yScale = (1.002791 + 0.993021)/2.1;
	   	imu2.zScale = (1.038342 + 1.028129)/2.1;

	   	imu3.xScale = (0.98810  + 1.002416)/2.1;
	   	imu3.yScale = (0.983751 + 0.998607)/2.1;
	   	imu3.zScale = (1.051970 + 1.006303)/2.1;

	   	imu4.xScale = (1.023299 + 0.990078)/2.1;
	   	imu4.yScale = (0.977411 + 1.01935 )/2.1;
	   	imu4.zScale = (1.009940 + 1.064865)/2.1;

	};

	/*
	* Callback functions are executed whenever a new message is published to a rostopic
	* Currently used to store/calculate the important values into sensor structs
	*/

	/**
	 * @brief       stores the minimum distance measured between -10 and 10 degrees
	 * 				Also, calculates the z-velocity
	 *
	 */	
	void laserScan0Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		int i; 
		//int numScans = 682;  // determined experimentally 

		int startAng = 286; // 110 deg / 0.36 deg/scan starting index ~10deg
		int endAng = 396; // ~10 deg from vertical


		double curMin = 100.0;  // arbitrary large value	

		// Loop to find the minimum
		for(i = startAng; i < endAng; ++i){
			// Uses a minimum threshold
			if(msg->ranges[i] < curMin && msg->ranges[i] > msg->range_min){
				curMin = msg->ranges[i]; 
			}
		}

		// Store minimum distance 
		laser0.cur_dist = curMin;
		laser0.cur_time = msg->header.stamp.nsec;

		// Set flag
		gotlaser0 = true;
	}
	void laserScan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		int i; 
		//int numScans = 682;  // determined experimentally 

		int startAng = 286; // 110 deg / 0.36 deg/scan starting index ~10deg
		int endAng = 396; // ~10 deg from vertical


		double curMin = 100.0;  // arbitrary large value	

		// Loop to find the minimum
		for(i = startAng; i < endAng; ++i){
			// Uses a minimum threshold
			if(msg->ranges[i] < curMin && msg->ranges[i] > msg->range_min){
				curMin = msg->ranges[i]; 
			}
		}


		// Store minimum distance 
		laser1.cur_dist = curMin;
		laser1.cur_time = msg->header.stamp.nsec;

		// Set flag
		gotlaser1 = true;
	}
	
	/**
	 * @brief      IMU callback, stores time, accel (vel), gyro, and angle*
	 *
	 * @note 		angles determined using Quaternion-to-Euler calculation
	 */
	void Imu1Callback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		// Reset accumulators
		if(justSentImu1){
			// Set zeros 
			imu1.last_time = msg->header.stamp.nsec;
			imu1.duration = 0;
			imu1.numSamples = 0;

			imu1.acc_x = 0;
			imu1.acc_y = 0;
			imu1.acc_z = 0;
			imu1.ang_x = 0;
			imu1.ang_y = 0;
			imu1.ang_z = 0;
			imu1.roll = 0;
			imu1.pitch = 0;
			imu1.yaw = 0;
			imu1.vel_x = 0;
			imu1.vel_y = 0;
			imu1.vel_z = 0;

			justSentImu1 = false;
		}

		// Time
		imu1.cur_time = msg->header.stamp.nsec; 
		imu1.duration += (imu1.cur_time - imu1.last_time)%(int)10e9 ; 
		imu1.last_time = imu1.cur_time;

		imu1.numSamples ++;

		// Angular Rates
		imu1.ang_x += msg->angular_velocity.x ;
		imu1.ang_y += msg->angular_velocity.y ;
		imu1.ang_z += msg->angular_velocity.z ;

		// Angles given in Quaternions
		double qx = msg->orientation.x ;
		double qy = msg->orientation.y ;
		double qz = msg->orientation.z ;
		double qw = msg->orientation.w ;

		// Normalize
		double mag = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
		qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

		// Linear Acceleration
		vector3 g = rtGravity(qw,qx,qy,qz);
		imu1.acc_x += (msg->linear_acceleration.x * imu1.xScale) - g.x;
		imu1.acc_y += (msg->linear_acceleration.y * imu1.yScale) - g.y;
		imu1.acc_z += (msg->linear_acceleration.z * imu1.zScale ) - g.z;

		// Convert quarternian to eulerian (and store in IMU)
		eulerAngles angles = toEulerianAngle(qx, qy, qz, qw) ;
		imu1.roll += angles.roll ;
		imu1.pitch += angles.pitch ;
		imu1.yaw += angles.yaw ;
	}
	void Imu2Callback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		// Reset accumulators
		if(justSentImu2){
			// Set zeros 
			imu2.last_time = msg->header.stamp.nsec;
			imu2.duration = 0;
			imu2.numSamples = 0;

			imu2.acc_x = 0;
			imu2.acc_y = 0;
			imu2.acc_z = 0;
			imu2.ang_x = 0;
			imu2.ang_y = 0;
			imu2.ang_z = 0;
			imu2.roll = 0;
			imu2.pitch = 0;
			imu2.yaw = 0;
			imu2.vel_x = 0;
			imu2.vel_y = 0;
			imu2.vel_z = 0;

			justSentImu2 = false;
		}

		// Time
		imu2.cur_time = msg->header.stamp.nsec; 
		imu2.duration += (imu2.cur_time - imu2.last_time)%(int)10e9 ; 
		imu2.last_time = imu2.cur_time;

		imu2.numSamples ++;

		// Angular Rates
		imu2.ang_x += msg->angular_velocity.x ;
		imu2.ang_y += msg->angular_velocity.y ;
		imu2.ang_z += msg->angular_velocity.z ;

		// Angles given in Quaternions
		double qx = msg->orientation.x ;
		double qy = msg->orientation.y ;
		double qz = msg->orientation.z ;
		double qw = msg->orientation.w ;

		// Normalize
		double mag = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
		qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

		// Linear Acceleration
		vector3 g = rtGravity(qw,qx,qy,qz);
		imu2.acc_x += (msg->linear_acceleration.x * imu2.xScale) - g.x;
		imu2.acc_y += (msg->linear_acceleration.y * imu2.yScale) - g.y;
		imu2.acc_z += (msg->linear_acceleration.z * imu2.zScale ) - g.z;

		// Convert quarternian to eulerian (and store in IMU)
		eulerAngles angles = toEulerianAngle(qx, qy, qz, qw) ;
		imu2.roll += angles.roll ;
		imu2.pitch += angles.pitch ;
		imu2.yaw += angles.yaw ;
	}
	void Imu3Callback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		// Reset accumulators
		if(justSentImu3){
			// Set zeros 
			imu3.last_time = msg->header.stamp.nsec;
			imu3.duration = 0;
			imu3.numSamples = 0;

			imu3.acc_x = 0;
			imu3.acc_y = 0;
			imu3.acc_z = 0;
			imu3.ang_x = 0;
			imu3.ang_y = 0;
			imu3.ang_z = 0;
			imu3.roll = 0;
			imu3.pitch = 0;
			imu3.yaw = 0;
			imu3.vel_x = 0;
			imu3.vel_y = 0;
			imu3.vel_z = 0;

			justSentImu3 = false;
		}

		// Time
		imu3.cur_time = msg->header.stamp.nsec; 
		imu3.duration += (imu3.cur_time - imu3.last_time)%(int)10e9 ; 
		imu3.last_time = imu3.cur_time;

		imu3.numSamples ++;

		// Angular Rates
		imu3.ang_x += msg->angular_velocity.x ;
		imu3.ang_y += msg->angular_velocity.y ;
		imu3.ang_z += msg->angular_velocity.z ;

		// Angles given in Quaternions
		double qx = msg->orientation.x ;
		double qy = msg->orientation.y ;
		double qz = msg->orientation.z ;
		double qw = msg->orientation.w ;

		// Normalize
		double mag = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
		qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

		// Linear Acceleration
		vector3 g = rtGravity(qw,qx,qy,qz);
		imu3.acc_x += (msg->linear_acceleration.x * imu3.xScale) - g.x;
		imu3.acc_y += (msg->linear_acceleration.y * imu3.yScale) - g.y;
		imu3.acc_z += (msg->linear_acceleration.z * imu3.zScale ) - g.z;

		// Convert quarternian to eulerian (and store in IMU)
		eulerAngles angles = toEulerianAngle(qx, qy, qz, qw) ;
		imu3.roll += angles.roll ;
		imu3.pitch += angles.pitch ;
		imu3.yaw += angles.yaw ;
	}
	void Imu4Callback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		// Reset accumulators
		if(justSentImu4){
			// Set zeros 
			imu4.last_time = msg->header.stamp.nsec;
			imu4.duration = 0;
			imu4.numSamples = 0;

			imu4.acc_x = 0;
			imu4.acc_y = 0;
			imu4.acc_z = 0;
			imu4.ang_x = 0;
			imu4.ang_y = 0;
			imu4.ang_z = 0;
			imu4.roll = 0;
			imu4.pitch = 0;
			imu4.yaw = 0;
			imu4.vel_x = 0;
			imu4.vel_y = 0;
			imu4.vel_z = 0;

			justSentImu4 = false;
		}

		// Time
		imu4.cur_time = msg->header.stamp.nsec; 
		imu4.duration += (imu4.cur_time - imu4.last_time)%(int)10e9 ; 
		imu4.last_time = imu4.cur_time;

		imu4.numSamples ++;

		// Angular Rates
		imu4.ang_x += msg->angular_velocity.x ;
		imu4.ang_y += msg->angular_velocity.y ;
		imu4.ang_z += msg->angular_velocity.z ;

		// Angles given in Quaternions
		double qx = msg->orientation.x ;
		double qy = msg->orientation.y ;
		double qz = msg->orientation.z ;
		double qw = msg->orientation.w ;

		// Normalize
		double mag = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
		qx = qx/mag; qy = qy/mag; qz = qz/mag; qw = qw/mag;

		// Linear Acceleration
		vector3 g = rtGravity(qw,qx,qy,qz);
		imu4.acc_x += (msg->linear_acceleration.x * imu4.xScale) - g.x;
		imu4.acc_y += (msg->linear_acceleration.y * imu4.yScale) - g.y;
		imu4.acc_z += (msg->linear_acceleration.z * imu4.zScale ) - g.z;

		// Convert quarternian to eulerian (and store in IMU)
		eulerAngles angles = toEulerianAngle(qx, qy, qz, qw) ;
		imu4.roll += angles.roll ;
		imu4.pitch += angles.pitch ;
		imu4.yaw += angles.yaw ;
	}

	void motorInputsCallback(const mavros_msgs::RCOut::ConstPtr& msg)
	{
		// Reset all values when old values have been published out
		if(justSentMotor){
			u.numSamples = 0;
			u.motor1 = 0;
			u.motor2 = 0;
			u.motor3 = 0;
			u.motor4 = 0;
			justSentMotor = false;
		}
		// Update all values
		u.numSamples++;
		u.motor1 += msg->channels[0];
		u.motor2 += msg->channels[1];
		u.motor3 += msg->channels[2];
		u.motor4 += msg->channels[3];
	}

	void pixStateCallback(const mavros_msgs::State::ConstPtr& msg) { pixState = msg->mode; }


	/**
	 * @brief      	Calculates the average values for the 
	 * 			   	velocities of the IMU's, and the average
	 * 			   	motor input values
	 */
	void calculateAverages()
	{

	// Find delta T
		double deltaT_1 = imu1.duration * 10e-9;
		double deltaT_2 = imu2.duration * 10e-9;
		double deltaT_3 = imu3.duration * 10e-9;
		double deltaT_4 = imu4.duration * 10e-9;

	// Linear Velocity (Acceleration)
		imu1.acc_x = 9.81* imu1.acc_x / imu1.numSamples;
		imu2.acc_x = 9.81* imu2.acc_x / imu2.numSamples;
		imu3.acc_x = 9.81* imu3.acc_x / imu3.numSamples;
		imu4.acc_x = 9.81* imu4.acc_x / imu4.numSamples;

		// ROS_INFO("imu1.acc_x %lf",imu1.acc_x);

		imu1.vel_x = imu1.acc_x * deltaT_1 + imu1.v0_x;
		imu2.vel_x = imu2.acc_x * deltaT_2 + imu2.v0_x;
		imu3.vel_x = imu3.acc_x * deltaT_3 + imu3.v0_x;
		imu4.vel_x = imu4.acc_x * deltaT_4 + imu4.v0_x;

		imu1.v0_x = imu1.vel_x;
		imu2.v0_x = imu2.vel_x;
		imu3.v0_x = imu3.vel_x;
		imu4.v0_x = imu4.vel_x;


		imu1.acc_y = 9.81* imu1.acc_y / imu1.numSamples;
		imu2.acc_y = 9.81* imu2.acc_y / imu2.numSamples;
		imu3.acc_y = 9.81* imu3.acc_y / imu3.numSamples;
		imu4.acc_y = 9.81* imu4.acc_y / imu4.numSamples;

		// ROS_INFO("imu1.acc_y %lf",imu1.acc_y);

		imu1.vel_y = imu1.acc_y * deltaT_1 + imu1.v0_y;
		imu2.vel_y = imu2.acc_y * deltaT_2 + imu2.v0_y;
		imu3.vel_y = imu3.acc_y * deltaT_3 + imu3.v0_y;
		imu4.vel_y = imu4.acc_y * deltaT_4 + imu4.v0_y;

		imu1.v0_y = imu1.vel_y;
		imu2.v0_y = imu2.vel_y;
		imu3.v0_y = imu3.vel_y;
		imu4.v0_y = imu4.vel_y;

	// Calcuate Z velocity
		// Calculate AVG Accelerations
		imu1.acc_z = 9.81* (imu1.acc_z / imu1.numSamples);
		imu2.acc_z = 9.81* (imu2.acc_z / imu2.numSamples);
		imu3.acc_z = 9.81* (imu3.acc_z / imu3.numSamples);
		imu4.acc_z = 9.81* (imu4.acc_z / imu4.numSamples);

		// Integrate for IMU velocity
		imu1.vel_z  = imu1.acc_z * deltaT_1 + imu1.v0_z;
		imu2.vel_z  = imu2.acc_z * deltaT_2 + imu2.v0_z;
		imu3.vel_z  = imu3.acc_z * deltaT_3 + imu3.v0_z;
		imu4.vel_z  = imu4.acc_z * deltaT_4 + imu4.v0_z;

		// Set Laser Scanner velocities
		vz_1 = laser0.v_z;
		vz_2 = laser1.v_z;
		// average IMU velocities
		vz_3 = (imu1.vel_z + imu2.vel_z) / 2;
		vz_4 = (imu3.vel_z + imu4.vel_z) / 2;

		// Update z
		imu1.v0_z = imu1.vel_z;
		imu2.v0_z = imu2.vel_z;
		imu3.v0_z = imu3.vel_z;
		imu4.v0_z = imu4.vel_z;

	// Angular Velocity
		imu1.ang_x = imu1.ang_x / imu1.numSamples;
		imu2.ang_x = imu2.ang_x / imu2.numSamples;
		imu3.ang_x = imu3.ang_x / imu3.numSamples;
		imu4.ang_x = imu4.ang_x / imu4.numSamples;

		imu1.ang_y = imu1.ang_y / imu1.numSamples;
		imu2.ang_y = imu2.ang_y / imu2.numSamples;
		imu3.ang_y = imu3.ang_y / imu3.numSamples;
		imu4.ang_y = imu4.ang_y / imu4.numSamples;

		imu1.ang_z = imu1.ang_z / imu1.numSamples;
		imu2.ang_z = imu2.ang_z / imu2.numSamples;
		imu3.ang_z = imu3.ang_z / imu3.numSamples;
		imu4.ang_z = imu4.ang_z / imu4.numSamples;

		imu1.roll = imu1.roll / imu1.numSamples;
		imu2.roll = imu2.roll / imu2.numSamples;
		imu3.roll = imu3.roll / imu3.numSamples;
		imu4.roll = imu4.roll / imu4.numSamples;

		imu1.pitch = imu1.pitch / imu1.numSamples;
		imu2.pitch = imu2.pitch / imu2.numSamples;
		imu3.pitch = imu3.pitch / imu3.numSamples;
		imu4.pitch = imu4.pitch / imu4.numSamples;

		imu1.yaw = imu1.yaw / imu1.numSamples;
		imu2.yaw = imu2.yaw / imu2.numSamples;
		imu3.yaw = imu3.yaw / imu3.numSamples;
		imu4.yaw = imu4.yaw / imu4.numSamples;

	// Motor Inputs
	
		// Deals with slow motors, only update if motors recently called
		if(justSentMotor == false){
			// Use Calibration curve, then convert from RPM to rad/s
			u.motor1 = (u.motor1 / u.numSamples * 19.61 - 19022) * 0.10471975511965999;
			u.motor2 = (u.motor2 / u.numSamples * 19.61 - 19022) * 0.10471975511965999;
			u.motor3 = (u.motor3 / u.numSamples * 19.61 - 19022) * 0.10471975511965999;
			u.motor4 = (u.motor4 / u.numSamples * 19.61 - 19022) * 0.10471975511965999;

			// printf("u.numSamples: %d\n",u.numSamples);

			// Square the values
			u.motor1 = u.motor1*u.motor1;
			u.motor2 = u.motor2*u.motor2;
			u.motor3 = u.motor3*u.motor3;
			u.motor4 = u.motor4*u.motor4;
		}
		// If havent recieved new motor yet, keep old value
		else{
			// do nothing
		}



		// printf("u.motors: %lf\t %lf\t %lf\t %lf \n",u.motor1, u.motor2, u.motor3, u.motor4);

	// Laser Scans
	double avgRoll = (imu1.roll + imu2.roll + imu3.roll + imu4.roll) / 4.0;
	if(gotlaser0){
		// Store minimum distance 
		laser0.cur_dist = (laser0.cur_dist - LASER_OFFSET)*cos(avgRoll);

		// Numerically differentiate
		// Calculate time with nanoseconds, convert to seconds, and use modulo to deal with negative values
		laser0.v_z = (laser0.cur_dist - laser0.last_min) / ( ( (laser0.cur_time - laser0.last_time)%(int)pow(10,9)) *pow(10,-9) );
		
		// Store current data for next iteration
		laser0.last_min = laser0.cur_dist;
		laser0.last_time = laser0.cur_time;

	}
	if(gotlaser1){
		// Store minimum distance 
		laser1.cur_dist = (laser1.cur_dist - LASER_OFFSET)*cos(avgRoll);

		// Numerically differentiate
		// Calculate time with nanoseconds, convert to seconds, and use modulo to deal with negative values
		laser1.v_z = (laser1.cur_dist - laser1.last_min) / ( ( (laser1.cur_time - laser1.last_time)%(int)pow(10,9)) *pow(10,-9) );
		
		// Store current data for next iteration
		laser1.last_min = laser1.cur_dist;
		laser1.last_time = laser1.cur_time;

	}

	}
	
	/**
	 * @brief      Converts Quaternion angles to Eulerian angles
	 *
	 * source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	 *
	 * @return     struct of angles
	 */
	eulerAngles toEulerianAngle(double qx, double qy, double qz, double qw) 
	{
		// Initialize result
		eulerAngles angles ;
 
		double ysqr = qy * qy ;

		// roll (x-axis rotation)
		double t0 = +2.0 * (qw * qx + qy * qz) ;
		double t1 = +1.0 - 2.0 * (qx * qx + ysqr) ;
		angles.pitch = std::atan2(t0, t1) ;//*180/PI;

		// pitch (y-axis rotation)
		double t2 = +2.0 * (qw * qy - qz * qx) ;
		t2 = t2 > 1.0 ? 1.0 : t2 ;
		t2 = t2 < -1.0 ? -1.0 : t2 ;
		angles.roll = -std::asin(t2) ; //*180/PI;

		// yaw (z-axis rotation)
		double t3 = +2.0 * (qw * qz + qx * qy) ;
		double t4 = +1.0 - 2.0 * (ysqr + qz * qz)  ; 
		// angles.yaw = fmod( std::atan2(t3, t4) *180/PI + 270, 360.0) - 180;																									
		angles.yaw = fmod( std::atan2(t3, t4) + 3*PI/2, 2*PI) - PI;
		return angles ;
	}

	
	vector3 rtGravity(double w, double x, double y, double z)
	{
		RTQuaternion rotatedGravity;
	    RTQuaternion fusedConjugate;
	    RTQuaternion qTemp;
	    RTVector3 residuals;

	    vector3 result;

	    // Construc the quaternion
	    RTQuaternion fusionQPose = RTQuaternion(w, x, y, z);
	    RTQuaternion gravity = RTQuaternion(0, 0, 0, 1);
	    
	    fusedConjugate = fusionQPose.conjugate();

	    qTemp = gravity * fusionQPose;
	    rotatedGravity = fusedConjugate * qTemp;

	    result.x = rotatedGravity.x();
	    result.y = rotatedGravity.y();
	    result.z = rotatedGravity.z();

	    return result;
	}



}; 




int main(int argc, char **argv)
{
	ROS_INFO("Starting Subsriber");

	// Initialize the node named "subscriber"
	ros::init(argc, argv, "subscriber") ;

	// Create Node Handle
	ros::NodeHandle n ;

	// Create the SensorMonitor
	SensorMonitor mon ;

	// Subscribe to topics
	ros::Subscriber laser0 = n.subscribe("scan_laser0", 100, &SensorMonitor::laserScan0Callback, &mon) ;
	ros::Subscriber laser1 = n.subscribe("scan_laser1", 100, &SensorMonitor::laserScan1Callback, &mon) ;
	ros::Subscriber imu1 = n.subscribe("imu_data_0_28", 100, &SensorMonitor::Imu1Callback, &mon) ;
	ros::Subscriber imu2 = n.subscribe("imu_data_0_29", 100, &SensorMonitor::Imu2Callback, &mon) ;
	ros::Subscriber imu3 = n.subscribe("imu_data_1_28", 100, &SensorMonitor::Imu3Callback, &mon) ;
	ros::Subscriber imu4 = n.subscribe("imu_data_1_29", 100, &SensorMonitor::Imu4Callback, &mon) ;
	ros::Subscriber motorInputs = n.subscribe("/mavros/rc/out",100,&SensorMonitor::motorInputsCallback, &mon);
	ros::Subscriber pixhawkState = n.subscribe("/mavros/state",100,&SensorMonitor::pixStateCallback, &mon);

	// Publish to topic
	ros::Publisher sensorData_pub = n.advertise<beginner_tutorials::SensorData>("sensor_data", 1000) ;


	// Set frequency = 10 Hz update rate
	ros::Rate loop_rate(100) ; 

	// Initialize msg to publish
	beginner_tutorials::SensorData sensorData ;

	// usleep(1000000);

	// ROS_INFO("Publishing data");

	// Loop until break (ctrl+z)	
	while(ros::ok){
		
		// Don't start publishing until in OFFBOARD mode
		if(mon.pixState != "OFFBOARD"){
			ROS_INFO("Waiting for OFFBOARD mode");
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}

		// Create custom message, and publish when laserScanners publish
		if(mon.gotlaser0){ //&& mon.gotlaser1){
			ROS_INFO("Published data");

			// Calculate averages
			mon.calculateAverages();
		// Create message
			sensorData.header.stamp = ros::Time::now();
			sensorData.header.frame_id = "/world";

			sensorData.laser0_min_dist = mon.laser0.cur_dist;  
			sensorData.laser1_min_dist = mon.laser1.cur_dist;

			sensorData.imu1_ang_x = mon.imu1.ang_x; 
			sensorData.imu1_ang_y = mon.imu1.ang_y;
			sensorData.imu1_ang_z = mon.imu1.ang_z; 
			sensorData.imu1_vel_x = mon.imu1.vel_x;
			sensorData.imu1_vel_y = mon.imu1.vel_y;
			sensorData.imu1_roll  = mon.imu1.roll;
			sensorData.imu1_pitch = mon.imu1.pitch;
			sensorData.imu1_yaw   = mon.imu1.yaw; 

			sensorData.imu2_ang_x = mon.imu2.ang_x; 
			sensorData.imu2_ang_y = mon.imu2.ang_y;
			sensorData.imu2_ang_z = mon.imu2.ang_z; 
			sensorData.imu2_vel_x = mon.imu2.vel_x;
			sensorData.imu2_vel_y = mon.imu2.vel_y;
			sensorData.imu2_roll  = mon.imu2.roll;
			sensorData.imu2_pitch = mon.imu2.pitch;
			sensorData.imu2_yaw   = mon.imu2.yaw; 

			sensorData.imu3_ang_x = mon.imu3.ang_x; 
			sensorData.imu3_ang_y = mon.imu3.ang_y;
			sensorData.imu3_ang_z = mon.imu3.ang_z; 
			sensorData.imu3_vel_x = mon.imu3.vel_x;
			sensorData.imu3_vel_y = mon.imu3.vel_y;
			sensorData.imu3_roll  = mon.imu3.roll;
			sensorData.imu3_pitch = mon.imu3.pitch;
			sensorData.imu3_yaw   = mon.imu3.yaw; 

			sensorData.imu4_ang_x = mon.imu4.ang_x; 
			sensorData.imu4_ang_y = mon.imu4.ang_y;
			sensorData.imu4_ang_z = mon.imu4.ang_z; 
			sensorData.imu4_vel_x = mon.imu4.vel_x;
			sensorData.imu4_vel_y = mon.imu4.vel_y;
			sensorData.imu4_roll  = mon.imu4.roll;
			sensorData.imu4_pitch = mon.imu4.pitch;
			sensorData.imu4_yaw   = mon.imu4.yaw; 

			sensorData.vz_1 = mon.vz_1; // Laser scanner 0 velocity
			sensorData.vz_2 = mon.vz_2; // Laser scanner 1 velocity
			sensorData.vz_3 = mon.vz_3; // IMU 1&2 velocity
			sensorData.vz_4 = mon.vz_4; // IMU 3&4 velocity

			sensorData.u1 = mon.u.motor1;
			sensorData.u2 = mon.u.motor2;
			sensorData.u3 = mon.u.motor3;
			sensorData.u4 = mon.u.motor4;

		// Publish the data!
			sensorData_pub.publish(sensorData);

			// Reset flags
			mon.gotlaser0 = false; 
			mon.gotlaser1 = false;
			mon.justSentImu1 = true; 
			mon.justSentImu2 = true; 
			mon.justSentImu3 = true; 
			mon.justSentImu4 = true;
			mon.justSentMotor = true;
		}

		// Some sort of delay
		ros::spinOnce();  // gives time for the callbacks to execute
		loop_rate.sleep();  // Sleep for the remaining time to hit 10Hz
	}
	

	ROS_INFO("Ending Subscriber");

	return 0; 
}



