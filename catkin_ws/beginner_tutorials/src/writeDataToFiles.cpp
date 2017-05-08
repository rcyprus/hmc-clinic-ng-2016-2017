/**
 * Writes data to files
 * Use:
 * 	record a rosbag (subscribed to /sensor_data)
 * 	run this node
 * 	play rosbag back
 * 	(This will parse all the data, and save it to a txt file
 * 
 * Austin Chun
 * 
 *********************************
 * TODO (list)
 *
 *************************** 
 */

#include "ros/ros.h"
#include "beginner_tutorials/SensorData.h"
#include <iostream>
#include <fstream>
#include <string>

#include <sstream>

std::string ToString(double val)
{
    std::ostringstream stream;
    stream << val;
    return stream.str();
}

std::string intToString(int val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

void dataCallback(const beginner_tutorials::SensorData::ConstPtr& msg)
{
	// Helpful Printing
	ROS_INFO("Called: ");
	ROS_INFO("Seq: %d", msg->header.seq);
	ROS_INFO("Time: %d.%d", msg->header.stamp.sec, msg->header.stamp.nsec);

// Write Position File
	std::ofstream posFile;
	posFile.open ("posData.txt", std::ios_base::app);
	posFile << ToString(msg->laser0_min_dist) << ","  << ToString(msg->laser1_min_dist) << "," 
		<< ToString(msg->imu1_vel_x) << "," << ToString(msg->imu1_vel_y) << "," << ToString(msg->vz_1) << ","
		<< ToString(msg->imu2_vel_x) << "," << ToString(msg->imu2_vel_y) << "," << ToString(msg->vz_2) << ","
		<< ToString(msg->imu3_vel_x) << "," << ToString(msg->imu3_vel_y) << "," << ToString(msg->vz_3) << ","
		<< ToString(msg->imu4_vel_x) << "," << ToString(msg->imu4_vel_y) << "," << ToString(msg->vz_4) << "\n";
	posFile.close();

// Write Orientation File
	std::ofstream angFile;
	angFile.open ("angData.txt", std::ios_base::app);	
	angFile 
		<< ToString(msg->imu1_roll) << "," << ToString(msg->imu1_pitch) << "," << ToString(msg->imu1_yaw) << "," 
		<< ToString(msg->imu1_ang_x) << "," << ToString(msg->imu1_ang_y) << "," << ToString(msg->imu1_ang_z) << ","

		<< ToString(msg->imu2_roll) << "," << ToString(msg->imu2_pitch) << "," << ToString(msg->imu2_yaw) << "," 
		<< ToString(msg->imu2_ang_x) << "," << ToString(msg->imu2_ang_y) << "," << ToString(msg->imu2_ang_z) << ","
	
		<< ToString(msg->imu3_roll) << "," << ToString(msg->imu3_pitch) << "," << ToString(msg->imu3_yaw) << "," 
		<< ToString(msg->imu3_ang_x) << "," << ToString(msg->imu3_ang_y) << "," << ToString(msg->imu3_ang_z) << ","

		<< ToString(msg->imu4_roll) << "," << ToString(msg->imu4_pitch) << "," << ToString(msg->imu4_yaw) << ","
		<< ToString(msg->imu4_ang_x) << "," << ToString(msg->imu4_ang_y) << "," << ToString(msg->imu4_ang_z) << "\n";
	angFile.close();

// Write Motor inputs file
	std::ofstream uFile;
	uFile.open ("uData.txt", std::ios_base::app);
	uFile 
		<< ToString(msg->u1) << "," << ToString(msg->u2) << "," 
		<< ToString(msg->u3) << "," << ToString(msg->u4) << "," 
		<< "9.81\n";
	uFile.close();

	return;
}




int main(int argc, char **argv)
{
	// Empty the previous files
	std::ofstream delFile;
	delFile.open("posData.txt", std::ofstream::out | std::ofstream::trunc);
	delFile.close();
	delFile.open("angData.txt", std::ofstream::out | std::ofstream::trunc);
	delFile.close();
	delFile.open("uData.txt", std::ofstream::out | std::ofstream::trunc);
	delFile.close();


	// Initialize the node named "writeDataToFiles"
	ros::init(argc, argv, "writeDataToFiles") ;

	// Create Node Handle
	ros::NodeHandle n ;

	// Subscribe to topic (published from the rosbag)
	ros::Subscriber data = n.subscribe("sensor_data",10000, dataCallback);

	ROS_INFO("Ready for Data");

	// Loop (until ctrl+C)
	ros::spin();

	ROS_INFO("Ending writeDataToFiles");

	return 0;
}
