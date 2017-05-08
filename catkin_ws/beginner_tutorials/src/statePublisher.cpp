/**
 * statePublisher
 * 
 * pull states from Ang_SSE and Pos_SSE and publish to pixhawk
 * 
 */

#include "ros/ros.h"
#include "beginner_tutorials/SensorData.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

// Define global variables to store states in
geometry_msgs::Point pos;
geometry_msgs::Quaternion ang;

geometry_msgs::PoseStamped pixSE;

void posCallback(const geometry_msgs::Point::ConstPtr & msg)
{
    pos.x = msg->x;
    pos.y = msg->y;
    pos.z = msg->z;
}
void angCallback(const geometry_msgs::Quaternion::ConstPtr & msg)
{
    ang.x = msg->x;
    ang.y = msg->y;
    ang.z = msg->z;
    ang.w = msg->w;
}

void pixStateCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    pixSE.header = msg->header;
    pixSE.pose = msg->pose;
}



int main(int argc, char **argv) {
    ROS_INFO("Starting statePublisher");
    ros::init(argc,argv,"statePublisher");
    ros::NodeHandle n;

    ros::Subscriber posStates = n.subscribe("pos_states",100, posCallback);
    ros::Subscriber angStates = n.subscribe("ang_states",100, angCallback);
    // ros::Subscriber pixStates = n.subscribe("/mavros/local_position_tmp/pose"
    //                                         ,100, pixStateCallback);


    // Publish to the Pixhawk controller
    ros::Publisher fullStates = n.advertise<geometry_msgs::PoseStamped>
                                ("state_estimate/pose",100);

    ros::Rate loop_rate(100);

    geometry_msgs::PoseStamped states; 

    while(ros::ok){
        ROS_INFO("Published states to Pixhawk");
        // ROS_INFO("Looping pix states back");

        // Construct the message
        states.header.stamp = ros::Time::now();
        states.header.frame_id = "/world";
        states.pose.position = pos;
        states.pose.orientation = ang;

        // states = pixSE;

        // Publish the message
        fullStates.publish(states);

        ros::spinOnce();
        loop_rate.sleep();
    }

}