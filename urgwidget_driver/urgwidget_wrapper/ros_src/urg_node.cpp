#include <ros/ros.h>

#include <tf/tf.h> // tf header for resolving tf prefix
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <string>

#include "urg_sensor.h"
#include "urg_utils.h"

#define MULTIECHO 0 //1 if we want multiecho, 0 otherwise

std::string frame_id_; ///< Parent tf frame of this link

ros::Publisher laser_pub_;
ros::Publisher primary_echo_pub_;
ros::Publisher secondary_echo_pub_;

int multiecho = 0;
int laserNum = 0;

/*void print_echo_data(long data[], unsigned short intensity[], int index)
{
    int i;

    // [mm]
    for (i = 0; i < URG_MAX_ECHO; ++i) {
        printf("%ld, ", data[(URG_MAX_ECHO * index) + i]);
    }

    // [1]
    for (i = 0; i < URG_MAX_ECHO; ++i) {
        printf("%d, ", intensity[(URG_MAX_ECHO * index) + i]);
    }
}*/

void publishData(urg_t *urg, long data[], unsigned short intensity[], int data_n, long time_stamp){
  (void)urg;
  int urg_max_echo = ((multiecho) ? URG_MAX_ECHO : 1);
  
  // Set up multiple output topics
  sensor_msgs::LaserScan scan;
  scan.header.stamp = ros::Time::now();
  scan.header.frame_id = frame_id_;
  scan.angle_min = -2.35619445;
  scan.angle_max = 2.35619445;
  scan.angle_increment = 0.004363322;
  scan.time_increment = 0.000017329;
  scan.scan_time = 0.025;
  long minr;
  long maxr;
  urg_distance_min_max(urg, &minr, &maxr);
  scan.range_min = (float)minr/1000.0;
  scan.range_max = (float)maxr/1000.0;
  
  scan.ranges.resize(data_n);
  scan.intensities.resize(data_n);
  
  sensor_msgs::LaserScan primary = scan;
  sensor_msgs::LaserScan secondary = scan;
  
  //printf("# n = %d, time_stamp = %ld\n", data_n, time_stamp);
  for (int i = 0; i < data_n; i++) {
    if (multiecho) {
        scan.ranges[i] = (float)data[(urg_max_echo * i) + 0]/1000.0;
        primary.ranges[i] = (float)data[(urg_max_echo * i) + 1]/1000.0;
        secondary.ranges[i] = (float)data[(urg_max_echo * i) + 2]/1000.0;
        scan.intensities[i] = intensity[(urg_max_echo * i) + 0];
        primary.intensities[i] = intensity[(urg_max_echo * i) + 1];
        secondary.intensities[i] = intensity[(urg_max_echo * i) + 2];
    } else {
        scan.ranges[i] = (float)data[(urg_max_echo * i) + 0]/1000.0;
        primary.ranges[i] = (float)data[(urg_max_echo * i) + 0]/1000.0;
        secondary.ranges[i] = (float)data[(urg_max_echo * i) + 0]/1000.0;
        scan.intensities[i] = intensity[(urg_max_echo * i) + 0];
        primary.intensities[i] = intensity[(urg_max_echo * i) + 0];
        secondary.intensities[i] = intensity[(urg_max_echo * i) + 0];
    }
  }
  
  laser_pub_.publish(scan);
  primary_echo_pub_.publish(primary);
  secondary_echo_pub_.publish(secondary);
}

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ros::init(argc, argv, "urg_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  // Get parameters so we can change these later.
  std::string tf_prefix;
  pnh.param<std::string>("tf_prefix", tf_prefix, "");
  pnh.param<std::string>("frame_id", frame_id_, "laser");
  frame_id_ = tf::resolve(tf_prefix, frame_id_);

  //check for multiecho parameter
  if (pnh.hasParam("multiecho")) {
      pnh.getParam("multiecho", multiecho);
  } else { //set the parameter
      pnh.setParam("multiecho", (int)MULTIECHO);
      multiecho = MULTIECHO;
  }

  // ROBERT CYPRUS COMMENTED -- PLACED BELOW
  //laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 20);
  //primary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("secondary_scan", 20);
  //secondary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("third_scan", 20);
  
  urg_t urg;
  int max_data_size;
  long *data = NULL;
  unsigned short *intensity = NULL;
  long time_stamp;
  int n = 0;
  
  // ROBERT CYPRUS COMMENTED
  // const char *ip_address = "192.168.0.10";
  // urg_connection_type_t connection_type = URG_ETHERNET;
  // long baudrate_or_port = 10940;
  // const char *device = ip_address;
  urg_connection_type_t connection_type = URG_SERIAL; // ROBERT CYPRUS INSERTED
  long baudrate_or_port = 115200; // ROBERT CYPRUS INSERTED

//ROBERT CYPRUS INSERTED FROM HERE
  //check for laserNum parameter
  if (pnh.hasParam("laserNum"))
      pnh.getParam("laserNum", laserNum);
  else { //set the parameter
      pnh.setParam("laserNum", laserNum);
  }
  const char *device;

  if( laserNum == 0) {
    device = "/dev/ttyACM0";
    laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_laser0", 20);
    primary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("secondary_scan_laser0", 20);
    secondary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("third_scan_laser0", 20);
  }
  else if (laserNum == 1) {
    device = "/dev/ttyACM1";
    laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_laser1", 20);
    primary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("secondary_scan_laser1", 20);
    secondary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("third_scan_laser1", 20);
  }
  else {
    device = "/dev/ttyACM0";
    laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_laser0", 20);
    primary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("secondary_scan_laser0", 20);
    secondary_echo_pub_ = nh.advertise<sensor_msgs::LaserScan>("third_scan_laser0", 20);
  }
  
  printf("device: %s\n", device);
//ROBERT CYPRUS INSERTED TO HERE



  ROS_INFO("Opening laser.");
  if (urg_open(&urg, connection_type, device, baudrate_or_port) < 0) {
      printf("urg_open: %s, %ld: %s\n", device, baudrate_or_port, urg_error(&urg));
      return 1;
  }

  max_data_size = urg_max_data_size(&urg);
  if (multiecho) {
      data = (long *)malloc(max_data_size * 3 * sizeof(data[0]));
      intensity = (unsigned short *)malloc(max_data_size * 3 * sizeof(intensity[0]));
  } else {
      data = (long *)malloc(max_data_size * sizeof(long));
      intensity = (unsigned short *)malloc(max_data_size * sizeof(unsigned short));
  }

  if (!data) {
      return 1;
  }
 
  if (multiecho) {
      urg_start_measurement(&urg, URG_MULTIECHO_INTENSITY, 0, 0);
  } else {
      urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 0, 0);
  }
 
  // Loop until ros exits
  while(ros::ok()){
    if (multiecho) {
        n = urg_get_multiecho_intensity(&urg, data, intensity, &time_stamp);
    } else {
        n = urg_get_distance_intensity(&urg, data, intensity, &time_stamp);
    }

    if (n <= 0) {
	  printf("urg_get_multiecho_intensity: %s\n", urg_error(&urg));
	  //break; //commented this break to avoid the driver to stop
    } else {
        publishData(&urg, data, intensity, n, time_stamp);
        printf("Timestamp=%ld\n", time_stamp); //added to see the timestamp output
        ros::spinOnce(); // Check in with ros
    }
  }
  
  free(data);
  free(intensity);
  urg_close(&urg);

  return EXIT_SUCCESS;
}

