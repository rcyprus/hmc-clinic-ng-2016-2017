/* Produced by CVXGEN, 2017-03-26 23:17:32 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */

#include "CVX_inputs.h"
#include "solver.h"
#include <time.h>

#include "ros/ros.h"
#include "beginner_tutorials/SensorData.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

struct quaternions{
  double qx;
  double qy;
  double qz;
  double qw;
};

bool newData = false;
double ang[6*4] = {};

// Add declarations for load_CA and load_YBu
void load_CA(int timeStep);
void load_YBu(int timeStep, double* yin, double* Uin);
void sensorDataCallback(const beginner_tutorials::SensorData::ConstPtr& msg);
quaternions toQuaternions(double roll, double pitch, double yaw);


void sensorDataCallback(const beginner_tutorials::SensorData::ConstPtr& msg)
{
  // Set flag
  newData = true;

  ROS_INFO("SSE got new ang data");
  /*
  pos[0] = msg->laser0_min_dist;
  pos[1] = msg->laser1_min_dist;
  pos[2] = msg->imu1_vel_x;
  pos[3] = msg->imu1_vel_y;
  pos[4] = msg->vz_1;
  pos[5] = msg->imu2_vel_x;
  pos[6] = msg->imu2_vel_y;
  pos[7] = msg->vz_2;
  pos[8] = msg->imu3_vel_x;
  pos[9] = msg->imu3_vel_y;
  pos[10] = msg->vz_3;
  pos[11] = msg->imu4_vel_x;
  pos[12] = msg->imu4_vel_y;
  pos[13] = msg->vz_4;
  */
  ang[0] = msg->imu1_roll;
  ang[1] = msg->imu1_pitch;
  ang[2] = msg->imu1_yaw;
  ang[3] = msg->imu1_ang_x;
  ang[4] = msg->imu1_ang_y;
  ang[5] = msg->imu1_ang_z;

  ang[6] = msg->imu2_roll;
  ang[7] = msg->imu2_pitch;
  ang[8] = msg->imu2_yaw;
  ang[9] = msg->imu2_ang_x;
  ang[10] = msg->imu2_ang_y;
  ang[11] = msg->imu2_ang_z;

  ang[12] = msg->imu3_roll;
  ang[13] = msg->imu3_pitch;
  ang[14] = msg->imu3_yaw;
  ang[15] = msg->imu3_ang_x;
  ang[16] = msg->imu3_ang_y;
  ang[17] = msg->imu3_ang_z;

  ang[18] = msg->imu4_roll;
  ang[19] = msg->imu4_pitch;
  ang[20] = msg->imu4_yaw;
  ang[21] = msg->imu4_ang_x;
  ang[22] = msg->imu4_ang_y;
  ang[23] = msg->imu4_ang_z;
}


int main(int argc, char **argv) {
  // Setup ROS stuff
  ROS_INFO("Starting Ang SSE");
  ros::init(argc,argv,"AngSSE");
  ros::NodeHandle n;

  ros::Subscriber sensorData = n.subscribe("sensor_data", 100, sensorDataCallback);
  ros::Publisher angStates = n.advertise<geometry_msgs::Quaternion>("ang_states",100);
  // ros::Publisher posStats = n.advertise<geometry_msgs::Point>("pos_states",100);
  
  ros::Rate loop_rate(100); // 100 Hz

  // Initialize message
  geometry_msgs::Quaternion orientation;



  // CVX setup
  set_defaults();
  setup_indexing();
  settings.verbose = 1;

  // Setup system matrices
  const char* filenameA = "Amatrix.txt";
  const char* filenameB = "Bmatrix.txt";
  const char* filenameC = "Cmatrix.txt";
  readArrayFromFile(filenameA, A);
  readArrayFromFile(filenameB, B);
  readArrayFromFile(filenameC, C);
  
  // Create string to store y filename
  const char* filenameY = "Ymatrix.txt";
  const char* filenameU = "Umatrix.txt";
  char filenameX[6];
  
  // START TIMING
  clock_t begin = clock();
  
  // *** CHANGE THIS TO A WHILE LOOP IN FINAL VERSION ***
  const size_t TT = 20;
  for (size_t ts = 0; ts < TT; ++ts) {
    printf("Current timestep is: %d\n", ts);


    // In this case we only need to update the YBu matrix, not recreate it
    if (ts < T) {
      // Read in matrix of most recent control inputs
      readLines(filenameU, U, 0, M, T);
      printArrayDouble(U,M,T);

      // Read in sensor data from file
      readLines(filenameY, y, ts, P, 1);
      //printArrayDouble(y,1,P);
      
      // Update solver parameters CA and YBu
      load_CA(ts);
      load_YBu(ts, y, U);
    }
    
    // Here we need to recreate the entire YBu matrix
    else {
      printf("Windowing load data\n");
      
      int tstart = ts-(T-1);

      // clear old data in YBu
      for (size_t t = 0; t < P*T; ++t) {
        YBu[t] = 0;
      }

      // Read in matrix of most recent control inputs
      readLines(filenameU, U, tstart, M, T);
      printArrayDouble(U,M,T);

      // Loop through T timesteps
      for (size_t t = 0; t < T; ++t) {
        
        readLines(filenameY, y, tstart+t, P, 1);
        printArrayDouble(y,1,P);

        // Create solver parameter YBu
        // (CA is already fully populated and does not change)
        load_YBu(t, y, U);
      }

      printf("Full YBu matrix is (P x T):\n");
      printArrayDouble(params.YBu, P, T);
    }
    
    // Print solver parameters
    //printf("Full CA matrix is (non-zero entries by T):\n");
    //printArrayDouble(params.CA, nonZeroEntries, T);
    //printf("Full YBu matrix is (P x T):\n");
    //printArrayDouble(params.YBu, P, T);
    
    // Solve optimization problem for initial state
    solve();
    
    printf("\nOptimized x BEFORE dynamics propagation is:\n");
    printArrayDouble(vars.x, N, 1);
    
    if (ts < T) {
      propagateDynamics(ts, U, vars.x);
    }
    else {
      printf("Windowing propagate dynamics\n");
      propagateDynamics(T-1, U, vars.x);
    }
    
    printf("Optimized x AFTER dynamics propagation is:\n");
    printArrayDouble(vars.x, N, 1);
    
    // save output state x
    sprintf(filenameX, "x%u.txt", ts);
    writeFile(filenameX, vars.x, N);
  }
  
  // END TIMING
  clock_t end = clock();
  double time_spent = (double) (end - begin) / CLOCKS_PER_SEC * 1000;
  printf("Time to solve per timestep is is %.2f ms\n", time_spent/TT);

/*
  printf("A is:\n");
  printArrayDouble(A,N,N);
  printf("B is:\n");
  printArrayDouble(B,N,M);
  printf("C is:\n");
  printArrayDouble(C,P,N);
*/

  return 0;
}



void load_CA(int timeStep) {
  // Set up CVX parameter matrix CA
  updateCA(timeStep);
  for (size_t i = 0; i < nonZeroEntries*T; ++i) {
    params.CA[i] = CA[i];
  }
  //printArrayDouble(params.CA,nonZeroEntries,T);
}

void load_YBu(int timeStep, double* yin, double* Uin) {
  // Set up CVX parameter vector YBu
  updateYBu(timeStep, yin, Uin);
  for (size_t i = 0; i < P*T; ++i) {
    params.YBu[i] = YBu[i];
  }
  //printArrayDouble(params.YBu,P,T);
}

/**
 * @brief      convert Euler to Quaternion angles
 *
 * @param[in]  roll   
 * @param[in]  pitch  
 * @param[in]  yaw    
 *
 * @return     returns a struct of quaternion angles
 * source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
quaternions toQuaternions(double roll, double pitch, double yaw)
{
  quaternions q;

  double t0 = std::cos(yaw * 0.5);
  double t1 = std::sin(yaw * 0.5);
  double t2 = std::cos(roll * 0.5);
  double t3 = std::sin(roll * 0.5);
  double t4 = std::cos(pitch * 0.5);
  double t5 = std::sin(pitch * 0.5);

  q.qw = t0 * t2 * t4 + t1 * t3 * t5;
  q.qx = t0 * t3 * t4 - t1 * t2 * t5;
  q.qy = t0 * t2 * t5 + t1 * t3 * t4;
  q.qz = t1 * t2 * t4 - t0 * t3 * t5;
  return q;
}
