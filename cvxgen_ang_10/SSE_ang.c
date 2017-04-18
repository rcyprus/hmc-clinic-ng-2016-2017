/* Produced by CVXGEN, 2017-03-26 23:17:32 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
extern "C" {
  #include "CVX_inputs.h"
  #include "solver.h"
  #include <time.h>
}

#include "ros/ros.h"
#include "beginner_tutorials/SensorData.h"
#include "geometry_msgs/Quaternion.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

// Define C functions


struct quaternions{
  double qx;
  double qy;
  double qz;
  double qw;
};

// Define some global variables
bool newData = false;
double ang[24] = {}; // 4*6
// timing
int lastTime = 0;
int curTime = 0;
double dt = 0;

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

  u[0] = msg->u1;
  u[1] = msg->u2;
  u[2] = msg->u3;
  u[3] = msg->u4;
  u[4] = 9.81; // Gravity

  // Time
  lastTime = curTime;
  curTime = msg->header.stamp.nsec;
  dt = ((curTime - lastTime) % (int)pow(10,9) ) * pow(10,-9);
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
  quaternions angles;


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

  // // Create string to store y filename
  // const char* filenameY = "Ymatrix.txt";
  // const char* filenameU = "Umatrix.txt";
  // char filenameX[6];
  
  // START TIMING
  // clock_t begin = clock();

  size_t ts = 0;

  while(ros::ok){ 
    printf("Current timestep is: %d\n", ts);

    if (newData) {
      // In this case we only need to update the YBu matrix, not recreate it
      if (ts < T) {
        // Read in matrix of most recent control inputs
        // readLines(filenameU, U, 0, M, T);
        for(size_t i = 0; i < M; ++i){
          U[ts*M + i] = u[i];
        }
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
        // readLines(filenameU, U, tstart, M, T);
        for(size_t i = 0; i < P; ++i){
          y[i] = pos[i];
        }
        printArrayDouble(U,M,T);

        // Loop through T timesteps
        for (size_t t = 0; t < T; ++t) {
          
          // readLines(filenameY, y, tstart+t, P, 1);
          for(size_t i = 0; i < P; ++i){
            y[i] = pos[i];
          }
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

      for (size_t i = 0; i < N; ++i) {
        x[i] = vars.x[i];
      }
      
      if (ts < T) {
        propagateDynamics(ts, U, x);
      }
      else {
        printf("Windowing propagate dynamics\n");
        propagateDynamics(T-1, U, x);
      }
      
      printf("Optimized x AFTER dynamics propagation is:\n");
      printArrayDouble(x, N, 1);
      
      // // save output state x
      // sprintf(filenameX, "x%u.txt", ts);
      // writeFile(filenameX, x, N);

      newData = false;
    }
    else {
      // dt;

      // Copy old state x into x_prev
      for (size_t i = 0; i < N; ++i) {
        x_prev[i] = x[i];
      }

      // Approximate angular accelerations and calculate new orientation/
      // angular velocities
      int alphax = (x_prev[7] - x_pprev[7])/dt;
      int alphay = (x_prev[8] - x_pprev[8])/dt;
      int alphaz = (x_prev[9] - x_pprev[9])/dt;
      x[4] = x_prev[4] + x_prev[7]*dt + 0.5*alphax*dt*dt;
      x[5] = x_prev[5] + x_prev[8]*dt + 0.5*alphay*dt*dt;
      x[6] = x_prev[6] + x_prev[9]*dt + 0.5*alphaz*dt*dt;
      x[7] = x_prev[7] + alphax*dt;
      x[8] = x_prev[8] + alphay*dt;
      x[9] = x_prev[9] + alphaz*dt;

      // Copy former x_prev into x_pprev
      for (size_t i = 0; i < N; ++i) {
        x_pprev[i] = x_prev[i];
      }

    }

    // Create the message
    angles = toQuaternions(x[0], x[1], x[2]);
    orientation.x = angles.qx;
    orientation.y = angles.qy;
    orientation.z = angles.qz;
    orientation.w = angles.qw;

    // ROS publish message
    angStates.publish(orientation);
    ros::spinOnces();
    loop_rate.sleep();

    // Increment "time step"
    ts++;
  }
  
  // // END TIMING
  // clock_t end = clock();
  // double time_spent = (double) (end - begin) / CLOCKS_PER_SEC * 1000;
  // printf("Time to solve per timestep is is %.2f ms\n", time_spent/TT);

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
