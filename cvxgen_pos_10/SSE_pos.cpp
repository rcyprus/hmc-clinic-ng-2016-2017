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
#include "geometry_msgs/Point.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

// Define some global variables
bool newData = false;
double pos[numSensors] = {};
double u[numInputs]    = {};
// timing
int lastTime = 0;
int curTime = 0;
double dt = 0;

// Add declarations for load_CA and load_YBu
void load_CA(int timeStep);
void load_YBu(int timeStep, double* yin, double* Uin);
void sensorDataCallback(const beginner_tutorials::SensorData::ConstPtr& msg);

// Define C fucntions
extern "C" void readArrayFromFile(const char* file_name, double* array);

void sensorDataCallback(const beginner_tutorials::SensorData::ConstPtr& msg)
{
  // Set flag
  newData = true;

  ROS_INFO("SSE got new pos data");
  
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
  ROS_INFO("Starting Pos SSE");
  ros::init(argc,argv,"PosSSE");
  ros::NodeHandle n;

  ros::Subscriber sensorData = n.subscribe("sensor_data", 100, sensorDataCallback);
  ros::Publisher posStates = n.advertise<geometry_msgs::Point>("pos_states",100);
  
  ros::Rate loop_rate(100); // 100 Hz

  // Initialize message
  geometry_msgs::Point position;

  // CVX setup
  set_defaults();
  setup_indexing();
  settings.verbose = 1;

  // Setup system matrices
  // const char* filenameA = "Amatrix.txt";
  // const char* filenameB = "Bmatrix.txt";
  // const char* filenameC = "Cmatrix.txt";
  // readArrayFromFile(filenameA, A);
  // readArrayFromFile(filenameB, B);
  // readArrayFromFile(filenameC, C);
  
  // printArrayDouble(A,numStates,numStates);
  // printf("\n");
  // printArrayDouble(B,numStates,numInputs);
  // printf("\n");
  // printf("%lf\n",B[0]);
  // printf("\n");
  // printArrayDouble(C,numSensors,numStates);
  // printf("\n");
  

  double x_prev[numStates];
  double x_pprev[numStates];
  
  // // START TIMING
  // clock_t begin = clock();
  
  // *** CHANGE THIS TO A WHILE LOOP IN FINAL VERSION ***
  //for (size_t ts = 0; ts < timeSteps; ++ts) {
  
  size_t ts = 0;

  while(ros::ok){
    // ROS_INFO("In While Loop, %d",ts);

    if (newData) {
      // In this case we only need to update the YBu matrix, not recreate it
      if (ts < timeSteps) {
        // Read in matrix of most recent control inputs
        // readLines(filenameU, &U[ts*numInputs], ts, numInputs, 1);
        for (size_t i = 0; i < numInputs; ++i) {
          U[ts*numInputs + i] = u[i];
        }
        // printArrayDouble(U,numInputs,timeSteps);

        // Read in sensor data from file
        // readLines(filenameY, y, ts, numSensors, 1);
        for(size_t i = 0; i < numSensors; ++i){
          y[i] = pos[i];
        }
        //printArrayDouble(y,1,numSensors);
        
        // Update solver parameters CA and YBu
        load_CA(ts);
        load_YBu(ts, y, U);
      }
      
      // Here we need to recreate the entire YBu matrix
      else {
        ROS_INFO("Windowing load data\n");
        
        int tstart = ts-(timeSteps-1);

        // clear old data in YBu
        for (size_t t = 0; t < numSensors*timeSteps; ++t) {
          YBu[t] = 0;
        }

        // Read in matrix of most recent control inputs
        //readLines(filenameU, U, tstart, numInputs, timeSteps);
        shiftArray(U, numInputs, timeSteps);
        // readLines(filenameU, &U[(timeSteps-1)*numInputs], tstart, numInputs, 1);
        for(size_t i = 0; i < numInputs; ++i){
          U[(timeSteps-1)*numInputs + i] = u[i];
        }
        //printArrayDouble(U,numInputs,timeSteps);

        // Loop through T timesteps
        for (size_t t = 0; t < timeSteps; ++t) {
          
          // readLines(filenameY, y, tstart+t, numSensors, 1);
          for(size_t i = 0; i < numSensors; ++i){
            y[i] = pos[i];
          }
          printArrayDouble(y,1,numSensors);

          // Create solver parameter YBu
          // (CA is already fully populated and does not change)
          load_YBu(t, y, U);
        }

        ROS_INFO("Full YBu matrix is (P x T):\n");
        printArrayDouble(params.YBu, numSensors, timeSteps);
      }
      
      // Print solver parameters
      //printf("Full CA matrix is (non-zero entries by T):\n");
      //printArrayDouble(params.CA, nonZeroEntries, timeSteps);
      //printf("Full YBu matrix is (numSensors x T):\n");
      //printArrayDouble(params.YBu, numSensors, timeSteps);
      
      // Solve optimization problem for initial state
      solve();
      
      ROS_INFO("Optimized x BEFORE dynamics propagation is:");
      printArrayDouble(vars.x, numStates, 1);

      for (size_t i = 0; i < numStates; ++i) {
        x[i] = vars.x[i];
      }
      
      if (ts < timeSteps) {
        propagateDynamics(ts, U, x);
      }
      else {
        printf("Windowing propagate dynamics\n");
        propagateDynamics(timeSteps-1, U, x);
      }
      
      printf("Optimized x AFTER dynamics propagation is:\n");
      printArrayDouble(x, numStates, 1);
      
      // // save output state x
      // sprintf(filenameX, "x%u.txt", ts);
      // writeFile(filenameX, x, numStates);

      newData = false;
    }
    else {
      // dt;

      // Copy old state x into x_prev
      for (size_t i = 0; i < numStates; ++i) {
        x_prev[i] = x[i];
      }

      // Approximate linear accelerations and calculate new position/velocity
      int ax = (x_prev[1] - x_pprev[1])/dt;
      int ay = (x_prev[2] - x_pprev[2])/dt;
      int az = (x_prev[3] - x_pprev[3])/dt;
      x[0] = x_prev[0] + x_prev[3]*dt + 0.5*az*dt*dt;
      x[1] = x_prev[1] + ax*dt;
      x[2] = x_prev[2] + ay*dt;
      x[3] = x_prev[3] + az*dt;
      
      // Copy former x_prev into x_pprev
      for (size_t i = 0; i < numStates; ++i) {
        x_pprev[i] = x_prev[i];
      }

    }

    // Create ROS message
    position.x = 0;
    position.y = 0;
    position.z = x[0];

    // ROS publish message
    posStates.publish(position);
    ros::spinOnce();
    loop_rate.sleep();


    // Increment "time step"s
    ts++;
  }
  
  // // END TIMING
  // clock_t end = clock();
  // double time_spent = (double) (end - begin) / CLOCKS_PER_SEC * 1000;
  // printf("Time to solve per timestep is is %.2f ms\n", time_spent/T);

/*
  printf("A is:\n");
  printArrayDouble(A,numStates,numStates);
  printf("B is:\n");
  printArrayDouble(B,numStates,numInputs);
  printf("C is:\n");
  printArrayDouble(C,numSensors,numStates);
*/

  return 0;
}

void load_CA(int timeStep) {
  // Set up CVX parameter matrix CA
  updateCA(timeStep);
  for (size_t i = 0; i < nonZeroEntries*timeSteps; ++i) {
    params.CA[i] = CA[i];
  }
  //printArrayDouble(params.CA,nonZeroEntries,timeSteps);
}

void load_YBu(int timeStep, double* yin, double* Uin) {
  // Set up CVX parameter vector YBu
  updateYBu(timeStep, yin, Uin);
  for (size_t i = 0; i < numSensors*timeSteps; ++i) {
    params.YBu[i] = YBu[i];
  }
  //printArrayDouble(params.YBu,numSensors,timeSteps);
}
