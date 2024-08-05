#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <time.h>
#include <fstream>

#include "eso.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std;

std::ofstream joint_state_file;
std::ofstream commands_file;
std::ofstream latency_file_snow_con;
std::ofstream target_file;
long time0, time1;
struct timespec spec;
static int setTime = 0;
static int pos;

// ESO values for effort (multiplier and reaction parameter)
ESO eso0(220, 22);
ESO eso1(220, 22);
ESO eso2(220, 22);
ESO eso3(220, 22);

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {

    // store commands and joint states to CSV file for latency testing
    joint_state_file.open("joint_state.csv");
    commands_file.open("commands.csv");
    latency_file_snow_con.open("latency_snow_con.csv");
    target_file.open("target.csv");

    // snow_con subscribes to /joint_states topic and publishes to /effort_controllers/commands topic
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscriber node created");

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 10);

    RCLCPP_INFO(this->get_logger(), "publisher node created");
  }

private: // function to print joint states
  void print_joints(double *pArm, double *vArm, double *pWheel, double *vWheel, double totalTime, int step, double targetAngle, double targetPosition)
  {
    int i;
    /*
    CSV GUIDE for joint states

    Column#     Label
    1       ->  totalTime
    2-5     ->  pWheel[4]
    6-9     ->  vWheel[4]
    10-13   ->  pArm[4]
    14-17   ->  vArm[4]
    18      ->  target angle
    */
    joint_state_file << totalTime;

    for (i = 0; i < 4; i++)
      joint_state_file << "," << *(pWheel + i); // divide by r to get m
    for (i = 0; i < 4; i++)
      joint_state_file << "," << *(vWheel + i);
    for (i = 0; i < 4; i++)
      joint_state_file << "," << *(pArm + i);
    for (i = 0; i < 4; i++)
      joint_state_file << "," << *(vArm + i);

    joint_state_file << "," << targetAngle;
    joint_state_file << "," << targetPosition;

    // printf("\n");
    joint_state_file << "," << step;
    joint_state_file << "\n";
  }

private: // function to print commands
  void print_commands(double *uArm, double *uWheel, double totalTime, int step)
  {
    int i;
    /*
    CSV GUIDE for commands

    Column#     Label
    1       ->  totalTime
    2-5     ->  uWheel[4]
    6-9     ->  uArm[4]
    10      ->  stepd
    */

    commands_file << totalTime;
    cout << totalTime;

    for (i = 0; i < 4; i++)
    {
      commands_file << "," << *(uWheel + i); // divide by r to get m
      cout << "," << *(uWheel + i);          // divide by r to get m
    }
    for (i = 0; i < 4; i++)
    {
      commands_file << "," << *(uArm + i);
      cout << "," << *(uArm + i);
    }

    // printf("\n");
    commands_file << "," << step;
    commands_file << "\n";
    cout << "," << step << endl;
  }

private: // function to calculate and print latency
  void print_latency(double totalTime)
  {
    // long x = (time1.tv_sec - time0.tv_sec) + (time1.tv_nsec - time0.tv_nsec) / 1000000000.0;

    // printf("Difference: %f\n", x);
    if (!setTime)
    {
      time1 = 0;
      setTime = 1;
    }

    // CACLCULATION FOR DELAY
    clock_gettime(CLOCK_REALTIME, &spec);
    time0 = spec.tv_nsec;

    float x = (time0 - time1) / 1000000.0; // after the call_back function, time1 is set. then when callback occurs again, time0 is set.

    if (x < 0)
      x = 1000 + x;

    // printf("%f,%f\n",totalTime,x);                     // difference between time1 and time0 is the delay
    if (time1)
      latency_file_snow_con << totalTime << "," << x << "\n"; // print to csv file

    // print_joints()
    clock_gettime(CLOCK_REALTIME, &spec);
    time1 = spec.tv_nsec;
  }
  
  void print_targets(double *d, double *v, double tA, double tP, int step, double totalTime)
  {
    int i;
    /*
    CSV GUIDE for targets

    Column#     Label
    1       ->  time
    2-5     ->  d[4]
    6-9     ->  v[4]
    10       ->  targetAngle
    11      ->  targetPosition
    12      ->  pstate
    */

    target_file << totalTime;

    for (i = 0; i < 4; i++)
    {
      target_file << "," << *(d + i);     
    }
    for (i = 0; i < 4; i++)
    {
      target_file << "," << *(v + i);
    }
    target_file << "," << tA;
    target_file << "," << tP;
    target_file << "," << step;
    target_file << "\n";
  }

  // function to turn on and off brakes, returns true if brake has been locked
  bool setBrakes(double *brake, double *uArm, double t)
  {
    // printf("\t time=%f, tsm=%f \n",t,tstartmove);
    static int j;
    static int setInit = 1;
    static double increment;
    bool isComplete = false;

    if (setInit)
    {
      // pos = -1;
      j = 0;
      setInit = 0;
      increment = t;
    }

    if (t > increment && j < 4)
    {
      uArm[j] = brake[j];
      increment += 0.10;
      j++;
    }

    if (t > increment + 0.3)
    {
      isComplete = true;
      setInit = 1;
    }

    return isComplete;
  }

  bool waitTime(double t, double stopTime) {
    static int setInit = 1;
    static bool isComplete;
    static double tstart;

    if(setInit) {
      setInit = 0;
      tstart = t;
      isComplete = false;
    }

    if (t > tstart + stopTime) {
      isComplete = true;
      setInit = 1;
    }

    return isComplete;
  }

  // function to initiate swiveling, returns true if swiveling is complete
  bool swivel(
      double *pArm, double *uArm,
      double t, double *d, double *v, double *a,
      double targetTime, double targetAngle,
      bool isHardcon, double *nStateSign)
  {

    // init variables
    int i;
    static int setInit = 1;
    static int initBrake = 1;
    static double currentTime;
    static bool isComplete;
    static double tstartmove;

    static double vCoast[4];
    static double brake[4];
    static int isEnd[4];
    static double adjustedTargetAngle[4];
    double s[4] = {1.0, -1.0, 1.0, -1.0};
    static double wheelCompensation = 0.02;

    // set STEP requirments
    if (setInit)
    {
      setInit = 0;
      tstartmove = t;
      pos = -1; // ESO control
      initBrake = 1;

      // when swiveling, 25% of time = acceleration, 50% = coast velocity, 25% deceleration
      for (i = 0; i < 4; i++)
      {
        adjustedTargetAngle[i] = s[i] * targetAngle;

        if(isHardcon) {
          if (targetAngle == M_PI / 2) {
            if      (nStateSign[i] > 0 && adjustedTargetAngle[i] > pArm[i])
              adjustedTargetAngle[i] += s[i] * wheelCompensation;
            else if (nStateSign[i] < 0 && adjustedTargetAngle[i] > pArm[i])
              adjustedTargetAngle[i] -= s[i] * wheelCompensation;
            else if (nStateSign[i] > 0 && adjustedTargetAngle[i] < pArm[i])
              adjustedTargetAngle[i] -= s[i] * wheelCompensation;
            else if (nStateSign[i] < 0 && adjustedTargetAngle[i] < pArm[i])
              adjustedTargetAngle[i] += s[i] * wheelCompensation;
          }

          else if (targetAngle == 0) {
            if      (nStateSign[i] > 0 && adjustedTargetAngle[i] > pArm[i])
              adjustedTargetAngle[i] -= s[i] * wheelCompensation;
            else if (nStateSign[i] < 0 && adjustedTargetAngle[i] > pArm[i])
              adjustedTargetAngle[i] += s[i] * wheelCompensation;
            else if (nStateSign[i] > 0 && adjustedTargetAngle[i] < pArm[i])
              adjustedTargetAngle[i] += s[i] * wheelCompensation;
            else if (nStateSign[i] < 0 && adjustedTargetAngle[i] < pArm[i])
              adjustedTargetAngle[i] -= s[i] * wheelCompensation;
          }
        }

        d[i] = pArm[i];
        vCoast[i] = (4.0 * (adjustedTargetAngle[i] - d[i])) / (3.0 * targetTime); // based on area of trapezoid
        v[i] = 0;                                                             // init to no speed;
        brake[i] = 1;
        isEnd[i] = 0;
      }

      return false;
    }

    // call setBrake function and wait until brakes has been set. 0.2s delay (only for hard_con)
    if (isHardcon && initBrake)
    {
      if (!setBrakes(brake, uArm, t))
        return false;
      // once complete,
      else
      {
        initBrake = 0;
        pos = 1;
        tstartmove = t;
      }
    }

    // simulated brakes for gazebo
    else if (!isHardcon)
    {
      pos = 1;

      for (i = 0; i < 4; i++)
        uArm[i] = 0;
    }

    // update currentTime
    currentTime = t - tstartmove;
    printf("ct=%.2f ta=%f", currentTime, targetAngle);

    // velocity control
    for (i = 0; i < 4; i++)
    {

      a[i] = vCoast[i] / (targetTime / 4.0);  

      if (currentTime <= targetTime / 4.0)
      {
        v[i] += a[i] * dt; // acceleration 25% of time
      }
      else if (currentTime > targetTime / 4.0 && currentTime <= (targetTime * (3.0 / 4.0)))
      {
        v[i] = vCoast[i]; // coast velocity 50% of time
        a[i] = 0;
      }
      else if (currentTime > targetTime * (3.0 / 4.0) && currentTime < targetTime)
      {
        a[i] = -1 * a[i];
        v[i] += a[i] * dt; // deceleration 25% of time
      }
      else
      {
        v[i] = 0;
        a[i] = 0;
      }

      // integrate distance
      if (fabs(adjustedTargetAngle[i] - d[i]) < 0.05)
        d[i] = adjustedTargetAngle[i];
      else
        d[i] += v[i] * dt;

      printf("d[%d]=%.3f,v[%d]=%.3f,a[%d]=%.3f  ", i, d[i], i, v[i],i,a[i]);
    }

    for(i=0; i<4; i++) {
      if (fabs(adjustedTargetAngle[i] - pArm[i]) < 0.02 && !isEnd[i]) {
        isEnd[i] = 1;
        uArm[i] = 0;
      }
    }

    // end condition
    if ( // target angles with an allowance of 0.05
        isEnd[0] &&
        isEnd[1] &&
        isEnd[2] &&
        isEnd[3] &&
        currentTime > 0.5)
    {
      isComplete = true;
      setInit = 1; // ready to initialize next step
      // nState = STEP2; // next STEP
      for (i=0; i<4; i++) {
        v[i] = 0;
        a[i] = 0;
      }
    }

    else
      isComplete = false;

    return isComplete;
  }

  // function to move robot to a target position
  bool moveRobot(
      double *s, double *u,
      double *pArm, double *uArm,
      double t, double *d, double *pWheel,
      double targetAngle,
      double *vArm, double targetPosition,
      double targetVelocity,
      bool isHardcon)
  {

    // init variables
    int i;
    static int setInit = 1;
    static int initBrake = 1;
    static double currentTime;
    static bool isComplete;
    static double tstartmove;
    static double d0[4];
    double angle[4] = {1.0, -1.0, 1.0, -1.0};

    if (setInit)
    {
      setInit = 0;
      pos = -1; // no control
      initBrake = 1;
      for (i = 0; i < 4; i++)
      {
        d[i] = pWheel[i]; // set initial / start position
        d0[i] = d[i];
      }
      return false;
    }

    // call setBrake function and wait until brakes has been set. 0.2s delay (for hard_con)
    if (isHardcon && initBrake)
    {
      // if (!setBrakes(brake, uArm, t)  && !uArm[0] && !uArm[1] && !uArm[2] && !uArm[3] )
      //   return false;
      // else
      // {
        pos = 0; // set to PID
        initBrake = 0;
        tstartmove = t; // set time baseline
      // }
    }

    else if (!isHardcon)
    { // for gazebo simulation
      pos = 0;
      for (i = 0; i < 4; i++)
        uArm[i] = -3 * (pArm[i] - angle[i]*targetAngle) - 2.5 * (vArm[i] - 0);
    }

    // update currentTime
    currentTime = t - tstartmove;
    printf("ct=%.2f ta=%f  ", currentTime, targetAngle);

    // end condition
    if ( // target position with allowance of 0.1
        (
          fabs(pWheel[0] - (s[0] * targetPosition + d0[0])) < 0.1 ||
          fabs(pWheel[1] - (s[1] * targetPosition + d0[1])) < 0.1 ||
          fabs(pWheel[2] - (s[2] * targetPosition + d0[2])) < 0.1 ||
          fabs(pWheel[3] - (s[3] * targetPosition + d0[3])) < 0.1 
        ) &&  currentTime >=  0.5)
    {
      for (i = 0; i < 4; i++)
      {
        u[i] = 0; // set effort back to 0 to stop motors running
      }
      isComplete = true;
      setInit = 1;
    }

    else
    {
      isComplete = false;
      for (i = 0; i < 4; i++)
      {
        u[i] = targetVelocity * s[i];
      }
    }

    for(i=0; i<4; i++) {
        printf("%d=%.3f - %.3f\t",i,  pWheel[i], s[i]*targetPosition+d0[i])  ;
        //printf("u[%d]=%f,tt=%f  ", i, u[i],targetVelocity);
    }

    return isComplete;
  }

  // primary function containing sequence of steps for robot path
private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double t;
    static double wheelRadius;
    static double centerToWheel;
    static int wait;
    int i;

    static double s[4];
    static double nStateSign[4];
    string frameID;
    static double targetTime;
    static double targetAngle;
    static double targetPosition;
    static double targetVelocity;
    static double tAngle[4];

    static double d[4];
    static double v[4];
    static double u[4];
    static double a[4];
    // static double dstartpos[4];
    static double tstartmove;
    static bool isHardcon;
    static int isESO;

    double pArm[4];
    double vArm[4];
    double pWheel[4];
    double vWheel[4];

    static double uArm[4];
    static double uWheel[4];
    static double inte[4], e[4], de[4], ePrev[4];

    std_msgs::msg::Float64MultiArray commands;
    t = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec * 1e-9);
    frameID = msg->header.frame_id;

    pArm[0] = msg->position[0];
    pArm[1] = msg->position[3];
    pArm[2] = msg->position[4];
    pArm[3] = msg->position[6];

    pWheel[0] = msg->position[1];
    pWheel[1] = msg->position[2];
    pWheel[2] = msg->position[5];
    pWheel[3] = msg->position[7];

    vArm[0] = msg->velocity[0];
    vArm[1] = msg->velocity[3];
    vArm[2] = msg->velocity[4];
    vArm[3] = msg->velocity[6];

    vWheel[0] = msg->velocity[1];
    vWheel[1] = msg->velocity[2];
    vWheel[2] = msg->velocity[5];
    vWheel[3] = msg->velocity[7];

    eso0.predict(pArm[0], 0, uWheel[0]);
    eso1.predict(pArm[1], 0, uWheel[1]);
    eso2.predict(pArm[2], 0, uWheel[2]);
    eso3.predict(pArm[3], 0, uWheel[3]);

    typedef enum states // state machine determining robot actions
    {
      INIT,
      START,

      STEP1, // arms swivel to 90deg
      STEP2, // move by 1m along -x axis
      STEP3, // arms swivel to 0deg
      STEP4, // move by 1m along +y axis
      STEP5, // arms swivel to 45deg
      STEP6, // robot rotates +180deg CW
      STEP7,  //
      STEP8,
      STEP9,
      STEP10,
      STEP11,
      STEP12,
      WAIT,
      OPEN, // unlocked brakes, displays pArm & pWheel (used for testing)
      END   // locked brakes, does nothing (used for testing)
    } RSTATES;

    static RSTATES pState, nState;

    switch (pState)
    {

    // initialize variable
    case INIT:
    {
      nState = START;
      tstartmove = t;
      pos = -1;
      wait = 0;
      targetVelocity = 3.0; // set target torque

      // check frame ID on wether its hardcon/simulation
      if (frameID == "") {
        isHardcon = false;
        wheelRadius = 0.044;      // model wheel radius
        centerToWheel = 0.3064 ;  // model center to wheel 
      }

      else {
        isHardcon = true;
        wheelRadius = 0.0435;     // actual wheel radius
        centerToWheel = 0.29863;  // actual center to wheel (used for rotation)
      }

      break;
    }

    case START:
    {
      if (t > tstartmove + 2)
      {
        nState = STEP3;
      }

      break;
    }

    // swivel 90deg
    case STEP1:
    {

      // set target anlge and time
      targetAngle = 0; // M_PI/4.0;
      targetTime = 2.0;
      nStateSign[0] = 1.0;
      nStateSign[1] = -1.0;
      nStateSign[2] = -1.0;
      nStateSign[3] = 1.0;

      // call swiveling function
      if (swivel(pArm, uArm, t, d, v, a, targetTime, targetAngle, isHardcon, nStateSign))
      {
        nState = STEP10; // 2 next STEP
        wait = 1;
      }

      break;
    }

     // swivel 90deg
    case STEP7:
    {

      // set target anlge and time
      targetAngle = M_PI / 4; // M_PI/4.0;
      targetTime = 2.0;
      nStateSign[0] = 1.0;
      nStateSign[1] = -1.0;
      nStateSign[2] = -1.0;
      nStateSign[3] = 1.0;

      // call swiveling function
      if (swivel(pArm, uArm, t, d, v, a, targetTime, targetAngle, isHardcon, nStateSign))
      {
        nState = STEP10; // next STEP
        wait = 1;
      }

      break;
    }

         // swivel 90deg
    case STEP10:
    {

      // set target anlge and time
      targetAngle = M_PI / 2; // M_PI/4.0;
      targetTime = 2.0;
      nStateSign[0] = 1.0;
      nStateSign[1] = -1.0;
      nStateSign[2] = -1.0;
      nStateSign[3] = 1.0;

      // call swiveling function
      if (swivel(pArm, uArm, t, d, v, a, targetTime, targetAngle, isHardcon, nStateSign))
      {
        nState = STEP1; // next STEP
        wait = 1;
      }

      break;
    }

    // move robot 1m along -x axis
    case STEP2:
    {

      // set target angle and position
      targetPosition = (1.0 / wheelRadius);
      targetAngle = M_PI / 2; // simulation only

      // set direction of rotation for wheels
      s[0] = 1.0;
      s[1] = -1.0;
      s[2] = -1.0;
      s[3] = 1.0;

      // call function to move robot
      if (moveRobot(s, u, pArm, uArm, t, d, pWheel, targetAngle, vArm, targetPosition, targetVelocity, isHardcon))
      {
        nState = STEP5; // next STEP
        wait = 1;
      }
      break;
    }

    // swivel to 0deg
    case STEP3:
    {
      // set target angle and time
      targetAngle = 0; // M_PI/4.0;
      targetTime = 2.0;
      nStateSign[0] = 1.0;
      nStateSign[1] = 1.0;
      nStateSign[2] = -1.0;
      nStateSign[3] = -1.0;

      // call swiveling function
      if (swivel(pArm, uArm, t, d, v, a, targetTime, targetAngle, isHardcon, nStateSign))
      {
        nState = STEP4; // next STEP
        wait = 1;
      }

      break;
    }
    

    // move 1m
    case STEP4:
    {
      // set target angle and position
      targetPosition = (1.0 / wheelRadius);
      targetAngle = 0; // simulation only

      // set direction of rotation for wheels
      s[0] = 1.0;
      s[1] = 1.0;
      s[2] = -1.0;
      s[3] = -1.0;

      // call function to move robot
      if (moveRobot(s, u, pArm, uArm, t, d, pWheel, targetAngle, vArm, targetPosition, targetVelocity, isHardcon))
      {
        nState = STEP9; // next STEP
        wait = 1;
      }
      break;
    }

    // move 1m
    case STEP8:
    {
      // set target angle and position
      targetPosition = (1.0 / wheelRadius);
      targetAngle = 0; // simulation only

      // set direction of rotation for wheels
      s[0] = -1.0;
      s[1] = -1.0;
      s[2] = 1.0;
      s[3] = 1.0;

      // call function to move robot
      if (moveRobot(s, u, pArm, uArm, t, d, pWheel, targetAngle, vArm, targetPosition, targetVelocity, isHardcon))
      {
        nState = STEP3; // next STEP
        wait = 1;
      }
      break;
    }

        // swivel to 0deg
    case STEP9:
    {
      // set target angle and time
      targetAngle = 0; // M_PI/4.0;
      targetTime = 2.0;
      nStateSign[0] = -1.0;
      nStateSign[1] = -1.0;
      nStateSign[2] = 1.0;
      nStateSign[3] = 1.0;

      // call swiveling function
      if (swivel(pArm, uArm, t, d, v, a, targetTime, targetAngle, isHardcon, nStateSign))
      {
        nState = STEP8; // next STEP
        wait = 1;
      }

      break;
    }

    // swivel 45deg
    case STEP5:
    {

      // set targets
      targetAngle = M_PI / 4.0; // set target angle
      targetTime = 2.0;
      nStateSign[0] = 1.0;
      nStateSign[1] = 1.0;
      nStateSign[2] = 1.0;
      nStateSign[3] = 1.0;

      // call swiveling function
      if (swivel(pArm, uArm, t, d, v, a, targetTime, targetAngle, isHardcon, nStateSign))
      {
        nState = STEP6; // next STEP
        wait = 1;
      }

      break;
    }

    // turn around by 180
    case STEP6:
    {

      // set target angle and position
      targetPosition = ((2) * M_PI * (centerToWheel / wheelRadius) / 2); // calculated mid of robot to tip of wheel; set target position
      targetAngle = M_PI / 4;                            // simulation only

      // set direction of rotation for wheels
      s[0] = 1.0;
      s[1] = 1.0;
      s[2] = 1.0;
      s[3] = 1.0;

      // call function to move robot
      if (moveRobot(s, u, pArm, uArm, t, d, pWheel, targetAngle, vArm, targetPosition, targetVelocity, isHardcon))
      {
        nState = STEP3;  // next STEP
        wait = 1;
      }

      break;
    }

    case WAIT:
    { // unlock all brakes only
      if (waitTime(t,1.0)) 
        wait = 0;

      for (i = 0; i < 4; i++)
      {
        uWheel[i]  = -0.2*(vArm[i] - 0);
        uArm[i]    = 0; 
        // uArm[i] = 0;
      }

      break;
    }

    case OPEN:
    { // unlock all brakes only
      for (i = 0; i < 4; i++)
      { // print values of joints
        uArm[i] = 1.0;
        printf("pW[%d]=%.3f, pA[%d]=%.3f  ", i, pWheel[i], i, pArm[i]);
      }
      pos = -1;
      printf("---------------OPEN-----------------");

      break;
    }

    case END:
    { // does nothing
      pos = -1;
      printf("---------------END-----------------");
      break;
    }
    }

    if(wait) 
      pState = WAIT;
    else
      pState = nState;

    printf("Step: %i\t hard_con?=%i\n", pState - 1, isHardcon);

    if(isHardcon)
      for(i=0; i<4; i++)
        vArm[i] = vWheel[i] * 0.33;

    // ESO Control for Swiveling
    if (pos == 1)
    {
      u[0] = -0.10 * (pArm[0] - d[0]) - 0.14 * (vArm[0] - v[0]);
      u[0] -= eso0.fHat / eso0.b;
      //u[0] += 0.025 * a[0];
      u[0] += a[0] / eso0.b;

      u[1] = -0.10 * (pArm[1] - d[1]) - 0.14 * (vArm[1] - v[1]);
      u[1] -= eso1.fHat / eso1.b;
      //u[1] += 0.025 * a[1];
      u[1] += a[1] / eso1.b;

      u[2] = -0.10 * (pArm[2] - d[2]) - 0.14 * (vArm[2] - v[2]);
      u[2] -= eso2.fHat / eso2.b;
      //u[2] += 0.025 * a[2];
      u[2] += a[2] / eso2.b;

      u[3] = -0.10 * (pArm[3] - d[3]) - 0.14 * (vArm[3] - v[3]);
      u[3] -= eso3.fHat / eso3.b;
      //u[3] += 0.025 * a[3];
      u[3] += a[3] / eso3.b;

      for (i = 0; i < 4; i++)
      {
        // printf("u[%d]=%.3f\t",i,u[i]);
        // printf("pW[%d]=%.3f, d[%d]=%.3f  ",i,pWheel[i],i,d[i]);
        uWheel[i] = u[i];
      }
      printf("\n");
      isESO = 1;
    }
    // PID Control for Forward/sidewards/rotations movements
    else if (pos == 0)
    {
      for (i = 0; i < 4; i++)
      {
        e[i] = vWheel[i] - u[i] * 1.0;
        inte[i] = inte[i] + e[i] * dt;
        de[i] = e[i] - ePrev[i];
        ePrev[i] = e[i];
      }

      uWheel[0] = -0.032 * (e[0]) - 0.1 * inte[0] - 0.0 * de[0]; 
      uWheel[1] = -0.032 * (e[1]) - 0.1 * inte[1] - 0.0 * de[1]; 
      uWheel[2] = -0.032 * (e[2]) - 0.1 * inte[2] - 0.0 * de[2]; 
      uWheel[3] = -0.032 * (e[3]) - 0.1 * inte[3] - 0.0 * de[3]; 

      isESO = 0;
    }

    else
    {
      isESO = 0;
      for (i = 0; i < 4; i++)
      {
        uWheel[i] = 0;
        // uArm[i] = 0;
      }
    }

    // torque limiter (soft)
    for (i = 0; i < 4; i++)
    {
      // printf("%f\n", uArm[i]);
      if (fabs(uWheel[i]) > 0.6 && uWheel[i] >= 0)
        uWheel[i] = 0.6;
      else if (fabs(uWheel[i]) > 0.6 && uWheel[i] < 0)
        uWheel[i] = -0.6;
    }

    // publish commands to /effort_controllers/commands
    commands.data.push_back(uArm[0]);
    commands.data.push_back(uArm[1]);
    commands.data.push_back(uArm[2]);
    commands.data.push_back(uArm[3]);

    commands.data.push_back(uWheel[0]);
    commands.data.push_back(uWheel[1]);
    commands.data.push_back(uWheel[2]);
    commands.data.push_back(uWheel[3]);

    publisher_->publish(commands);

    //commands to print topic values for testing
    print_joints(pArm, vArm, pWheel, vWheel, t, pState - 1,targetAngle, targetPosition);
    print_commands(uArm, uWheel, t, 0);
    print_latency(t);
    print_targets(d,v,targetAngle, targetPosition, pState - 1,t);
  }
  // Declaration of subscription_ attribute
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  // Declaration of the publisher_ attribute
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

// main function
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
