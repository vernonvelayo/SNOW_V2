#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <time.h>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std;

std::ofstream joint_state_file;
std::ofstream commands_file;
long time0, time1;
struct timespec spec;
static int setTime = 0;
static int pos;
std::ofstream latency_file_snow_con;

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

    // snow_con subscribes to /joint_states topic and publishes to /effort_controllers/commands topic
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscriber node created");

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 10);

    RCLCPP_INFO(this->get_logger(), "publisher node created");
  }

private: // function to print joint states
  void print_joints(double *pArm, double *vArm, double *pWheel, double *vWheel, double totalTime, int step)
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

  // function to move robot to a target position
  bool moveMecanumWheel(
      double *s, double *u,
      double t, double *d, double *pWheel,
      double targetPosition,
      double *targetVelocity)
  {

    // init variables
    int i;
    static int setInit = 1;
    static double currentTime;
    static bool isComplete;
    static double tstartmove;
    static double d0[4];

    if (setInit)
    {
      setInit = 0;
      pos = -1; // no control
      tstartmove = t; // set time baseline
      for (i = 0; i < 4; i++)
      {
        d[i] = pWheel[i]; // set initial / start position
        d0[i] = d[i];
      }
      return false;
    }

    // update currentTime
    currentTime = t - tstartmove;

    // end condition
    if ( // target position with allowance of 0.1
        (
            fabs(pWheel[0] - (s[0] * targetPosition + d0[0])) < 0.1 ||
            fabs(pWheel[1] - (s[1] * targetPosition + d0[1])) < 0.1 ||
            fabs(pWheel[2] - (s[2] * targetPosition + d0[2])) < 0.1 ||
            fabs(pWheel[3] - (s[3] * targetPosition + d0[3])) < 0.1) &&
        currentTime >= 0.5)
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
        u[i] = targetVelocity[i] * s[i];
      }
    }

    for (i = 0; i < 4; i++)
    {
      printf("%d=%.3f - %.3f\t", i, pWheel[i], s[i] * targetPosition + d0[i]);
      // printf("u[%d]=%f,tt=%f  ", i, u[i],targetVelocity);
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
    static double isOff[4];
    int i;

    static double s[4];
    static double nStateSign[4];
    string frameID;
    static double targetTime;
    static double targetAngle;
    static double targetPosition;
    static double targetVelocity[4];

    static double d[4];
    static double v[4];
    static double u[4];
    static double a[4];
    // static double dstartpos[4];
    static double tstartmove;
    static bool isHardcon;

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

    typedef enum states // state machine determining robot actions
    {
      INIT,
      START,

      STEP1, // arms swivel to 90deg
      STEP2, // move by 1m along -x axis
      STEP3, // arms swivel to 0deg
      STEP4,
      STEP5,
      END // locked brakes, does nothing (used for testing)
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


        isHardcon = true;
        wheelRadius = 0.03;    // model wheel radius
        for(i=0; i<4; i++)
          isOff[i] = 1.0;
        break;
      }

      case START:
      {
        if (t > tstartmove + 2)
        {
          nState = STEP1;
        }

        break;
      }

      // move robot 1m along -x axis
      case STEP1:
      {

        // set target angle and position
        targetPosition = (1.0 / wheelRadius);
        targetAngle = M_PI / 2; // simulation only
        targetVelocity[0] = 3.0; // set target torque
        targetVelocity[1] = 3.0; // set target torque
        targetVelocity[2] = 3.0; // set target torque
        targetVelocity[3] = 3.0; // set target torque

        // set direction of rotation for wheels
        s[0] = -1.0;
        s[1] = -1.0;
        s[2] = 1.0;
        s[3] = 1.0;

        // call function to move robot
        if (moveMecanumWheel(s, u, t, d, pWheel, targetPosition, targetVelocity))
        {
          nState = STEP2; // next STEP
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
        targetVelocity[0] = 3.0; // set target torque
        targetVelocity[1] = 3.0; // set target torque
        targetVelocity[2] = 3.0; // set target torque
        targetVelocity[3] = 3.0; // set target torque

        // set direction of rotation for wheels
        // s[0] = -1.0;
        // s[1] = 0;
        // s[2] = 1.0;
        // s[3] = 0;
        s[0] = 1.0;
        s[1] = -1.0;
        s[2] = -1.0;
        s[3] = 1.0;

        // call function to move robot
        if (moveMecanumWheel(s, u, t, d, pWheel, targetPosition, targetVelocity))
        {
          nState = STEP3; // next STEP
          wait = 1;
        }
        break;
      }

       // move rdiagonally
      case STEP3:
      {

        // set target angle and position
        targetPosition = (1.5 / wheelRadius);
        targetVelocity[0] = 0; // set target torque
        targetVelocity[1] = 3.0; // set target torque
        targetVelocity[2] = 0; // set target torque
        targetVelocity[3] = 3.0; // set target torque

        s[0] = 1.0;
        s[1] = 1.0;
        s[2] = -1.0;
        s[3] = -1.0;

        // call function to move robot
        if (moveMecanumWheel(s, u, t, d, pWheel, targetPosition, targetVelocity))
        {
          nState = STEP4; // next STEP
          wait = 1;
        }
        break;
      }

            // move rdiagonally
      case STEP4:
      {

        // set target angle and position
        targetPosition = (1.0 / wheelRadius);
        targetAngle = M_PI / 2; // simulation only
        targetVelocity[0] = 3.0; // set target torque
        targetVelocity[1] = 3.0; // set target torque
        targetVelocity[2] = 3.0; // set target torque
        targetVelocity[3] = 3.0; // set target torque


        s[0] = -1.0;
        s[1] = 1.0;
        s[2] = 1.0;
        s[3] = -1.0;

        // call function to move robot
        if (moveMecanumWheel(s, u, t, d, pWheel, targetPosition, targetVelocity))
        {
          nState = STEP5; // next STEP
          wait = 1;
        }
        break;
      }

       // move rdiagonally
      case STEP5:
      {

        // set target angle and position
        targetPosition = (1.5 / wheelRadius);
        targetVelocity[0] = 3.0; // set target torque
        targetVelocity[1] = 0; // set target torque
        targetVelocity[2] = 3.0; // set target torque
        targetVelocity[3] = 0; // set target torque

        s[0] = -1.0;
        s[1] = -1.0;
        s[2] = 1.0;
        s[3] = 1.0;

        // call function to move robot
        if (moveMecanumWheel(s, u, t, d, pWheel, targetPosition, targetVelocity))
        {
          nState = END; // next STEP
        }
        break;
      }

      case END:
      { // does nothing
        pos = -1;
        printf("---------------END-----------------");
        break;
      }
    }

    pState = nState;

    printf("Step: %i\t hard_con?=%i\n", pState - 1, isHardcon);

    // PID Control for Forward/sidewards/rotations movements
    for (i = 0; i < 4; i++)
    {
      e[i] = vWheel[i] - u[i] * 1.0;
      inte[i] = inte[i] + e[i] * 0.01;
      de[i] = e[i] - ePrev[i];
      ePrev[i] = e[i];
    }

    uWheel[0] = -0.032 * (e[0]) - 0.1 * inte[0] - 0.0 * de[0]; //-0.05*sin(2*M_PI*0.25*t)+0.0;// > 0 ? 1.0 : -1.0;//-0.4*(J0.dq - 2);
    uWheel[1] = -0.032 * (e[1]) - 0.1 * inte[1] - 0.0 * de[1]; //-0.05*sin(2*M_PI*0.25*t)+0.0;// > 0 ? 1.0 : -1.0;//-0.4*(J0.dq - 2);
    uWheel[2] = -0.032 * (e[2]) - 0.1 * inte[2] - 0.0 * de[2]; //-0.05*sin(2*M_PI*0.25*t)+0.0;// > 0 ? 1.0 : -1.0;//-0.4*(J0.dq - 2);
    uWheel[3] = -0.032 * (e[3]) - 0.1 * inte[3] - 0.0 * de[3]; //-0.05*sin(2*M_PI*0.25*t)+0.0;// > 0 ? 1.0 : -1.0;//-0.4*(J0.dq - 2);

    // torque limiter (soft)
    // for (i = 0; i < 4; i++)
    // {
    //   // printf("%f\n", uArm[i]);
    //   if (fabs(uWheel[i]) > 0.6 && uWheel[i] >= 0)
    //     uWheel[i] = 0.6;
    //   else if (fabs(uWheel[i]) > 0.6 && uWheel[i] < 0)
    //     uWheel[i] = -0.6;
    // }

    // publish commands to /effort_controllers/commands
    commands.data.push_back(0);
    commands.data.push_back(0);
    commands.data.push_back(0);
    commands.data.push_back(0);

    commands.data.push_back(uWheel[0]);
    commands.data.push_back(uWheel[1]);
    commands.data.push_back(uWheel[2]);
    commands.data.push_back(uWheel[3]);

    publisher_->publish(commands);

    // commands to print topic values for testing
    print_joints(pArm, vArm, pWheel, vWheel, t, 0);
    print_commands(uArm, uWheel, t, 0);
    print_latency(t);
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
