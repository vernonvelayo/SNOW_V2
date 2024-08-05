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

std::ofstream joint_state_file;
std::ofstream commands_file;
long time0, time1;
struct timespec spec;
static int setTime = 0;
std::ofstream latency_file_snow_con;

class MinimalSubscriber : public rclcpp::Node
{
  public:

    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      
      joint_state_file.open ("joint_state.csv");
      commands_file.open ("commands.csv");
      latency_file_snow_con.open ("latency_snow_con.csv");

      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "subscriber node created");


      //publisher
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 10);
      RCLCPP_INFO(this->get_logger(), "publisher node created");
    }
  
  private:
    void print_joints(double *pArm, double *vArm, double *pWheel, double *vWheel, double totalTime, int step) 
    {
      int i;
      /*
      
      CSV GUIDE

      Column#     Label
      1       ->  totalTime
      2-5     ->  pWheel[4]
      6-9     ->  vWheel[4]
      10-13   ->  pArm[4]
      14-17   ->  vArm[4]

      */
      joint_state_file << totalTime ;

      for(i=0; i<4; i++) 
        joint_state_file << "," << *(pWheel+i); // divide by r to get m
      for(i=0; i<4; i++) 
        joint_state_file << "," << *(vWheel+i); 
      for(i=0; i<4; i++) 
        joint_state_file << "," << *(pArm+i); 
      for(i=0; i<4; i++) 
        joint_state_file << "," << *(vArm+i); 


      //printf("\n");
      joint_state_file << "," << step; 
      joint_state_file << "\n" ; 
    }

  private:
    void print_commands(double *uArm, double *uWheel, double totalTime, int step) 
    {
      int i;
      /*
      
      CSV GUIDE

      Column#     Label
      1       ->  totalTime
      2-5     ->  uWheel[4]
      6-9     ->  uArm[4]
      10      ->  step
      */

      commands_file << totalTime ;

      for(i=0; i<4; i++) 
        commands_file << "," << *(uWheel+i); // divide by r to get m
      for(i=0; i<4; i++) 
        commands_file << "," << *(uArm+i); 


      //printf("\n");
      commands_file << "," << step; 
      commands_file << "\n" ; 
    }

    
  private:
    void print_latency(double totalTime) {
      int i, j;

      //long x = (time1.tv_sec - time0.tv_sec) + (time1.tv_nsec - time0.tv_nsec) / 1000000000.0;

      //printf("Difference: %f\n", x);
      if(!setTime) {
        time1 = 0;
        setTime = 1;
      }

      // CACLCULATION FOR DELAY
      clock_gettime(CLOCK_REALTIME, &spec);
      time0 = spec.tv_nsec;

      
      float x = (time0-time1) / 1000000.0; // after the call_back function, time1 is set. then when callback occurs again, time0 is set.

      if (x<0) 
        x = 1000 + x;
      
      //printf("%f,%f\n",totalTime,x);                     // the difference between time1 and time0 is the delay
      if (time1)
        latency_file_snow_con << totalTime << ","<< x << "\n";                  // print to csv file
      
      //print_joints()

      clock_gettime(CLOCK_REALTIME, &spec);
      time1 = spec.tv_nsec; 


    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
      double t;
      static double position = 0;
      static double velocity = 0;
      static double c;      
      static int step = 0;
      double rw = 0.044;
      int i;
      double s[] = {1,-1,1,-1};
      int pos = 0;
	
      static double d[4] = {0};
      static double v[4] = {0};
      static double dstartpos[4] = {0};
      static double dStartAngle[4] = {0};
      static double tstartmove = 0;
      static double endposition = 0;

      static bool wait = false;

      static float cVwheelRotate;
      static float cPwheelRotate;
      static float cVwheelForward;
      static float cPwheelForward;
      static float cVwheelBrake;
      static float cPArmBrake;

      static double dqD[4];
      static double dsJD[4];
      static double sJD[4];

      std_msgs::msg::Float64MultiArray commands;
      t = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec*1e-9);

      double pArm[4];
      double vArm[4];
      double pWheel[4];
      double vWheel[4];

      double uArm[4];
      double uWheel[4];
      double uWheel_0[4];


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

      uWheel_0[0] = msg->effort[1];
      uWheel_0[1] = msg->effort[2];
      uWheel_0[2] = msg->effort[5];
      uWheel_0[3] = msg->effort[7];

      ESO eso0(100,20);
      ESO eso1(100,20);
      ESO eso2(100,20);
      ESO eso3(100,20);
      
      eso0.predict(pArm[0],0,vArm[0]);
      eso1.predict(pArm[1],0,vArm[1]);
      eso2.predict(pArm[2],0,vArm[2]);
      eso3.predict(pArm[3],0,vArm[3]);


      typedef enum states
      {
        INIT, 
        START,
        STEP1,    // move by 1m along +x axis
        //STEP2,   move by 1m along -x axis
        //STEP3,   arms swivel -90deg
        //STEP4,   move by 1m along +y axis
        //STEP5,   arms swivel +45deg
        //STEP6,   robot rotates +360deg
        //STEP7,  
        WAIT
      } RSTATES;

      static RSTATES pState=INIT, nState=INIT;

      switch (pState) {
        // initialize variables
        case INIT:{
          nState = START;
          tstartmove = t;

          // equation constants
          //cVwheelRotate = 0.01;//.3; //0.0166667;
          //cPwheelRotate = 0.01; //2;  //0.1;
          cVwheelForward = 0.0104;//0.05; // 1.0;//0.05; 
          cPwheelForward = 0.0116; //0.2;//0.025;
          //cVwheelBrake;
          //cPArmBrake;
          printf("Initialize\n");

          break;
        }

        case START:{
          if (t>tstartmove + 3) {
            tstartmove = t;
            velocity = 0;
            position = M_PI/4;
            nState = STEP1;
            

            for (i = 0 ; i < 4 ; i++) {
              dstartpos[i] = pWheel[i];
              dStartAngle[i] = pArm[i];
            }

          }
          
          printf("Start Movement\n");
          break;
        }

        // robot moves forward along +x axis by 1 meter
        case STEP1: {
          
          // set direction
          s[0] = 1;
          s[1] = 1;
          s[2] = -1;
          s[3] = -1;

          double speed = 0.5; // adjust speed of the robot

          for(i = 0 ; i < 4 ; i++) {  
            //if (t < tstartmove + 4) 
            //if (d[0] >=  (-1.0 / rw) + dstartpos[0])
            if(fabs(pWheel[0] - dstartpos[0]) < (1.0 / rw)) // condition to stop at 1 meter
              v[i] = s[i] * speed / rw;
            else
              v[i] = 0;
        
            d[i] = d[i] + 0.01*v[i];

            // wheel torque and arm torque (brake)
            uWheel[i] = -cPwheelForward*(pWheel[i]- d[i]) + -cVwheelForward*(vWheel[i]- v[i]);
            uArm[i] = -0.2*(pArm[i] - 0)- 0.03*(vArm[i] - 0);

             // torque limiter
             if ((uWheel[i]/fabs(uWheel[i])) != s[i] && v[i]!= 0 ) {
              uWheel[i] = uWheel_0[i]; 
            }  
            printf("%f\t",d[0]);
            // 0.2 and 0.03

          }
          //if(1) {
          if (fabs(vWheel[0]) < 0.02 && t > tstartmove + 5) {
            //prevState = 1;
            nState = WAIT;
            tstartmove = t;
            velocity = 0;
            //position = 0;
            position = M_PI/2;
            wait = true;

            for (i = 0 ; i < 4 ; i++) {
              dstartpos[i] = pWheel[i];
            }


          }

          printf("STEP 1\n");
          break;
        }

        // Robot stops and waits for 2S
        case WAIT: {
          for(i = 0 ; i < 4 ; i++) {  
            uWheel[i] = -cPwheelForward*(pWheel[i]- dstartpos[i]) + -cVwheelForward*(vWheel[i] - 0);
            uArm[i] = - 2.5*(vArm[i] - 0);
          }

          //printf("jello");

          if ( t > tstartmove + 2) {
            wait = false;
            tstartmove = t;
            //next state here if needed
          }

          printf("WAIT\n");
          break; 
        }
      }

      if (pos == 1)  {

        dqD[0] = -0.10*(pArm[0] - sJD[0]) - 0.12*(vArm[0]*0.33-dsJD[0]);
        dqD[0] -= eso0.fHat/eso0.b;

        dqD[1] = -0.10*(pArm[1] + sJD[1]) - 0.12*(vArm[1]*0.33+dsJD[1]);
        dqD[1] -= eso1.fHat/eso1.b;

        dqD[2] = -0.10*(pArm[2] - sJD[2]) - 0.12*(vArm[2]*0.33-dsJD[2]);
        dqD[2] -= eso2.fHat/eso2.b;

        dqD[3] = -0.10*(pArm[3] + sJD[3]) - 0.12*(vArm[3]*0.33+dsJD[3]);
        dqD[3] -= eso3.fHat/eso3.b;

        for (i=0; i<4; i++) {
          uWheel[i] = dqD[i];
          uArm[i] = 0;
        }

      }


      
      if (wait) 
        pState = WAIT;
      else 
        pState = nState;


      //printf("Step: %i\n", pState-1);

      

      /* EFFORT LIMITER
      if(pState > 11) {
        for(i=0; i<4; i++) {
          if(uWheel[i] > -0.1 || uWheel[i] < 0.1) 
            uWheel[i] = 0.1;


        }
      } */
      

      commands.data.push_back(uArm[0]);
      commands.data.push_back(uArm[1]);
      commands.data.push_back(uArm[2]);
      commands.data.push_back(uArm[3]);
      commands.data.push_back( uWheel[0] );
      commands.data.push_back( uWheel[1] );
      commands.data.push_back( uWheel[2] );
      commands.data.push_back( uWheel[3]); 


      publisher_->publish(commands);

      print_joints(pArm,vArm,pWheel,vWheel,t, pState);
      print_commands(uArm, uWheel, t, pState) ;
      print_latency(t);
  
    }
    // Declaration of subscription_ attribute
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}

