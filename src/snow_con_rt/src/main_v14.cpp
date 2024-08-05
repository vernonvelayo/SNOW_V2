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
      
      printf("%f,%f\n",totalTime,x);                     // the difference between time1 and time0 is the delay
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
	
      static double d[4];
      static double v[4];
      static double dstartpos[4];
      static double tstartmove;
      static double endposition = 0;

      std_msgs::msg::Float64MultiArray commands;
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s' %e %e", msg->name[0].c_str(),msg->position[0],msg->position[1]);

      //t = msg->header.stamp.sec*0.50+msg->header.stamp.nanosec*1e-9;
      //printf("%ld\t", msg->header.stamp.nanosec);
      t = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec*1e-9);

      double pArm[4];
      double vArm[4];
      double pWheel[4];
      double vWheel[4];

      double uArm[4];
      double uWheel[4];

      //double check;
      
      pArm[0] = msg->position[1];
      pArm[1] = msg->position[3];
      pArm[2] = msg->position[4];
      pArm[3] = msg->position[6];

      pWheel[0] = msg->position[0];
      pWheel[1] = msg->position[2];
      pWheel[2] = msg->position[5];
      pWheel[3] = msg->position[7];

      vArm[0] = msg->velocity[1];
      vArm[1] = msg->velocity[3];
      vArm[2] = msg->velocity[4];
      vArm[3] = msg->velocity[6];

      vWheel[0] = msg->velocity[0];
      vWheel[1] = msg->velocity[2];
      vWheel[2] = msg->velocity[5];
      vWheel[3] = msg->velocity[7];
      
      

      typedef enum states
      {
        INIT, 
        START,
        STEP1,  // arms swivel +90deg
        STEP2,  // move by 1m along -x axis
        STEP3,  // arms swivel -90deg
        STEP4,  // move by 1m along +y axis
        STEP5,  // arms swivel +45deg
        STEP6,  // robot rotates +360deg
      } RSTATES;

      static RSTATES pState, nState;

      switch (pState) {
        // initialize variables
        case INIT:
          nState = START;
          break;

        case START:
          tstartmove = t;
          velocity = 0;
          position = 0;
          nState = STEP1;
          break;

        // arms will rotate 90deg
        case STEP1: {
          if (t > tstartmove && t < tstartmove + 5) 
          //if (pArm[0] < M_PI/2) 
            velocity = (M_PI/2)/4;
          else 
            velocity = 0;
        
          position += 0.01*velocity;

          for (i = 0; i < 2 ; i++)
            uWheel[i] = -.3*(pArm[i] - position) - 3*(vArm[i] - velocity);
          for (i = 2 ; i < 4 ; i++)
            uWheel[i] = -(-.3*(pArm[i] - position) - 3*(vArm[i] - velocity));

          for (i = 0 ; i < 4 ; i++) {
            if (velocity ) uArm[i] = 0; 
            else uArm[i] = -3*(pArm[i] - M_PI/2) - 3*(vArm[i] - 0); 
          }

          // if it reaches 45deg
          if (fabs(pArm[0] - M_PI/2) < 0.05 && fabs(vArm[0]) < 0.05) {
            nState = STEP2;
            tstartmove = t;
            velocity = 0;
            position = 0;

            for (i = 0 ; i < 4 ; i++) {
              d[i] = pWheel[i];
              v[i] = 0.0;
              dstartpos[i] = pWheel[i];
            }
          }
        }
      
          break;

          // robot will move 1m along -X axis
          case STEP2: {
            for(i = 0 ; i < 4 ; i++) {  
            //if (t < tstartmove + 4) 
            if (d[0] <=  (1.0 / rw) + dstartpos[0])
              v[i] = s[i]*1.0/12/rw;
              //v[i] = (1 / (2*M_PI)) / 4;
            else
              v[i] = 0;
        
            d[i] = d[i] + 0.01*v[i];

            // wheel torque
            uWheel[i] = -0.2*(pWheel[i]- d[i]) + -1.1*(vWheel[i]- v[i]);
            // arm torque (brake)
            uArm[i] = -2*(pArm[i] - M_PI/2) - 2.5*(vArm[i] - 0); 
          }
          
          printf("%f %f \t", -0.05*(pWheel[0]- d[0]), -0.4*(vWheel[0]- v[0]));
          //printf("torq = %f \t", uWheel[0]);

          if (fabs(vWheel[0]) < 0.02 && t > tstartmove + 15) {
            nState = STEP3;
            tstartmove = t;
            velocity = 0;
            position = M_PI/2;

          }
          break;
        }

        // robot will swivel back 90 degrees (return to default arm angle)
        case STEP3: {
          
          if (t > tstartmove && t < tstartmove + 4) 
            velocity = -(M_PI/2)/4;
          else 
            velocity = 0;
        
          position += 0.01*velocity;

          for (i = 0 ; i < 2 ; i++)
            uWheel[i] = -.3*(pArm[i] - position) - 2*(vArm[i] - velocity);
          for (i = 2 ; i < 4 ; i++)
            uWheel[i] = -(-.3*(pArm[i] - position) - 2*(vArm[i] - velocity));

          for (i = 0 ; i < 4 ; i++) {
            if (velocity != 0) uArm[i] = 0; 
            else uArm[i] = -2*(pArm[i] - 0) - 2*(vArm[i] - 0); 
          }

          if (fabs(pArm[0]) < 0.02 && t > tstartmove + 8) {
            tstartmove = t;
            nState = STEP4;
            for (i = 0 ; i < 4 ; i++) {
              d[i] = pWheel[i];
              v[i] = 0.0;
              dstartpos[i] = pWheel[i];
            }
          }
            
          break;
        }

        // robot moves forward by 1m along +Y axis
        case STEP4: {
         
          s[0] = 1;
          s[1] = 1;
          s[2] = 1;
          s[3] = 1;

          for(i = 0 ; i < 4 ; i++) {  
            //if (t < tstartmove + 4) 
            if (d[0] <=  (1.0 / rw) + dstartpos[0])
              v[i] = s[i]*1.0/12/rw;
              //v[i] = (1 / (2*M_PI)) / 4;
            else
              v[i] = 0;
        
            d[i] = d[i] + 0.01*v[i];

            // wheel torque and arm torque (brake)
            uWheel[i] = -0.2*(pWheel[i]- d[i]) + -1.1*(vWheel[i]- v[i]);
            uArm[i] = -2*(pArm[i] - 0) - 2.5*(vArm[i] - 0); 
          }
          
          printf("%f %f \t", -0.05*(pWheel[0]- d[0]), -0.4*(vWheel[0]- v[0]));
          //printf("torq = %f \t", uWheel[0]);

          if (fabs(vWheel[0]) < 0.02 && t > tstartmove + 15) {
            nState = STEP5;
            tstartmove = t;
            velocity = 0;
            position = 0;

          }

          break;
        }

        // arms will swivel by 45deg
        case STEP5: {
          if (t > tstartmove && t < tstartmove + 3) 
            velocity = 0;//-(M_PI/4)/4;
          else 
            velocity = 0;
          
          position = M_PI/4; // += 0.01*velocity;
          
          uWheel[0] =  (-1*(pArm[0] - position) - 3*(vArm[0] - velocity));
          uWheel[1] =  (-1*(pArm[1] - position) - 3*(vArm[1] - velocity));
          uWheel[2] = -(-1*(pArm[2] - position) - 3*(vArm[2] - velocity));
          uWheel[3] = -(-1*(pArm[3] - position) - 3*(vArm[3] - velocity)); 


          for (i = 0 ; i < 4 ; i++)
            //uArm[i] = 0; 
            uArm[i] = -2*(pArm[i] - M_PI/4) - 2*(vArm[i] - 0); 

          if (fabs(pArm[0] - M_PI/4) < 0.02 ) {
            velocity = 0;
            position = 0;
            nState = STEP6;

            endposition = 2*M_PI*(0.120+0.18175);

            for (i = 0 ; i < 4 ; i++) {
              dstartpos[i] = pWheel[i];
                
              d[i] = pWheel[i];
              v[i] = 0.0;
              dstartpos[i] = pWheel[i];
          
              }
          }
          break;
        }

        case STEP6: {
          // robot will spin 360 degs
      
          s[0] = -1;
          s[1] = 1;
          s[2] = 1;
          s[3] = -1;


          for(i = 0 ; i < 4 ; i++) {  
            //if (t < tstartmove + 4) 
            if (d[1] <=  (endposition / rw) + dstartpos[1])
              v[i] = s[i]*endposition/10/rw;
            else 
              v[i] = 0;
              
            d[i] = d[i] + 0.01*v[i];

            // wheel torque
            uWheel[i] = -0.2*(pWheel[i]- d[i]) + -1.1*(vWheel[i] - v[i]);
            // arm torque (brake)
            uArm[i] = -2*(pArm[i] - M_PI/4) - 3*(vArm[i] - 0); 
          } 

          if (fabs(vWheel[0]) < 0.02 && t > tstartmove + 15) {
            //nState = STEP6;
            tstartmove = t;
            velocity = 0;
            position = 0;

          }
          break;
        }

      }

      pState = nState;


      printf("Step: %i\n", nState);

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

