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
//#include "builtin_interfaces/msg/Time.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;
double pWheel[4], vWheel[4];
double pArm[4],   vArm[4];
double uWheel[4], uArm[4];
long time0, time1;
struct timespec spec;
static int setTime = 0;
std::ofstream latency_file;
//std::ofstream joint_state_file;

double com[8];

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      
      latency_file.open ("latency.csv");

      //subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
       // "/effort_controllers/commands", 10);
      
      publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
      pub_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::joint_callback, this));
 
      


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
        latency_file << totalTime << ","<< x << "\n";                  // print to csv file
      
      //print_joints()

      clock_gettime(CLOCK_REALTIME, &spec);
      time1 = spec.tv_nsec; 


    }
    

  private:
    void joint_callback()
    {
      

      //auto message = std_msgs::msg::String();
      //message.data = "Hello, world! " + std::to_string(count_++);
      static int state = 0;
      int i = 0;

      std::string label[8] = {
        "wm1_base_to_steering_axis", 
        "wm1_arm_to_wheel", 
        "wm2_arm_to_wheel",
        "wm2_base_to_steering_axis",
        "wm3_base_to_steering_axis",
        "wm3_arm_to_wheel",
        "wm4_base_to_steering_axis",
        "wm4_arm_to_wheel", 
        
      };

      int order[8] = {0,1,1,0,0,1,0,1}; // 0-arm , 1-wheel
      auto jt = sensor_msgs::msg::JointState(); 
      auto header = std_msgs::msg::Header(); 
      auto time = builtin_interfaces::msg::Time();

      // set seconds and nanoseconds
      double totalTime = (count_*0.010);
      int nsecs = (int(totalTime*1000.0) % 1000) * 1000000;
      printf("%f %i\n", totalTime,nsecs);
      
      int secs = totalTime;

      // set msg header
      time.sec = secs;
      time.nanosec = nsecs;
      header.stamp = time;
      header.frame_id = "hard_con"; 
      

      //jt.header.push_back(header); 
      //if(!com.empty()) {
        //printf("Command: %f %ld \t\ts: %i \tnsecs: %i \n", com[0], count_, secs, nsecs);
      //}
      
      // initialize
      if(!state) {
        for(i = 0; i < 4; i++) {
          pWheel[i] = 0;
          vWheel[i] = 0; 
          pArm[i]   = 0;
          vArm[i]   = 0;
          uWheel[i] = 0;
          uArm[i]   = 0;
          state = 1;
        }
      }
      else {
        for(i = 0; i < 4; i++) {
          pArm[i]   = pArm[i]   + 0.01*(vArm[i]);
          pWheel[i] = pWheel[i] + 0.01*(vWheel[i]);
          vArm[i]   = vArm[i]   + 0.01*(uArm[i]);
          vWheel[i] = vWheel[i] + 0.01*(uWheel[i]); 
        }
      }

      int a = 0, b = 0;
      jt.header = header;

      for(i = 0; i < 8; i++) {
        jt.name.push_back(label[i]);
        
        if(order[i]) {
          jt.position.push_back(pWheel[a]);
          jt.velocity.push_back(vWheel[a]);
          a++;
        }

        else {
          jt.position.push_back(pArm[b]);
          jt.velocity.push_back(vArm[b]); 
          b++;
        }

        if(i < 4) {
          jt.effort.push_back(uArm[i]);
        }

        else {
          jt.effort.push_back(uWheel[i-4]);
        }
        
      }

      
      
 
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      count_++;
      publisher_->publish(jt);

      print_latency(totalTime);

      
    }



    rclcpp::TimerBase::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
   // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    size_t count_;

    
};

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/effort_controllers/commands", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr commands) const
    {
      int i = 0;
      for (i=0; i < 8; i++) {
        //com[i] = commands->data[i];
        //
        if (i < 4) 
          uArm[i] = commands->data[i];
        else 
          uWheel[i-4] = commands->data[i];

        //printf("%0.4f \t", commands->data[i]);
      }
      //printf("\n");

      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::Node::SharedPtr subNode = 
    std::make_shared<MinimalSubscriber>();
  rclcpp::Node::SharedPtr pubNode = 
    std::make_shared<MinimalPublisher>();

  executor.add_node(subNode);
  executor.add_node(pubNode);
  executor.spin();

  rclcpp::shutdown();
  return 0;

  printf("je");
}