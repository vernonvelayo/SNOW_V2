#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscriber node created");

    // publisher
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands", 10);
    RCLCPP_INFO(this->get_logger(), "publisher node created");
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    double t;
    static double tStart = 0;
    static int startFlag = 0;
    static double position = 0;
    static double velocity = 0;
    static int step = 0;
    double rw = 0.044;

    static double d[4];
    static double v[4];
    static double tstartmove;

    std_msgs::msg::Float64MultiArray commands;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s' %e %e", msg->name[0].c_str(),msg->position[0],msg->position[1]);

    t = msg->header.stamp.sec * 0.50 + msg->header.stamp.nanosec * 1e-9 - tStart;

    double pArm[4];
    double vArm[4];
    double pWheel[4];
    double vWheel[4];
    double uArm[4];
    double uWheel[4];

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

    int i;
    double stopPosition[4];
    double tempTime = 0;
    double s[] = {1, -1, 1, -1};

    typedef enum states
    {
      INIT,
      START,
      STEP1,
      STEP2,
      STEP3,
      STEP4,
      STEP5,
      STEP6,
      STEP7,
    } RSTATES;

    static RSTATES pState, nState;

    

    //
    switch (pState)
    {
      // initialize variables
      case INIT:
        nState = START;
        break;

      case START:
        startFlag = 1;
        tstartmove = t;
        nState = STEP1;
        break;

      // robot will rotate its arms by 90deg
      case STEP1:
      {
        if (t > tstartmove && t < tstartmove + 2)
          velocity = (M_PI/2)/4;
        else 
          velocity = 0;
      
        position += 0.01*velocity;

        for (i = 0; i < 2 ; i++)
          uWheel[i] = -.3*(pArm[i] - position) - 3*(vArm[i] - velocity);
        for (i = 2 ; i < 4 ; i++)
          uWheel[i] = -(-.3*(pArm[i] - position) - 3*(vArm[i] - velocity));


        for (i = 0 ; i < 4 ; i++)
          uArm[i] = 0; 

        //if (pArm[0] >= M_PI/2)  
         //if (fabs(pArm[0] - M_PI/2) < 0.02)
        if (fabs(pArm[0] - M_PI/2) < 0.02) {
          nStep = STEP2;
          tstartmove = t;
        }

        //check = fabs(pArm[0] - M_PI/2);
        //printf("%lf %d %lf\n",t, step, check);
          
        for (i = 0 ; i < 4 ; i++) {
          d[i] = pWheel[i];
          v[i] = 0.0;
        }
        break;
      }

      // robot will move forward in 1 m
      case STEP2:
      {
        for(i = 0 ; i < 4 ; i++) {  
          if (t < tstartmove + 5) 
            v[i] = s[i]*1.0/10/rw;
          else
            v[i] = 0;
      
          d[i] = d[i] + 0.01*v[i];
        }
        // move forward after 90deg
        uWheel[0] = -0.1*(pWheel[0]- d[0]) + -0.2*(vWheel[0]- v[0]);
        uWheel[1] = -0.1*(pWheel[1]- d[1]) + -0.2*(vWheel[1]- v[1]);
        uWheel[2] = -0.1*(pWheel[2]- d[2]) + -0.2*(vWheel[2]- v[2]);
        uWheel[3] = -0.1*(pWheel[3]- d[3]) + -0.2*(vWheel[3]- v[3]);
        //printf("%lf %lf %lf\n",t, d[0],v[0]);

        for (i = 0 ; i < 4 ; i++)
          uArm[i] = -2*(pArm[i] - M_PI/2) - 2*(vArm[i] - 0); 


        if (fabs(vWheel[0]) < 0.02) {
          step = 2;
        }
        
        break;
      }

      // robot will stop
      case STEP3: 
      {
        uWheel[0] = -(.17 * (vWheel[0] - -2));
        uWheel[1] = (.17 * (vWheel[0] - -2));
        uWheel[2] = -(.17 * (vWheel[0] - -2));
        uWheel[3] = (.17 * (vWheel[0] - -2));

        for (i = 0; i < 4; i++)
          uArm[i] = -2 * (pArm[i] - M_PI / 2) - 2 * (vArm[i] - 0);

        if (vWheel[0] <= 0)
        {
          step = 3;
          tempTime = t;
        }
        break;
      }
    
      default:
        nState = START;
        break;

    }
    pState = nState;

    //________________________________________________________________
    /*
    // robot will rotate axis by 90deg
    if (!step)
    {
    }

    // robot will move forward for 25 units
    else if (step == 1)
    {
    }

    // robot will attempt to stop
    else if (step == 2)
    {

      
    }

    else if (step == 4)
    {
      // robot will swivel back 90 degrees (return to default position)

      if (tempTime > 0 && tempTime < 2)
      {
        velocity = 0;
      }

      else
      {
        velocity = 0;
      }

      position += 0.01 * velocity;

      for (i = 0; i < 2; i++)
        uWheel[i] = -.3 * (pArm[i] - position) - 3 * (vArm[i] - velocity);
      for (i = 2; i < 4; i++)
        uWheel[i] = -(-.3 * (pArm[i] - position) - 3 * (vArm[i] - velocity));

      for (i = 0; i < 4; i++)
        uArm[i] = 0;

      // if (pArm[0] >= M_PI/2)
      // if (fabs(pArm[0] - M_PI/2) < 0.02)
      if (fabs(pArm[0]) < 0.02)
      {
        tstartmove = t;
        step = 5;
        tStart = 1;
      }

      // check = fabs(pArm[0] - M_PI/2);
      // printf("%lf %d %lf\n",t, step, check);

      for (i = 0; i < 4; i++)
      {
        d[i] = pWheel[i];
        v[i] = 0.0;
      }
    }

    else if (step == 5)
    {
      // move forward

      s[0] = -1;
      s[1] = 1;
      s[2] = -1;
      s[3] = 1;

      for (i = 0; i < 4; i++)
      {
        if (t < tstartmove + 15)
          v[i] = s[i] * 1.0 / 10 / rw;
        else
          v[i] = 0;

        d[i] = d[i] + 0.01 * v[i];
      }
      // move forward after 90deg
      uWheel[0] = -0.1 * (pWheel[0] - d[0]) + -0.1 * (vWheel[0] - v[0]);
      uWheel[1] = -0.1 * (pWheel[1] - d[1]) + -0.1 * (vWheel[1] - v[1]);
      uWheel[2] = -0.1 * (pWheel[2] - d[2]) + -0.1 * (vWheel[2] - v[2]);
      uWheel[3] = -0.1 * (pWheel[3] - d[3]) + -0.1 * (vWheel[3] - v[3]);

      for (i = 0; i < 4; i++)
        uArm[i] = -2 * (pArm[i] - 0) - 2 * (vArm[i] - 0);

      if (tstartmove + 25)
      {

        step = 6;
      }
    }

    else if (step == 6)
    {
      // robot arms will swivel 45deg (NOT FINISHED)

      if (t > tempTime && t < tempTime + 2)
      {
        velocity = 0; //-(M_PI/4)/4;
      }

      else
      {
        velocity = 0;
      }

      position = M_PI / 4; // += 0.01*velocity;

      uWheel[0] = (-1 * (pArm[0] - position) - 3 * (vArm[0] - velocity));
      uWheel[1] = (-1 * (pArm[1] - position) - 3 * (vArm[1] - velocity));
      uWheel[2] = -(-1 * (pArm[2] - position) - 3 * (vArm[2] - velocity));
      uWheel[3] = -(-1 * (pArm[3] - position) - 3 * (vArm[3] - velocity));

      for (i = 0; i < 4; i++)
        // uArm[i] = 0;
        uArm[i] = -2 * (pArm[i] - M_PI / 4) - 2 * (vArm[i] - 0);

      if (pArm[0] <= M_PI / 4)
      {
        // if (t >= tempTime + 8) {
        tempTime = t;
        velocity = 0;
        position = 0;
        step = 7;
        // tStart = 1;
        for (i = 0; i < 4; i++)
        {
          stopPosition[i] = pWheel[i];
        }
      }
    }

    else if (step == 7)
    {
      // robot will spin 360 degs

      /*
      if (t > tempTime && t < tempTime + 6) {
        // distance to circle around in radians: 135.36
        velocity = 22.56; // 135.36 / 6 seconds = 22.56
      }

      else {
        velocity = 0;
      }

      position += 0.01*velocity; 

      uWheel[0] = -1 * (pArm[0] - M_PI / 4) - 3 * (vArm[0] - 0) + 1;
      uWheel[1] = -1 * (pArm[1] - M_PI / 4) - 3 * (vArm[1] - 0) + -1;
      uWheel[2] = -(-1 * (pArm[2] - M_PI / 4) - 3 * (vArm[2] - 0)) + -1;
      uWheel[3] = -(-1 * (pArm[3] - M_PI / 4) - 3 * (vArm[3] - 0)) + 1;

      for (i = 0; i < 4; i++)
        uArm[i] = -2 * (pArm[i] - M_PI / 4) - 2 * (vArm[i] - 0);

      if (pWheel[0] >= stopPosition[0] + 75)
      {
        step = 8;
      }
    }

    else if (step == 8)
    {
      // robot will attempt to stop
      uWheel[0] = -(.17 * (vWheel[0] - -1));
      uWheel[1] = (.17 * (vWheel[0] - -1));
      uWheel[2] = (.17 * (vWheel[0] - -1));
      uWheel[3] = -(.17 * (vWheel[0] - -1));

      for (i = 0; i < 4; i++)
        uArm[i] = -2 * (pArm[i] - M_PI / 4) - 3 * (vArm[i] - 0);

      if (vWheel[0] <= 0)
      {
        step = 9;
        tempTime = t;
      }
    }

    else if (step == 9)
    {
      // robot will stop for 2 seconds
      uWheel[0] = -(.17 * (vWheel[0] - 0));
      uWheel[1] = (.17 * (vWheel[0] - 0));
      uWheel[2] = -(.17 * (vWheel[0] - 0));
      uWheel[3] = (.17 * (vWheel[0] - 0));

      for (i = 0; i < 4; i++)
        uArm[i] = -2 * (pArm[i] - M_PI / 2) - 2 * (vArm[i] - 0);

      if (tempTime + 2 <= t)
      {
        tempTime = t;
        // step = 8;
        velocity = 0;
        position = 0;
      }
    }

    */

    commands.data.push_back(uArm[0]);
    commands.data.push_back(uArm[1]);
    commands.data.push_back(uArm[2]);
    commands.data.push_back(uArm[3]);
    commands.data.push_back(uWheel[0]);
    commands.data.push_back(uWheel[1]);
    commands.data.push_back(uWheel[2]);
    commands.data.push_back(uWheel[3]);

    publisher_->publish(commands);
  }
  // Declaration of subscription_ attribute
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  // Declaration of the publisher_ attribute
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
