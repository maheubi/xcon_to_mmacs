#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include </home/user/catkin_ws/src/xcon_to_mmacs/include/ZbCom.hpp>
#include <array>
#include <iostream>
#include <unistd.h>
#include <boost/asio.hpp>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/Joy.h>

double cmd_linearx ;
double cmd_lineary ;
double cmd_angular_z; 
int emerstopint;
int velxmax = 0.5; //m/s with parametes set in parameters
int angmax = 1.8; 

int accwheelint, decwheelint, maxturnspeedint;
int PivYLimitint, PivSpeedint, PivScaleint;

int axis00int, axis01int, axis02int, axis03int;
int axis04int, axis05int, axis06int, axis07int;

int button00int, button01int, button02int; 
int button03int, button04int, button05int;
int button06int, button07int, button08int;
int button09int, button10int;
int angular_speed, left_speed, right_speed;
int right_rpm, left_rpm, linear_speed;

std::vector<uint8_t> emerstopuint8;

std::vector<uint8_t> accwheeluint8, decwheeluint8, maxturnspeeduint8;
std::vector<uint8_t> PivYLimituint8, PivSpeeduint8, PivScaleuint8;
std::vector<uint8_t> SpeedLeftuint8, SpeedRightuint8;
std::vector<uint8_t> right_rpmuint8, left_rpmuint8;
std::vector<uint8_t> axis00uint8, axis01uint8;
std::vector<uint8_t> axis02uint8, axis03uint8;
std::vector<uint8_t> axis04uint8, axis05uint8;
std::vector<uint8_t> axis06uint8, axis07uint8;

std::vector<uint8_t> button00uint8, button01uint8;
std::vector<uint8_t> button02uint8, button03uint8;
std::vector<uint8_t> button04uint8, button05uint8;
std::vector<uint8_t> button06uint8, button07uint8;
std::vector<uint8_t> button08uint8, button09uint8;
std::vector<uint8_t> button10uint8;
 // odom publisher
std::string base_frame;
std::string cmdvel_topic;
std::string joy_topic; 
bool open_loop = true;
double wheel_circumference = 0.24;
double track_width = 0.33;
int max_rpm = 1000;
float MIN_ABS_ANG_SPEED = .1;
  
void cmdvel_callback(const geometry_msgs::Twist& twist_msg){
 //gives linear and angular velocitiy 
 // Emercency-Stop Simulation
    emerstopint = 1;
    emerstopuint8 = {(uint8_t)( emerstopint % 0xFF)};

    // Definition acceleration Motor 1-4 (0...100)
    accwheelint = 85;
    accwheeluint8 = {(uint8_t)( accwheelint % 0xFF)};

    // Definition deceleration Motor 1-4 (0...100)
    decwheelint = 85;
    decwheeluint8 = {(uint8_t)( decwheelint % 0xFF)};

    // define max turning speed in place (0...100)-+

    maxturnspeedint = 70;
    maxturnspeeduint8 = {(uint8_t)( maxturnspeedint % 0xFF)};

    // Differential Drive Pivot axis0 Limit (Velocity, 0...100)
    PivYLimitint = 1;
    PivYLimituint8 = {(uint8_t)( PivYLimitint % 0xFF)};

    // Differential Drive Pivot Speed (-127...127)
    PivSpeedint = 1;
    PivSpeeduint8 = {(uint8_t)( PivSpeedint % 0xFF)};

    // Balance scale b/w drive and pivot ( 0..10 )
    PivScaleint     = 8;
    PivScaleuint8   = {(uint8_t)( PivScaleint % 0xFF)};

    cmd_linearx = twist_msg.linear.x;
    //cmd_lineary = twist_msg.linear.y;
    cmd_angular_z = twist_msg.angular.z;
    
    // = cmd_angular_z;
     
    //right_speed =  (int)(((cmd_linearx)) + track_width * angular_speed / 2.0);
    //left_speed = (int)(((cmd_linearx)*255)-track_width * angular_speed /2.0);
    
    linear_speed =  (int)((255*cmd_linearx));
    angular_speed = (int)(127+cmd_angular_z*127);
    

    // motor speed (rpm)
    //right_rpm = (int)(127+127*right_speed / wheel_circumference * 60.0);
    //left_rpm = (int)(127+127*left_speed / wheel_circumference * 60.0);
    right_rpmuint8 = {(uint8_t)(linear_speed % 0XFF)}; 
    left_rpmuint8 = {(uint8_t)(angular_speed % 0XFF)}; 
    }
  
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
    // axis 0
    axis00int   = (int)(127+(127*joy->axes[0]));
    axis00uint8 = {(uint8_t)( axis00int % 0xFF)};
    //std::cout << "[main] value00: " << axis00int << std::endl;

    // axis 1
    axis01int = (int)(127+(127*joy->axes[1]));
    axis01uint8 = {(uint8_t)( axis01int % 0xFF)};
    
    // axis 2
    axis02int = (int)(127+(127*joy->axes[2]));
    axis02uint8 = {(uint8_t)( axis02int % 0xFF)};

    // axis 3
    axis03int     = (int)(127+(127*joy->axes[3]));
    axis03uint8 = {(uint8_t)( axis03int % 0xFF)};

    // axis 4
//    axis04int   = (int)(127+(127*joy->axes[4]));
//    axis04uint8 = {(uint8_t)( axis04int % 0xFF)};

    // axis 5
//    axis05int   = (int)(127+(127*joy->axes[5]));
//    axis05uint8 = {(uint8_t)( axis05int % 0xFF)};

    // axis 6
//    axis06int = (int)(1+(joy->axes[6]));
//    axis06uint8 = {(uint8_t)( axis06int % 0xFF)};
    
    // axis 7
//    axis07int = (int)(1+(joy->axes[7]));
//    axis07uint8 = {(uint8_t)( axis07int % 0xFF)};

    // button 0
    button00int = (int)(joy->buttons[0]);
    button00uint8 = {(uint8_t)( button00int % 0xFF)};
    
      //button 1
    button01int = (int)(joy->buttons[1]);
    button01uint8 = {(uint8_t)( button01int % 0xFF)}; 

    // button 2
//    button02int = (int)(joy->buttons[2]);
//    button02uint8 = {(uint8_t)( button02int % 0xFF)};

    // button 3
    button03int = (int)(joy->buttons[3]);
    button03uint8 = {(uint8_t)( button03int % 0xFF)};

    // button 4
    button04int = (int)(joy->buttons[4]);
    button04uint8 = {(uint8_t)( button04int % 0xFF)};

    // button 5
    //button05int = (int)(joy->buttons[5]);
    //button05uint8 = {(uint8_t)( button05int % 0xFF)};

    // button 6
//    button06int = (int)(joy->buttons[6]);
//    button06uint8 = {(uint8_t)( button06int % 0xFF)};

    // button 7
    button07int = (int)(joy->buttons[7]);
    button07uint8 = {(uint8_t)( button07int % 0xFF)};

    // button 8
//    button08int = (int)(joy->buttons[8]);
//    button08uint8 = {(uint8_t)( button08int % 0xFF)};

    // button 9
    button09int = (int)(joy->buttons[9]);
    button09uint8 = {(uint8_t)( button09int % 0xFF)};

    // button 10
//    button10int = (int)(joy->buttons[10]);
//    button10uint8 = {(uint8_t)( button10int % 0xFF)};

}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "driver");
   ros::NodeHandle node;
   //signal(SIGINT, mySigintHandler);
   
  ROS_INFO("Beginning setup...");
  //serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  // change this string to the controller ip on which the example is running
  std::string mycontroller_ip = "169.254.73.7";
  // initialize the log of the zubco library
  //enable_log(log_level_error);
  // the controller used for communication and demonstration of this example
  // will be stored in this variable
  controller mycontroller;
  mycontroller = controller(mycontroller_ip, 23);
  // connect the TCP socket of this PC to the controller
  // TODO implement error if connection was unsucessful
  mycontroller.connect();
  
  ros::Subscriber cmdvel_sub=node.subscribe("cmd_vel",100,cmdvel_callback);
      // subscribe to /joy topic to get joystick messages
  ros::Subscriber sub = node.subscribe("joy", 100, joyCallback);
  // create a random 16-byte PDO message to confirm connection 
  std::vector<uint8_t> confconnpdo = {
  // PDO[1]: 12345678
  (uint8_t)( 120 % 0xFF), (uint8_t)(86 % 0xFF),
  (uint8_t)(52 % 0xFF), (uint8_t)(18 % 0xFF),
  };
  
    
  
  	
while (ros::ok())
  {
   
 
    std_msgs::Bool msg;
    msg.data = true;
    ROS_INFO("%d", msg.data);
    ROS_INFO("right_rpm" );
    mycontroller.write_sdo(0x2201, 0x01, emerstopuint8);
    mycontroller.write_sdo(0x2201, 0x02, accwheeluint8);
    mycontroller.write_sdo(0x2201, 0x03, decwheeluint8);
    mycontroller.write_sdo(0x2201, 0x04, maxturnspeeduint8);
    mycontroller.write_sdo(0x2201, 0x05, PivYLimituint8);
    mycontroller.write_sdo(0x2201, 0x06, PivSpeeduint8);
    mycontroller.write_sdo(0x2201, 0x07, PivScaleuint8);
    mycontroller.write_sdo(0x2201, 0x1E, left_rpmuint8);
    mycontroller.write_sdo(0x2201, 0x1F, right_rpmuint8);

    mycontroller.write_sdo(0x2201, 0x0B, axis00uint8);
    mycontroller.write_sdo(0x2201, 0x0C, axis01uint8);
    mycontroller.write_sdo(0x2201, 0x0D, axis02uint8);
    mycontroller.write_sdo(0x2201, 0x0E, axis03uint8);
    mycontroller.write_sdo(0x2201, 0x14, button00uint8);
    mycontroller.write_sdo(0x2201, 0x15, button01uint8);
  //    mycontroller.write_sdo(0x2201, 0x16, button02uint8);
    mycontroller.write_sdo(0x2201, 0x17, button03uint8);
    mycontroller.write_sdo(0x2201, 0x18, button04uint8);

      // send the PDO to the controller
    mycontroller.send_pdo(confconnpdo);



    ros::spinOnce();
    usleep(10000);
  }
    
  
  ros::spin();
  return 0;
}
  

