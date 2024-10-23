#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include </home/user/catkin_ws/src/xcon_to_mmacs/include/ZbCom.hpp>
#include <array>
#include <iostream>
#include <unistd.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h> 
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


#include <signal.h>
#include <string>
#include <sstream>
#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>

//define SiginHanlder -> shutsdown 
void mySigintHandler(int sig)
{
    ROS_INFO("Recieved SIGINT sognal, shutting down . . .");
    ros::shutdown();
}

uint32_t millis()
{
	ros::WallTime walltime = ros::WallTime::now();
//	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	return (uint32_t)(walltime.toNSec()/1000000);
}


class MainNode
{

public:
  MainNode();

public:

  void cmdvel_callback( const geometry_msgs::Twist& twist_msg); //gives linear and angular velocitiy 
  void cmdvel_setup();

  int run();

protected:
  ros::NodeHandle n;
  controller mycontroller;

  uint32_t starttime;
  uint32_t hstimer;
  uint32_t mstimer;
  uint32_t lstimer;

  
  // cmd_vel subscriber
  ros::Subscriber cmdvel_sub;


 
  // odom publisher
  std::string base_frame;
  std::string cmdvel_topic;
  std::string joy_topic; 
  bool open_loop;
  double wheel_circumference;
  double track_width;
  int max_rpm = 100;
  float MIN_ABS_ANG_SPEED = 0.1;

};


MainNode::MainNode()  

{
  /*// CBA Read local params (from launch file)
  //ros::NodeHandle nLocal("~");
  nLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nLocal.param<std::string>("cmdvel_topic", cmdvel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
  nLocal.param<std::string>("joy_topic", joy_topic, "joy");
  ROS_INFO_STREAM("joy_topic: " << joy_topic);

  
  nLocal.param("open_loop", open_loop, true);
  ROS_INFO_STREAM("open_loop: " << open_loop);
  nLocal.param("wheel_circumference", wheel_circumference, 0.8325); // 0.3192 check this value
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nLocal.param("track_width", track_width, 0.589); // .04318 chech this value*/ 
}



void MainNode::cmdvel_callback( const geometry_msgs::Twist& twist_msg)
{

// wheel speed (m/s)

  float angular_speed = twist_msg.angular.z;
  float MIN_ABS_ANG_SPEED = 0.4;


  if (angular_speed > 0 && angular_speed < MIN_ABS_ANG_SPEED)
  {

    angular_speed = MIN_ABS_ANG_SPEED;
  }

  if (angular_speed < 0 && angular_speed > -MIN_ABS_ANG_SPEED)
  {
    angular_speed = -MIN_ABS_ANG_SPEED;
  }
  
  float right_speed = (twist_msg.linear.x + track_width * angular_speed * 2 / 2.0 * -1) * 0.25; //*0.25, because 300 instead of 75 encoder values per revolution on the controller for better resolution
  float left_speed = (-1*twist_msg.linear.x + track_width * angular_speed * 2 / 2.0 * -1) * 0.25;
#ifdef _CMDVEL_DEBUG
ROS_DEBUG_STREAM("cmdvel speed right: " << right_speed << " left: " << left_speed);
#endif


  if (open_loop)
  {
    // motor power (scale 0-1000)
    uint32_t right_rpm = right_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
    uint32_t left_rpm = left_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
    
    
    std::vector<uint8_t>  right_rpmuint8 {(uint8_t)(right_rpm % 0XFF)}; 
    std::vector<uint8_t>  left_rpmuint8 = {(uint8_t)(left_rpm % 0XFF)}; 
    

#ifdef _CMDVEL_DEBUG
ROS_DEBUG_STREAM("cmdvel power right: " << right_power << " left: " << left_power);
#endif
   
  }
  else
  {
    // motor speed (rpm)
    uint32_t right_rpm = right_speed / wheel_circumference * 60.0;
    uint32_t left_rpm = left_speed / wheel_circumference * 60.0;
    std::vector<uint8_t>  right_rpmuint8 {(uint8_t)(right_rpm % 0XFF)}; 
    std::vector<uint8_t>  left_rpmuint8 = {(uint8_t)(left_rpm % 0XFF)}; 
#ifdef _CMDVEL_DEBUG
ROS_DEBUG_STREAM("cmdvel rpm right: " << right_rpm << " left: " << left_rpm);
#endif

    ROS_INFO("cmdvel speed right:");
    mycontroller.write_sdo(0x2201, 0x0B, left_rpmuint8);
    mycontroller.write_sdo(0x2201, 0x0C, right_rpmuint8);
    
  }


}


void MainNode::cmdvel_setup()
{

 /* if (open_loop)
  {
  }
  else
  {
  }*/
  

  //ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
  cmdvel_sub = n.subscribe(cmdvel_topic, 1000, &MainNode::cmdvel_callback, this);
  //cmdjoy_sub = n.subscribe(joy_topic, 1000, &MainNode::cmdjoy_callback, this);
  //fused_imu_sub = nh.subscribe(imu_topic, 1000, &MainNode::imu_callback, this);

}

int MainNode::run()
{

	ROS_INFO("Beginning setup...");
  

	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	// create node handle
	ros::NodeHandle node;

	// change this string to the controller ip on which the example is running
	std::string mycontroller_ip = "169.254.73.";
	// initialize the log of the zubco library
	// this creates a file zubco.log in the same folder as the executable
	// the file will be overwritten at every start of the
	// the argument tells if the output is verbose (for debugging)
	enable_log(log_level_error);
	// the controller used for communication and demonstration of this example
	// will be stored in this variable
	controller mycontroller;
	mycontroller = controller(mycontroller_ip, 23);
	// connect the TCP socket of this PC to the controller
	// TODO implement error if connection was unsucessful
	mycontroller.connect();
	cmdvel_setup();
	//odom_setup();
  	starttime = millis();
  	hstimer = starttime;
  	mstimer = starttime;
  	lstimer = starttime;
  	ros::Rate loop_rate(10);
  	ROS_INFO("Beginning looping...");	
  
  
  while (ros::ok())
  {
    
    uint32_t nowtime = millis();

    if (DELTAT(nowtime,hstimer) >= 33)
    {
      hstimer = nowtime;
//      odom_hs_run();
    }

    // Handle 10 Hz publishing
    if (DELTAT(nowtime,mstimer) >= 100)
    {
      mstimer = nowtime;

    }
    // Handle 1 Hz publishing
    if (DELTAT(nowtime,lstimer) >= 1000)
    {
      lstimer = nowtime;
      //odom_ls_run();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_node");
  // init ros
  //controller mycontroller;
  MainNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}

