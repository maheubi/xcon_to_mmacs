// This node is to use with a xxx.m aposs programm.
// It will connect its self with a MACS controller on the local network 
// with the specified ip which is stored in the 'my_controller_ip' string.
// It writes the values of a Xbox One S Controller in to the SDO directory:
//
//  2201/01 Emergency-Stop [0x01220101] ! =1 to start motor
//
//  2201/02 Acceleration        [0x01220102]    (0...100)
//  2201/03 Deceleration        [0x01220103]    (0...100)
//  2201/04 max turning Speed   [0x01220104]    (0...100)
//  2201/05 Pivot Limit         [0x01220105]    (1...127)
//  2201/06 Pivot Speed         [0x01220106]    (-127...127)
//  2201/07 Pivot Scale         [0x01220107]    (0...10)
//
//	2201/11	Axis 0,     Stick-Left horizontal:  [0x0122010B]    (0...127...254)
//	2201/12	Axis 1,     Stick-Left vertical:	[0x0122010C]    (0...127...254)
//	2201/13	Axis 2,     Pusher LT:		        [0x0122010D]      (254...0)
//	2201/14	Axis 3,     Stick-Right horizontal:	[0x0122010E]    (0...127...254)
//	2201/15	Axis 4,     Stick-Right vertical:	[0x0122010F]    (0...127...254)
//	2201/16	Axis 5,     Pusher RT:              [0x01220110]      (254...0)
//	2201/17	Axis 6,     Cross-Stick horizontal: [0x01220111]      (0||1||2)
//	2201/18	Axis 7,     Cross-Stick vertical:   [0x01220112]      (0||1||2)
//	2201/20	Button  0,  Button A:               [0x01220114]       (0||1)
//	2201/21	Button  1,  Button B:               [0x01220115]       (0||1)
//	2201/22	Button  2,  Button X:               [0x01220116]       (0||1)
//	2201/23	Button  3,  Button Y:               [0x01220117]       (0||1)
//	2201/24	Button  4,  Button LB:              [0x01220118]       (0||1)
//	2201/25	Button  5,  Button RB:		        [0x01220119]       (0||1)
//	2201/26	Button  6,  Button back:		    [0x0122011A]       (0||1)
//	2201/27	Button  7,  Button start:		    [0x0122011B]       (0||1)
//	2201/28	Button  8,  Button power:		    [0x0122011C]       (0||1)
//	2201/29	Button  9,  Button stick left:	    [0x0122011D]       (0||1)
//	2201/30	Button 10,  Button stick right: 	[0x0122011E]       (0||1)

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include </home/user/catkin_ws/src/xcon_to_mmacs/include/ZbCom.hpp>
#include <array>
#include <iostream>
#include <unistd.h>
#include <boost/asio.hpp>


// definition emergency/axis/buttons variable
int emerstopint;

int accwheelint, decwheelint, maxturnspeedint;
int PivYLimitint, PivSpeedint, PivScaleint;

int axis00int, axis01int, axis02int, axis03int;
int axis04int, axis05int, axis06int, axis07int;

int button00int, button01int, button02int; 
int button03int, button04int, button05int;
int button06int, button07int, button08int;
int button09int, button10int;

std::vector<uint8_t> emerstopuint8;

std::vector<uint8_t> accwheeluint8, decwheeluint8, maxturnspeeduint8;
std::vector<uint8_t> PivYLimituint8, PivSpeeduint8, PivScaleuint8;

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


// Callback function for Joystick
// updates the joy axis/buttons value as int
// writes the int values into the uint8_t vector
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
    // Emercency-Stop Simulation
    emerstopint = 1;
    emerstopuint8 = {(uint8_t)( emerstopint % 0xFF)};

    // Definition acceleration Motor 1-4 (0...100)
    accwheelint = 85;
    accwheeluint8 = {(uint8_t)( accwheelint % 0xFF)};

    // Definition deceleration Motor 1-4 (0...100)
    decwheelint = 85;
    decwheeluint8 = {(uint8_t)( decwheelint % 0xFF)};

    // define max turning speed in place (0...100)
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
    // init ros
    ros::init(argc, argv, "xctomm");

    // create node handle
    ros::NodeHandle node;

    // change this string to the controller ip on which the example is running
    std::string mycontroller_ip = "169.254.73.7";
    
    // initialize the log of the zubco library
    // this creates a file zubco.log in the same folder as the executable
    // the file will be overwritten at every start of the
    // the argument tells if the output is verbose (for debugging)
    //enable_log(log_level_error);


    // the controller used for communication and demonstration of this example
    // will be stored in this variable
    controller mycontroller;

    mycontroller = controller(mycontroller_ip, 23);
   // connect the TCP socket of this PC to the controller
    // TODO implement error if connection was unsucessful
    mycontroller.connect();



    // subscribe to /joy topic to get joystick messages
    ros::Subscriber sub = node.subscribe("joy", 100, joyCallback);

    // create a random 16-byte PDO message to confirm connection 
    std::vector<uint8_t> confconnpdo = {
	// PDO[1]: 12345678
        (uint8_t)( 120 % 0xFF), (uint8_t)(86 % 0xFF),
        (uint8_t)(52 % 0xFF), (uint8_t)(18 % 0xFF),
    };
       while(ros::ok())
    {

        mycontroller.write_sdo(0x2201, 0x01, emerstopuint8);

        mycontroller.write_sdo(0x2201, 0x02, accwheeluint8);
        mycontroller.write_sdo(0x2201, 0x03, decwheeluint8);
        mycontroller.write_sdo(0x2201, 0x04, maxturnspeeduint8);
        mycontroller.write_sdo(0x2201, 0x05, PivYLimituint8);
        mycontroller.write_sdo(0x2201, 0x06, PivSpeeduint8);
       mycontroller.write_sdo(0x2201, 0x07, PivScaleuint8);

        mycontroller.write_sdo(0x2201, 0x0B, axis00uint8);
        mycontroller.write_sdo(0x2201, 0x0C, axis01uint8);
        mycontroller.write_sdo(0x2201, 0x0D, axis02uint8);
        mycontroller.write_sdo(0x2201, 0x0E, axis03uint8);
    //    mycontroller.write_sdo(0x2201, 0x0F, axis04uint8);
    //    mycontroller.write_sdo(0x2201, 0x10, axis05uint8);
    //    mycontroller.write_sdo(0x2201, 0x11, axis06uint8);
    //    mycontroller.write_sdo(0x2201, 0x12, axis07uint8);

	    mycontroller.write_sdo(0x2201, 0x14, button00uint8);
	    mycontroller.write_sdo(0x2201, 0x15, button01uint8);
    //    mycontroller.write_sdo(0x2201, 0x16, button02uint8);
        mycontroller.write_sdo(0x2201, 0x17, button03uint8);
        mycontroller.write_sdo(0x2201, 0x18, button04uint8);
    //    mycontroller.write_sdo(0x2201, 0x19, button05uint8);
    //    mycontroller.write_sdo(0x2201, 0x1A, button06uint8);
	    mycontroller.write_sdo(0x2201, 0x1B, button07uint8);
    //    mycontroller.write_sdo(0x2201, 0x1C, button08uint8);
    	mycontroller.write_sdo(0x2201, 0x1D, button09uint8);
    //    mycontroller.write_sdo(0x2201, 0x1E, button10uint8);

        // send the PDO to the controller
        mycontroller.send_pdo(confconnpdo);

        ros::spinOnce();
        usleep(10000);
    }

    // spin until process is killed with <Ctrl + C>
    ros::spin();

  return 0; 
}
    
