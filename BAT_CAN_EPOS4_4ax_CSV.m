////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Differential Steering Joystick Algorithm
// ========================================
//
//   by Marco Stössel @ HSLU T&A for maxon Group
//
// Program for four axis using EPOS4 in CSV mode. All axis are connected to the CAN1.
// Speed and Direction Inputs are read from the SDO directory.
// The program is for controlling maxon motors by an Xbox One S Controller.
// The controller should be connected to a Linux computer which launches the ROS-node xcon_to_mmacs
//
// For operation with linear velocity commands, the Y velocity is set vertically with
// the right-hand joystick.
// Movements in X-directions are made with the left joystick along the horizontal axis.
//
// Operation with logarithmic speed commands is set by pressing the A button continuously.
// The speed setting in positive Y direction is set with the RT slider. For a negative Y speed,
// the RB button must be pressed.
// Movements in X-directions are made with the left joystick via the horizontal axis.
//
// For reliable operation, the Xbox Controller should be freshly calibrated before operation.
// (jstest-gtk application as example)
// Make sure that the LT and RT sliders deliver the value 254 in unactuated state.
// Buttons and axes should send 0 in unactuated state.
// Otherwise change the controller configuration on your Linux computer.
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////


#include <sysdef.mi>

#define C_VELMAX    2000		 // set max velocity
#define C_ENC_QC    4096*4       // set encoder to correct value

#include "BAT_SetupCAN_EPOS4_4ax_CSV.mi"

print " BAT Try program for four EPOS4 on CAN bus in CSV mode"

delay 2000				// wait till EPOS woke up

SYS_CANOPMASTER = 0  	// preop and disable master functionality


SetupDrive_EPOS4(1, 2, 1, C_VELMAX)    // SetupDrive_EPOS4(AxisNo, CanId, PdoNumber, C_VELMAX)
SetupDrive_EPOS4(2, 3, 1, C_VELMAX)    // SetupDrive_EPOS4(AxisNo, CanId, PdoNumber, C_VELMAX)

SetupDrive_EPOS4(3, 4, 1, C_VELMAX)    // SetupDrive_EPOS4(AxisNo, CanId, PdoNumber, C_VELMAX)
SetupDrive_EPOS4(4, 5, 1, C_VELMAX)    // SetupDrive_EPOS4(AxisNo, CanId, PdoNumber, C_VELMAX)

set cansynctimer  CAN_TIMER  // sync every CAN_TIMER ms
SYS_CANOPMASTER = 1  // operational and enable master functionality

errclr
nowait on

on error gosub errorhandler
//
//
//// record (internal scope) some parameter
//AxisNo = 1
//TESTSETINDEX INDEX_REG_COMPOS(AxisNo), INDEX_REG_ACTPOS(AxisNo), INDEX_REG_TRACKERR(AxisNo), INDEX_AMP_GETCUR(AxisNo)
//DELAY 1000
//TESTSTART 0   // start recording
//
//
//DELAY 100
//
//AxisNo = 2
//TESTSETINDEX INDEX_REG_COMPOS(AxisNo), INDEX_REG_ACTPOS(AxisNo), INDEX_REG_TRACKERR(AxisNo), INDEX_AMP_GETCUR(AxisNo)
//DELAY 1000
//TESTSTART 0   // start recording
//
//DELAY 100
//
//AxisNo = 3
//TESTSETINDEX INDEX_REG_COMPOS(AxisNo), INDEX_REG_ACTPOS(AxisNo), INDEX_REG_TRACKERR(AxisNo), INDEX_AMP_GETCUR(AxisNo)
//DELAY 1000
//TESTSTART 0   // start recording
//
//DELAY 100
//
//AxisNo = 4
//TESTSETINDEX INDEX_REG_COMPOS(AxisNo), INDEX_REG_ACTPOS(AxisNo), INDEX_REG_TRACKERR(AxisNo), INDEX_AMP_GETCUR(AxisNo)
//DELAY 1000
//TESTSTART 0   // start recording
//
//DELAY 100



////////////////////////////////////////////////////////////////////
//
// State Machine
// 0 -> Init	1 -> Init+	2 -> Run	3 -> Connection lost	4 -> Emergency-Off
Robostate	= 0
// Heartbeat-Check
heartbeatcheck	= 0


while (1) do
//////////
INIT:

//////////
//Init
   if (Robostate < 2) then

		// time to write the SDO-Values
		delay 100

		////////////////////////////////////////////////////////////////////
		//
		// Direction of Travel, right:	Front Motor	-> x(1) [Nodeid/CANId=2]
		//								Back Motor	-> x(2) [Nodeid/CANId=3]
		// Direction of Travel, left:	Front Motor	-> x(4) [Nodeid/CANId=5]
		//								Back Motor	-> x(3) [Nodeid/CANId=4]
		// Definition acceleration und deceleration Motor 1-4

		// accelerationwheel	= 85
		 accelerationwheel	= sysvar[0x01220102]
		//decelerationwheel	= 85
		 decelerationwheel	= sysvar[0x01220103]
		acc x(1) accelerationwheel	x(2) accelerationwheel	x(3) accelerationwheel	x(4) accelerationwheel
		dec x(1) decelerationwheel	x(2) decelerationwheel	x(3) decelerationwheel	x(4) decelerationwheel

		// define max turning speed in place
		// maxturnspeed	= 70
		maxturnspeed	= sysvar[0x01220104]

		// Def Origin Motors
		deforigin	x(1)	x(2)	x(3)	x(4)
		// Motor ready
		motor on	x(1)	x(2)	x(3)	x(4)

/////
   		// StateMachine to Run
		if (sysvar[0x01220101] == 1) then
			Robostate	=Robostate + 1
		// StateMachine to Emergency-Off
//		elseif ((Robostate > 0) AND (sysvar[0x01220101] == 0)) then
//			Robostate	= 4
		endif


//////////
//RUN
	elseif (Robostate == 2) then

		// was once alife
		heartbeatcheck		= 1

		// Velocity bevore
		velocity_left_k1	= 0
		velocity_right_k1	= 0

		///////////////////////////////////
		// Logical Prozess
		///////////////////////////////////
		START:

			// while Emergency-Release = true
			while (sysvar[0x01220101] == 1 AND (Robostate == 2)) do

   				// check time since last PDO -> lost if time > 1sec
   				if (heartbeatcheck == 1 AND (sysvar[0x01140004] > 1000)) then
	   				Robostate = 3
   				endif

				// Start-up motors
				cstart	x(1)	x(2)	x(3)	x(4)

				///////////////////////////////////
				// Differential Drive
				///////////////////////////////////

				// INPUTS
				axis0	= sysvar[0x0122010B]-127	// Axis 0, Stick-Left horizontal
				axis4	= sysvar[0x0122010F]-127    // Axis 4, Stick-Right vertical
				axis5	= 254-sysvar[0x01220110]	// Axis 5, Pusher RT
				button0	= sysvar[0x01220114]		// Button 0, Button A
				button5	= sysvar[0x01220119]		// Button 5, Button RB

				// speed threshold -> if controller isnt well calibratet
				//fPivYLimit = 1					(1...127)
				fPivYLimit	= sysvar[0x01220105]
				// Pivot Speed						(-127..+127)
				//nPivSpeed	= 1
				nPivSpeed	= sysvar[0x01220106]
				// Balance scale b/w drive and pivot ( 0..1 )
				//fPivScale	= 0.8
				fPivScale	= sysvar[0x01220107]%10

				///////////////////////////////////
				// Logarithm Speed Mode
				if (button0 == 1) then

					fPivYLimit = 1
					velsca	= axis5

					// forward in log-Mode
					// 0 - 5 in 80 Steps
					if (velsca < 80) then
						vellog = velsca*5%79
					// 5 - 20 in 75 Steps
					elseif ((velsca >= 80)&(velsca < 155)) then
						vellog = (velsca * 15 % 74) - 11
					// 20 - 45 in 50 Steps
					elseif ((velsca >= 155)&(velsca < 205)) then
						vellog = (velsca * 25 % 50) - 57.5
					// 45 - 65 in 25 Steps
					elseif ((velsca >= 205)&(velsca < 230)) then
						vellog = (velsca * 35 % 24) - 253.95
					// 65 - 100 in 25 Steps
					elseif (velsca >= 230) then
						vellog = (velsca * 40 % 24) - 303.33
					endif

					// backward log-Mode withRB
					if (button5 == 1) then
						vellog	= -vellog
					endif

					if (axis4 >= 0) then
						if (axis0>=0) then
	  						nMotPremixL = (127 - axis0)
	  						nMotPremixR = 127
	  					else
	  						nMotPremixL = 127
	  						nMotPremixR = (127 + axis0)
	  					endif
	  				else
						if (axis0>=0) then
	  						nMotPremixL = 127
	  						nMotPremixR = (127 - axis0)
	  					else
	  						nMotPremixL = (127 + axis0)
	  						nMotPremixR = 127
	  					endif
					endif

					// Scale Drive output due to Joystick axis4 input (throttle)
					nMotPremixL = nMotPremixL * vellog%127
					nMotPremixR = nMotPremixR * vellog%127

					// Now calculate pivot amount
					// - Strength of pivot (nPivSpeed) based on Joystick axis0 input
					// - Blending of pivot vs drive (fPivScale) based on Joystick axis4 input
					nPivSpeed = axis0
					if (abs(vellog)>fPivYLimit) then
						fPivScale = 0
					else
						fPivScale = (1.0 - abs(vellog)%fPivYLimit)
					endif

					// Calculate final mix of Drive and Pivot
					velocity_left	= (1.0-fPivScale)*nMotPremixL + fPivScale*(-nPivSpeed)
					velocity_right	= (1.0-fPivScale)*nMotPremixR + fPivScale*( nPivSpeed)

					// define max turning speed in place
					if (axis5 == 0) then
						if (velocity_right > maxturnspeed)	then
							velocity_right = maxturnspeed
						elseif (velocity_right < -maxturnspeed) then
							velocity_right = -maxturnspeed
						endif
						if (velocity_left > maxturnspeed) then
							velocity_left = maxturnspeed
						elseif (velocity_left < -maxturnspeed) then
							velocity_left = -maxturnspeed
						endif
					endif


				else
				///////////////////////////////////
				// Linear Speed
					if (axis4 >= 0) then
					//forward
						if (axis0>=0) then
	  						nMotPremixL = (127 - axis0)
	  						nMotPremixR = 127
	  					else
	  						nMotPremixL = 127
	  						nMotPremixR = (127 + axis0)
	  					endif
	  				else
	  				// backward
						if (axis0>=0) then
	  						nMotPremixL = 127
	  						nMotPremixR = (127 - axis0)
	  					else
	  						nMotPremixL = (127 + axis0)
	  						nMotPremixR = 127
	  					endif
					endif

					// Scale Drive output due to Joystick axis4 input (throttle)
					nMotPremixL = nMotPremixL * axis4%127
					nMotPremixR = nMotPremixR * axis4%127

					// Now calculate pivot amount
					// - Strength of pivot (nPivSpeed) based on Joystick axis0 input
					// - Blending of pivot vs drive (fPivScale) based on Joystick axis0 input
					nPivSpeed = axis0
					if (abs(axis4)>fPivYLimit) then
						fPivScale = 0
					else
						fPivScale = (1.0 - abs(axis4)%fPivYLimit)
					endif

					// Calculate final mix of Drive and Pivot
					velocity_left	= (1.0-fPivScale)*nMotPremixL + fPivScale*(-nPivSpeed)
					velocity_right	= (1.0-fPivScale)*nMotPremixR + fPivScale*( nPivSpeed)
					//print " velocity_left	lin = ", velocity_left, " (1.0 -",fPivScale, ") * ",nMotPremixL, " + ",fPivScale," * ",nPivSpeed
					//print " velocity_right 	lin = ", velocity_right, " (1.0 -",fPivScale, ") * ",nMotPremixR, " + ",fPivScale," * ",nPivSpeed

					// define max turning speed in place
					if (axis4 == 0) then
						if (velocity_right > maxturnspeed)	then
							velocity_right = maxturnspeed
						elseif (velocity_right < -maxturnspeed) then
							velocity_right = -maxturnspeed
						endif
						if (velocity_left > maxturnspeed) then
							velocity_left = maxturnspeed
						elseif (velocity_left < -maxturnspeed) then
							velocity_left = -maxturnspeed
						endif
					endif

				endif



				///////////////////////////////////
				// set final Velocity for Robot
				//
				// Direction of Travel, right:	Front Motor -> x(1) Back Motor -> x(2)
				// Direction of Travel, left:	Front Motor -> x(4) Back Motor -> x(3)
				cvel		x(1) -velocity_right		x(2) -velocity_right		x(3) velocity_left	x(4) velocity_left

				if (velocity_left_k1!=velocity_left) then
				//	print " Aktuelle Geschwindigkeit in % von max. Geschw_left.: ",velocity_left
			    endif
			    velocity_left_k1 = velocity_left

				if (velocity_right_k1!=velocity_right) then
				//	print " Aktuelle Geschwindigkeit in % von max. Geschw_right.: ",velocity_right
			    endif
			    velocity_right_k1 = velocity_right

			endwhile

			// Velocity 0 if Emergency-Stop
			if (sysvar[0x01220101] != 1) then
				cvel		x(1) 0		x(2) 0		x(3) 0	x(4) 0
				Robostate	= 4
			endif

			// Check Robotstate
			if (Robostate != 2) then
				GOTO INIT
			endif

		GOTO START

//////////
//Connection lost
	elseif (Robostate == 3) then
		// print Problem as long as not connected
		while (sysvar[0x01140004] > 1000) do
			cvel		x(1) 0		x(2) 0		x(3) 0	x(4) 0
			print " Connection lost !!! "
			Robostate	= 0
			delay 3000
		endwhile

//////////
//Emergency-Off
	elseif (Robostate == 4) then
		// print Problem till restart
		while (Robostate == 4) do
			cvel		x(1) 0		x(2) 0		x(3) 0	x(4) 0
			motor off	x(1)		x(2)		x(3)	x(4)
			print " Emergency-Off!!! "
			delay 3000
		endwhile

//////////
	endif
endwhile




////////////////////////////////////////////////////////////////////////
submainprog

subprog errorhandler

   print " !!!!! ERROR: ",errno," AxisNo: ",errax
   if (errno == 89)  then // CAN Error
      print "CAN-error :  33: ",sysvar[33],"  34:",sysvar[34]
      printf "SDO Abort Code %lX", SYS_PROCESS(SYS_CANOM_SDOABORT)
   elseif (errno == 8) then
      print "track error limit reached"            // see:  set poserr x(Axis_no) 2000

   elseif (errno == 40) then
        print " Axis Error: ",errax
        //printf "EPOS4 Error: 0x%lx ",(sdoread (errax) 0x603F  0)   // set the correct CAN-errax-busid and activate this line!
        amperrclr x(errax)
/* 0s1001 / 0
Bit | Error-Reason
7 = Motion error        (value 128)
6 = reserved (always 0)
5 = Device profile-specific
4 = Communication error
3 = Temperature error
2 = Voltage error
1 = Current error
0 = Generic error
*/
   endif

   delay 1000
   errclr
   motor on	x(1)	x(2)	x(3)	x(4)

return


endprog
