#pragma config(Motor,  port2,            ,             tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,            ,             tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port4,            ,             tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port5,            ,             tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port6,            ,             tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port7,            ,             tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port8,            ,             tmotorServoContinuousRotation, openLoop)
#pragma config(UART_Usage, UART1, uartUserControl, baudRate1200, IOPins, None, None)

// custom baud rate set code

typedef unsigned long  uint32_t;
typedef unsigned short uint16_t;

typedef struct
{
  uint16_t SR;
  uint16_t RESERVED0;
  uint16_t DR;
  uint16_t RESERVED1;
  uint16_t BRR;
  uint16_t RESERVED2;
  uint16_t CR1;
  uint16_t RESERVED3;
  uint16_t CR2;
  uint16_t RESERVED4;
  uint16_t CR3;
  uint16_t RESERVED5;
  uint16_t GTPR;
  uint16_t RESERVED6;
} USART_TypeDef;

/* Peripheral memory map */
#define PERIPH_BASE           ((unsigned long)0x40000000)
#define APB1PERIPH_BASE       PERIPH_BASE
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define USART2                ((USART_TypeDef *) USART2_BASE)
#define USART3                ((USART_TypeDef *) USART3_BASE)

void setBaud( const TUARTs nPort, int baudRate ) {
    uint32_t tmpreg = 0x00, apbclock = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    /* pclk1 - 36MHz */
    apbclock = 36000000;

    /* Determine the integer part */
    integerdivider = ((0x19 * apbclock) / (0x04 * (baudRate)));
    tmpreg = (integerdivider / 0x64) << 0x04;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (0x64 * (tmpreg >> 0x04));
    tmpreg |= ((((fractionaldivider * 0x10) + 0x32) / 0x64)) & 0x0F;

    /* Write to USART BRR */
    USART_TypeDef *uart = USART2;
    if( nPort == UART2 ) {
      uart = USART3;
    }
    uart->BRR = (uint16_t)tmpreg;
}

task main()
{
setBaud( UART1, 600); //initializes IR sensor
//UNHP 22-23 Robotics Main File
	/*
	Motors Power Levels:
	127 = Full power clockwise
	-127 = Full power backwards
	0 = Stationary
	Servo Positionioning:
	127 = Max 60 degrees in the positive direction
	-128 = Max -60 degrees in the negative direction
	0 = Middle
	Binary:
	0 = OFF
	1 = ON
	Motors & Servos Ports:
	port1 = Unused
	port2 = Left Wheel
	port3 = Right Wheel
	port4 = First Arm Servo
	port5 = Upper Arm
	port6 = Second Arm Servo
	port 7 = Third Arm Servo
	port 8 = Unused
	Note: vexRT[] refers to the VEX Joystick's channels (there are up to 8 channels on the joystick).
	Robot Controls:
	Wheels: Left and Right Side Joystick for the left and right wheel, up and down.
	Upper Arm: Right Side Joystick, left to right.
	Claw Motors:
	Left Claw: Button 5D Closes the claw, Button 5U opens the claw, button 7U resets the claw to 0 degrees, and clicking on Button 7L will increase the claw degree count by 25 degree
	Right Claw: Same controls except the buttons are as follows: 6D, 6U, 7D, and 7R.
	IR Controls: Should be utilizing all the "8" buttons.
	*/


	while(1 == 1) {
		motor[port2] = -(vexRT[Ch3]); // Channel 3 (left-side joystick, up & down) for the left wheel
		motor[port3] = (vexRT[Ch2]); // Channel 2 (right-side joystick, up & down) for the right wheel
		motor[port5] = (vexRT[Ch1]); // Channel 1 (right-side joystick, left & right) for the Upper Arm


	//Claw Motors
/*	if (vexRT[Btn5U] == 1 && vexRT[Btn5U] != 0) { //Holding Button 5 Up on the controller
	motor[port4] = 127; // Port 4 Motor Opens Left Claw
		} else if (vexRT[Btn5D] == 1 && vexRT[Btn5D] != 0) { //Holding Button 5 Down on the controller
	motor[port4] = -128; // Port 4 Motor Closes Left Claw
		} else if (vexRT[Btn7U] == 1) {
	motor[port4] = 0; // Pressing Button 7U resets the angle of the claw to 0 degrees
		}
if (vexRT[Btn7L] == 1) {
	motor[port4] += 25; // Every time button 7R is clicked, the angle of the claw is incremented by 25 degrees.
		}
	if (vexRT[Btn6U] == 1 && vexRT[Btn6U] != 0) {// Holding Channel 6 Up Button
	motor[port6] = 127; // Port 6 Motor closes right claw
		} else if (vexRT[Btn6D] == 1 && vexRT[Btn6D] != 0) {// Holding Channel 6 Down Button
	motor[port6] = -128; // Port 6 Motor opens right claw
		} else if(vexRT[Btn7D] == 1) {
	motor[port6] = 0;
		}
	if (vexRT[Btn7R] == 1 && vexRT[Btn7R] != 0) {
  motor[port6] += 25;
 }*/

 if (vexRT[Btn5D] == 1) { //Holding Button 5 Up on the controller
	motor[port4] = 127; // Port 4 Motor Opens Left Claw
	motor[port6] = -128; // Port 6 Motor Opens right claw
		} else if (vexRT[Btn6D] == 1) { //Holding Button 5 Down on the controller
	motor[port4] = -128; // Port 4 Motor Closes Left Claw
	motor[port6] = 127; //Port 6 Motor Closes Right Claw
		} else if (vexRT[Btn7U] == 1) {
	motor[port4] = 0; // Pressing Button 7U resets the angle of the claw to 0 degrees
	motor[port6] = 0;
		}
if (vexRT[Btn7L] == 1) {
	motor[port4] = 25; // Every time button 7R is clicked, the angle of the left claw is equal to 25 degrees.
		}

if (vexRT[Btn8U] == 1) {
	sendChar( UART1,0x33 ); // Every time button 8U is clicked, the Squeaky controls are changed to Drive-Rotation-Elevation
}

if (vexRT[Btn8R] == 1) {
	sendChar( UART1,0x66 ); // Every time button 8R is clicked, the Squeaky controls are changed to Rotation - Drive - Elevation
}

if (vexRT[Btn8D] == 1) {
	sendChar( UART1,0x3c ); // Every time Button 8D is clicked, the Squeaky controls are changed to Elevation-Drive-Rotation
	}

	//Lever Servos for Squeaky Control Panel
	if (vexRT[Btn8U] == 1) {
		motor[port4] = 127; // First Squeaky Arm Servo turned Fully Forward
	}
	if (vexRT[Btn8R] == 1) {
		motor[port4] = -128; // First Squeaky Arm Servo turned Fully Backward
	}
	/*if (vexRT[Btn8D] == 1) {
		motor[port6] = 127; // Second Squeaky Arm Servo turned Fully Forward
	}
	if (vexRT[Btn8L] == 1) {
		motor[port6] = -128; // Second Squeaky Arm Servo turned Fully Backward
	}
	if (vexRT[Btn7U] == 1) {
		motor[port7] = 127;	// Third Squeaky Arm Servo turned Fully Forward
	}
	if (vexRT[Btn7R] == 1) {
		motor[port7] = -128;	// Third Squeaky Arm Servo turned Fully Backward
	}*/
}
}
