#pragma config(UserModel, "39zconfig.c") //Cortex Configs
#pragma platform(VEX)//Platform Type
#pragma competitionControl(Competition) //This is a Competition Template
#pragma autonomousDuration(15) //15 second autonomous mode
#pragma userControlDuration(105) //1:45 driver control mode
#include "Vex_Competition_Includes.c" //Uses Vex stuff


int initX, initY, initH;
bool armUp, armDown;
float potValLeft, potValRight;
int startTile;
bool tracking = false;
float curXSpd = 0, curYSpd = 0, curXPos, curYPos, curHeading;
int interval = 75;
float uC = 1;

int startPos[4][3];
void initStartPos(){
	startPos[0][0] = 0;
	startPos[0][1] = 0;
	startPos[0][2] = 0;
	startPos[1][0] = 0;
	startPos[1][1] = 0;
	startPos[1][2] = 0;
	startPos[2][0] = 0;
	startPos[2][1] = 0;
	startPos[2][2] = 0;
	startPos[3][0] = 0;
	startPos[3][1] = 0;
	startPos[3][2] = 0;
}

void initIMU(){
	curXSpd = 0;
	curYSpd = 0;
	curHeading = initH;
	SensorValue[gyro] = initH;
}

void track()
{
	curXSpd += SensorValue[acX]*interval*uC;
	curYSpd += SensorValue[acY]*interval*uC;
	curXPos += curXSpd*interval*uC;
	curYPos += curYSpd*interval*uC;
	curHeading = SensorValue[gyro];
}

void setArm(float pos)
{
	potValLeft = SensorValue(potLeft);
	while (potValLeft != pos)
	{
		potValLeft = SensorValue(potLeft);
		if (potValLeft > pos)
		{
			motor[leftLift] = -127;
			motor[rightLift] = 127;
		}
		else if (potValLeft < pos)
		{
			motor[leftLift] = 127;
			motor[rightLift] = -127;
		}
	}
	motor[leftLift] = 0;
	motor[rightLift] = 0;
}
void armGoDown(int time)
{
	motor[leftLift] = 127;
	motor[rightLift] = -127;
	delay(time);
	motor[leftLift] = 0;
	motor[rightLift] = 0;

}
void moveY(int time, int dir)//dir>0 moves forwards
{
	dir = sgn(dir);
	motor[lFront] = dir*127;
	motor[lBack] =  dir*127;
	motor[rFront] = dir*-127;
	motor[rBack] = dir*-127;
	delay(time);
	motor[lFront] = 0;
	motor[lBack] = 0;
	motor[rFront] = 0;
	motor[rBack] = 0;
}
void moveX(int time, int dir)//dir>0 moves right
{
	dir = sgn(dir);
	motor[lFront] = dir*127;
	motor[lBack] = dir*-127;
	motor[rFront] = dir*127;
	motor[rBack] = dir*-127;
	delay(time);
	motor[lFront] = 0;
	motor[lBack] = 0;
	motor[rFront] = 0;
	motor[rBack] = 0;
}
void rotate(int time, int dir)//dir>0 moves clockwise
{
	dir = sgn(dir);
	motor[lFront] = dir*127;
	motor[lBack] = dir*127;
	motor[rFront] = dir*127;
	motor[rBack] = dir*127;
	delay(time);
	motor[lFront] = 0;
	motor[lBack] = 0;
	motor[rFront] = 0;
	motor[rBack] = 0;
}

void autoLeft()
{
	moveY(500,1);
	armGoDown(500);
	//moveY(500,-1);
	delay(150);
	setArm(2650);
	delay(150);
	moveY(2350,1);
	delay(150);
	moveY(500,-1);
	delay(150);
	moveX(1000,-1);
	delay(150);
	moveY(1300,1);
	delay(150);
	moveY(500,-1);
}
void autoRight()
{
	moveY(500,1);
	armGoDown(500);
	//moveY(500,-1);
	delay(150);
	setArm(2650);
	delay(150);
	moveY(2350,1);
	delay(150);
	moveY(500,-1);
	delay(150);
	moveX(750,1);
	delay(150);
	moveY(1300,1);
	delay(150);
	moveY(500,-1);
}
void pre_auton()
{
	bStopTasksBetweenModes = true;
	initStartPos();
}
task autonomous()
{
	autoRight();
}

task usercontrol()
{
	bool armLock = false;
	while (true)
	{
		potValLeft = SensorValue(potLeft);
		potValRight = SensorValue(potRight);
		motor[lFront] = vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1];
		motor[lBack] =  vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1];
		motor[rFront] = -vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1];
		motor[rBack] = -vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1];
		motor[leftLift] = motor[leftLiftY] = vexRT[Btn5U] ? -127 : vexRT[Btn5D] ? 127 : armLock ? -20 : 0;
		motor[rightLift] = motor[rightLiftY] = vexRT[Btn5U] ? 127 : vexRT[Btn5D] ? -127 : armLock ? 20 : 0;
		if (vexRT[Btn6D]){
			armLock = !armLock;
			delay(500);
		}
	}
}
