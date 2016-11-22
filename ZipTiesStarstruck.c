#pragma config(UserModel, "39zconfig.c") //Cortex Configs
#pragma platform(VEX)//Platform Type
#pragma competitionControl(Competition) //This is a Competition Template
#pragma autonomousDuration(15) //15 second autonomous mode
#pragma userControlDuration(105) //1:45 driver control mode
#include "Vex_Competition_Includes.c" //Uses Vex stuff
#include "CKVexMotorSet.h"


int initX, initY, initH;
float potValLeft, potValRight;
int startTile;
float curXSpd = 0, curYSpd = 0, curXPos, curYPos, curHeading;
int interval = 75;
float uC = 39.3701; //1 meter = 39.3701 inches

tMotor lLiftMotors[] = {lLiftB, lLiftM, lLiftT};
MotorSet leftLift;

tMotor rLiftMotors[] = {rLiftB, rLiftY};
MotorSet rightLift;


int startPos[4][3];
void initStartPos(){
	//Red Left Tile
	startPos[0][0] = 0;
	startPos[0][1] = 0;
	startPos[0][2] = 0;

	//Red Right Tile
	startPos[1][0] = 0;
	startPos[1][1] = 0;
	startPos[1][2] = 0;

	//Blue Left Tile
	startPos[2][0] = 0;
	startPos[2][1] = 0;
	startPos[2][2] = 0;

	//Blue Right Tile
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

void setArm(float pos)
{
	potValLeft = SensorValue(potLeft);
	while (potValLeft != pos)
	{
		potValLeft = SensorValue(potLeft);
		if (potValLeft > pos)
		{
			setPower(leftLift, -1);
			setPower(rightLift, 1);
		}
		else if (potValLeft < pos)
		{
			setPower(leftLift, 1);
			setPower(rightLift, -1);
		}
	}
	motor[leftLift] = 0;
	motor[rightLift] = 0;
}

void initPos()
{
	initX = startPos[startTile][0];
	initY = startPos[startTile][1];
	initH = startPos[startTile][2];
	curXPos = initX;
	curYPos = initY;
	curHeading = initH;
}

task track()
{
	while (true)
	{
		curXSpd += SensorValue(acX)*interval*uC*1000; // m/s^2 * s/1000 * unit conversion
		curYSpd += SensorValue(acY)*interval*uC*1000;
		curXPos += curXSpd*interval*1000;
		curYPos += curYSpd*interval*1000;
		curHeading = SensorValue[gyro];
		delay (interval);
	}
}

void moveTo(float xTar, float yTar, float hTar)
{

}

void pre_auton()
{
	bStopTasksBetweenModes = true;
	initStartPos();
	initIMU();
	MotorSetInit (leftLift, lLiftMotors, 3);
	MotorSetInit (rightLift, rLiftMotors, 2);
}

void redLeft()
{
	startTile = 0;
}
void redRight()
{
	startTile = 1;
}
void blueLeft()
{
	startTile = 2;
}
void blueRight()
{
	startTile = 3;
}

task autonomous()
{
	initPos();
	startTask(track);
	stopTask(track);
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
		setPower(leftLift, vexRT[Btn5U] ? -127 : vexRT[Btn5D] ? 127 : armLock ? -20 : 0);
		setPower(rightLift, vexRT[Btn5U] ? 127 : vexRT[Btn5D] ? -127 : armLock ? 20 : 0);
		if (vexRT[Btn6D]){
			armLock = !armLock;
			delay(500);
		}
	}
}
