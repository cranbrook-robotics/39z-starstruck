#pragma config(UserModel, "39zconfig.c") //Cortex Configs
#pragma platform(VEX)//Platform Type
#pragma competitionControl(Competition) //This is a Competition Template
#pragma autonomousDuration(15) //15 second autonomous mode
#pragma userControlDuration(105) //1:45 driver control mode
#include <Vex_Competition_Includes.c> //Uses Vex stuff
#include <CKVexMotorSet.h>
#include <CKHolonomic.h>


int initX, initY, initH;
float potValLeft, potValRight;
int startTile;
float curXSpd = 0, curYSpd = 0, curXPos, curYPos, curHeading;
float interval = 0.075; //Interval is in SECONDS

tMotor lLiftMotors[] = {lLiftB, lLiftM, lLiftT};
MotorSet leftLift;

tMotor rLiftMotors[] = {rLiftB, rLiftY};
MotorSet rightLift;

tMotor driveMotors[] = {rBack, rFront, lBack, lFront};
HolonomicBase driveTrain;


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

typedef enum AccAxis { XAxis, YAxis };
tSensors accPorts[] = {acX, acY};
float accBias[] = {0, 0};

float getAcc(AccAxis axis)
{
	return SensorValue(accPorts[axis])*0.5234 - 1071.7 - accBias[axis];//Conversion from Accelerometer Returns to in/s^2
}
void initIMU(){
	curXSpd = 0;
	curYSpd = 0;
	curHeading = initH;
	SensorValue(gyro) = initH;
	float xC = 0;
	float yC = 0;
	for (int i = 0; i < 50; i++)
	{
		xC += getAcc(XAxis);
		yC += getAcc(YAxis);
		delay(10);
	}
	accBias[XAxis] = xC/50;
	accBias[YAxis] = yC/50;
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
		curXSpd += getAcc(XAxis)*interval; // m/s^2 * s/1000 * unit conversion
		curYSpd += getAcc(YAxis)*interval;
		curXPos += curXSpd*interval;
		curYPos += curYSpd*interval;
		curHeading = SensorValue(gyro);
		delay (interval/1000);
	}
}

void moveTo(float xTar, float yTar, float hTar)
{
	float translationalDirection = atan2(yTar, xTar);
}

void pre_auton()
{
	bStopTasksBetweenModes = true;
	SensorValue(initIndicator) = 1;
	initStartPos();
	initIMU();
	MotorSetInit (leftLift, lLiftMotors, 3);
	MotorSetInit (rightLift, rLiftMotors, 2);
	SensorValue(initIndicator) = 0;
	InitHolonomicBase(driveTrain, driveMotors, 4);
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
	//redLeft();
	//redRight();
	//blueLeft();
	//blueRight();
	stopTask(track);
}

task usercontrol()
{
	while (true)
	{
		potValLeft = SensorValue(potLeft);
		potValRight = SensorValue(potRight);
		setDriveXYR(driveTrain, vexRT[Ch4], vexRT[Ch3], vexRT[Ch1]);
		//motor[lFront] = vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1];
		//motor[lBack] =  vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1];
		//motor[rFront] = -vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1];
		//motor[rBack] = -vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1];
		//setPower(leftLift, vexRT[Btn5U] ? -127 : vexRT[Btn5D] ? 127 : 0);
		setPower(rightLift, vexRT[Btn5U] ? 127 : vexRT[Btn5D] ? -127 : 0);
	}
}
