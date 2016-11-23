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

typedef enum StartingColor {red, blue};
typedef enum StartingPosition {pole, noPole};
StartingColor team;
StartingPosition side;

float curXSpd = 0, curYSpd = 0, curXPos, curYPos, curHeading;
float interval = 0.075; //Interval is in SECONDS

tMotor lLiftMotors[] = {lLiftB, lLiftM, lLiftT};
MotorSet leftLift;

tMotor rLiftMotors[] = {rLiftB, rLiftY};
MotorSet rightLift;

tMotor driveMotors[] = {rBack, rFront, lBack, lFront};
HolonomicBase driveTrain;

float startX = 35.126;
float startY = 58.543;

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
	initX = startX * (side == pole ? 1 : -1);
	initY = startY * (team == blue ? 1 : -1);
	initH = team == blue ? PI : 0;
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
		curHeading = degreesToRadians(SensorValue(gyro)/10.);
		delay (interval/1000);
	}
}

void moveTo(float xTar, float yTar, float hTar)
{
	bool xArrive = false;
	bool yArrive = false;
	bool hArrive = false;
	while (!xArrive || !yArrive || !hArrive){
		setDriveXYR(driveTrain, xTar - curXPos, yTar - curYPos, hTar - curHeading);
		xArrive = abs(xTar - curXPos) <= 1.5;
		yArrive = abs(yTar - curYPos) <= 1.5;
		hArrive = abs(hTar - curHeading) <= 1.5;
	}
}

void pre_auton()
{
	bStopTasksBetweenModes = true;
	SensorValue(initIndicator) = 1;
	initIMU();
	MotorSetInit (leftLift, lLiftMotors, 3);
	MotorSetInit (rightLift, rLiftMotors, 2);
	SensorValue(initIndicator) = 0;
	InitHolonomicBase(driveTrain, driveMotors, 4);
}

void redLeftAuto()
{
	team = red;
	side = noPole;
	initPos();
}
void redRightAuto()
{
	team = red;
	side = pole;
	initPos();
}
void blueLeftAuto()
{
	team = blue;
	side = pole;
	initPos();
}
void blueRightAuto()
{
	team = blue;
	side = noPole;
	initPos();
}

task autonomous()
{
	startTask(track);
	redLeftAuto();
	//redRightAuto();
	//blueLeftAuto();
	//blueRightAuto();
	stopTask(track);
}

task usercontrol()
{
	while (true)
	{
		potValLeft = SensorValue(potLeft);
		potValRight = SensorValue(potRight);
		setDriveXYR(driveTrain, vexRT[Ch4]/127., vexRT[Ch3]/127., vexRT[Ch1]/127.);
		setPower(rightLift, vexRT[Btn5U] ? 127 : vexRT[Btn5D] ? -127 : 0);
	}
}
