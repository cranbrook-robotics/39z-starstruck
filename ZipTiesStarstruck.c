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

typedef enum StartingTile {redLeft, redRight, blueLeft, blueRight};
StartingTile position;

float curXSpd = 0, curYSpd = 0, curXPos, curYPos, curHeading;
float interval = 0.075; //Interval is in SECONDS

tMotor lLiftMotors[] = {lLiftB, lLiftM, lLiftT};
MotorSet leftLift;

tMotor rLiftMotors[] = {rLiftB, rLiftY};
MotorSet rightLift;

tMotor driveMotors[] = {rBack, rFront, lBack, lFront};
HolonomicBase driveTrain;

typedef enum PositionAxes {xA, yA, hA};



int startPos[4][3];
void initStartPos(){
	//Red Left Tile
	startPos[redLeft][xA] = 0;
	startPos[redLeft][yA] = 0;
	startPos[redLeft][hA] = 0;

	//Red Right Tile
	startPos[redRight][xA] = 0;
	startPos[redRight][yA] = 0;
	startPos[redRight][hA] = 0;

	//Blue Left Tile
	startPos[blueLeft][xA] = 0;
	startPos[blueLeft][yA] = 0;
	startPos[blueLeft][hA] = 0;

	//Blue Right Tile
	startPos[blueRight][xA] = 0;
	startPos[blueRight][yA] = 0;
	startPos[blueRight][hA] = 0;
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
	initX = startPos[position][xA];
	initY = startPos[position][yA];
	initH = startPos[position][hA];
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
	initStartPos();
	initIMU();
	MotorSetInit (leftLift, lLiftMotors, 3);
	MotorSetInit (rightLift, rLiftMotors, 2);
	SensorValue(initIndicator) = 0;
	InitHolonomicBase(driveTrain, driveMotors, 4);
}

void redLeftAuto()
{
	position = redLeft;
}
void redRightAuto()
{
	position = redRight;
}
void blueLeftAuto()
{
	position = blueLeft;
}
void blueRightAuto()
{
	position = blueRight;
}

task autonomous()
{
	initPos();
	startTask(track);
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
