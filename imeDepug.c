#pragma config(UserModel, "39zconfig.c")
#pragma platform(VEX)
#include <CKVexMotorSet.h>
#include <CKHolonomic.h>
#include <ZipTiesSFCS.h>
#include <CKVexIME.h>


int initX, initY, initH; //Initial X, Y, Heading

//IME lFrontIME, rFrontIME, lBackIME, rBackIME;
//IME baseEncoders[] = {lFrontIME, lBackIME, rFrontIME, rBackIME};
//int wheelAngles[] = {225, 315, 135, 45};
int wheelAngles[] = {45, 135, 225, 315};
int wheelRadius = 2;
int potVal, clawPotVal;

typedef enum StartingColor {red, blue}; //Color of the Starting Tile (red or blue)
typedef enum StartingPosition {pole, noPole}; //Side of the Field of the Starting Tile (near the pole or away from the pole)
StartingColor team;
StartingPosition side;

float curXPos, curYPos, curHeading; //Current X Position, Y Position, Heading
float interval = 0.075; //Interval is in SECONDS

tMotor liftMotors[] = {lLiftT, rLiftT, botLift};
MotorSet lift; //MotorSet of Right Lift Motors

tMotor driveMotors[] = {rBack, rFront, lFront, lBack}; //Array of Drive Motors
HolonomicBase driveTrain; //HolonomicBase of Drive Train Motors

float startX = 35.126; //All 4 starting tiles have +- this X Coord
float startY = 58.543; //All 4 starting tiles have +- this Y Coord
float rBackIME1 = nMotorEncoder[rBack];
/*void initIME(){
	IMEInit( lFrontIME, 3);
	IMEInit( rFrontIME, 9);
	IMEInit( lBackIME, 2);
	IMEInit( rBackIME, 8);
}*/

float dxRobot()
{
	float displacement = 0;
	int count = 0;
	for (int encoder = 0; encoder < 4; encoder++)
	{
		float wheelDisplacement = 2*PI*nMotorEncoder[driveMotors[encoder]]/627.2 * wheelRadius;
		nMotorEncoder[driveMotors[encoder]] = 0;
		float directionCosine = cos(degreesToRadians(wheelAngles[encoder]) - degreesToRadians(curHeading));
		if (directionCosine != 0.0)
		{
			count++;
			displacement += wheelDisplacement / directionCosine;
		}
	}
	displacement /= count;
	return displacement;
}

float dyRobot()
{
	float displacement = 0;
	int count = 0;
	for (int encoder = 0; encoder < 4; encoder++)
	{
		float wheelDisplacement = 2*PI*nMotorEncoder[driveMotors[encoder]]/627.2 * wheelRadius;
		nMotorEncoder[driveMotors[encoder]] = 0;
		float directionSine = sin(degreesToRadians(wheelAngles[encoder]) - degreesToRadians(curHeading));
		if (directionSine != 0.0)
		{
			count++;
			displacement += wheelDisplacement / directionSine;
		}
	}
	displacement /= count;
	return displacement;
}

void initGyro(){
	SensorType[gyro] = sensorNone; //Fixes common RobotC error with initializing Gyroscope
	wait1Msec(1000);
	SensorType[gyro] = sensorGyro;
	wait1Msec(2000);
	SensorValue(gyro) = 0; //Default Gyro heading is 0
}

//Calculates Initial Position and Heading based on current side and team
void initPos()
{
	initX = startX * ((side == pole) ? 1 : -1); //Which side of the field is the robot starting on?
	initY = startY * ((team == blue) ? 1 : -1);
	initH = (team == blue) ? 180 : 0; //Heading is determined by which color (which direction are we facing)
	curHeading = initH;
	SensorValue(gyro) = initH;
	curXPos = initX;
	curYPos = initY;
	nMotorEncoder[rFront] = 0;
	nMotorEncoder[rBack] = 0;
	nMotorEncoder[lFront] = 0;
	nMotorEncoder[lBack] = 0;
}

//Tracks Current Location and Heading while the track task is being run
task track()
{
	while (true)
	{
		curHeading = SensorValue(gyro);
		curXPos += dxRobot();
		curYPos += dyRobot();
		delay (interval*1000);
		potVal = SensorValue(pot);
		clawPotVal = SensorValue(clawPot);
	}
}

task main()
{
	initGyro();
	initPos();
	//initIME();
	startTask(track);
	while(true)
	{
		writeDebugStreamLine("X: %f", curXPos);
		writeDebugStreamLine("Y: %f", curYPos);
		//writeDebugStreamLine("IME: %f", nMotorEncoder[rFront]);
		delay(500);

	}
}
