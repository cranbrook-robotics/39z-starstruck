#ifndef __HolonomicAuto__
#define __HolonomicAuto__

#include<ZipTiesSFCS.h>

float wheelTargetDisp[] = {0, 0, 0, 0}; //Wheel Target Displacement
float wheelDisp[] = {0, 0, 0, 0}; //Current Wheel Displacement
float wheelCruisingVelocity[] = {0, 0, 0, 0}; //Wheel Cruising Velocity as calculated by Program
int importedWheelHeadings[4]; // Wheel headings (will be imported)
tMotor importedDriveMotors[4]; // Drive Motor Array (will be imported)
float wheelPercentDisp[] = {0, 0, 0, 0}; // Wheel percent Displacement (Current Displacement / Target Displacement)
float wheelCurrentVelocity[] = {0, 0, 0, 0}; // Wheel Current Velocity (Calculated using PID loop as a reduction from Cruising Velocity
float curXPos, curYPos; // Current X and Y Coordinate (as of end of last movement OR starting position)
float tarXPos, tarYPos; // Target Location, gets input by program
float tarHead; // Target Heading = Direction of Target Vector - robotHeading()
float kP = 0.5;
float wheelError[] = {0, 0, 0, 0};
float startX = 35.126; //All 4 starting tiles have +- this X Coord
float startY = 58.543; //All 4 starting tiles have +- this Y Coord
float initX, initY, initH; //Initial X, Y, Heading
typedef enum StartingColor {red, blue}; //Color of the Starting Tile (red or blue)
typedef enum StartingPosition {pole, driver}; //Side of the Field of the Starting Tile (near the pole or away from the pole)
StartingColor team;
StartingPosition side;
float  curHeading; //Current X Position, Y Position, Heading

float robotHeading(){
	return initH + curHeading;
}

void initGyro(){
	SensorType[gyro] = sensorNone; //Fixes common RobotC error with initializing Gyroscope
	wait1Msec(1000);
	SensorType[gyro] = sensorGyro;
	wait1Msec(2000);
	SensorValue[gyro] = 0; //Default Gyro heading is 0
}

//Calculates Initial Position and Heading based on current side and team
void initPos()
{
	initX = startX * ((side == pole) ? 1 : -1); //Which side of the field is the robot starting on?
	initY = startY * ((team == blue) ? 1 : -1);
	initH = (team == blue) ? 270 : 90; //Heading is determined by which color (which direction are we facing)
	curHeading = initH;
	curXPos = initX;
	curYPos = initY;
}


typedef struct Vector //Struct Created to Model a Vector: Has direction and magnitude
{
	float direction;
	float magnitude;
} Vector;

struct Vector targetVector;

void calcTarVector() //Calculates the Target Drive Vector
{
	float tarYDisp = tarYPos - curYPos;
	float tarXDisp = tarXPos - curXPos;
	targetVector.direction = atan2(tarYDisp, tarXDisp);
	targetVector.magnitude = sqrt(tarYDisp * tarYDisp + tarXDisp * tarXDisp);
}

void calcWheelTargetDisp() //Calculates target displacement for each wheel
{
	tarHead = targetVector.direction - robotHeading();
	for (int i = 0; i < 4; i++)
	{
		wheelTargetDisp[i] = targetVector.magnitude * cos(importedWheelHeadings[i] - tarHead);
	}
}

void convertInchesToTicks() //Calculates target displacement in in ticks for each wheel
{
	for (int i = 0; i < 4; i++)
	{
		wheelTargetDisp[i] = wheelTargetDisp[i] * 627.2 / 4 / PI;
	}
}

void calcWheelCrusingVelocity() //Calculates cruising velocity for each wheel
{
	float largestAbsoluteTargetDisp = -1000000;
	for (int i = 0; i < 4; i++)
	{
		largestAbsoluteTargetDisp = abs(wheelTargetDisp[i]) > largestAbsoluteTargetDisp ? wheelTargetDisp[i] : largestAbsoluteTargetDisp;
	}
	for (int i = 0; i < 4; i++)
	{
		wheelCruisingVelocity[i] = (wheelTargetDisp[i] / largestAbsoluteTargetDisp)	* 127;
		wheelCurrentVelocity[i] = wheelCruisingVelocity[i];
	}
}

void setdriveMotors() //Sets motors to value stored in Current Velocity Array
{
	for (int i = 0; i < 4; i++)
	{
		motor[importedDriveMotors[i]] = wheelCurrentVelocity[i];
	}
}

void calcWheelDisp() //Calculates wheel's displacement
{
	for (int i = 0; i < 4; i++)
	{
		wheelDisp[i] += nMotorEncoder[importedDriveMotors[i]];
		nMotorEncoder[importedDriveMotors[i]] = 0;
	}
}

void calcPercentDisp() //Calculates wheel's percent displacement
{
	for (int i = 0; i < 4; i++)
	{
		wheelPercentDisp[i] = wheelDisp[i] / wheelTargetDisp[i];
	}
}

void adjustMotor() // Uses PID Loop to Adjust Motor (slows down motors that are too fast)
{
	//float largestPercentDisp = 0;
	float smallestPercentDisp = 1;
	for (int i = 0; i < 4; i++)
	{
		//largestPercentDisp = wheelPercentDisp[i] > largestPercentDisp ? wheelPercentDisp[i] : largestPercentDisp;
		smallestPercentDisp = wheelPercentDisp[i] < smallestPercentDisp ? wheelPercentDisp[i] : smallestPercentDisp;
	}
	for (int i = 0; i < 4; i++)
	{
		wheelError[i] = wheelPercentDisp[i] - smallestPercentDisp;
		wheelCurrentVelocity[i] -= kP * wheelError;
		if (wheelPercentDisp[i] >= 1)//NOTE: can displacement exceed 1?
		{
			wheelCurrentVelocity[i] = 0;
			wheelCruisingVelocity[i] = 0;
		}
	}
}

bool hasArrived()
{
	return wheelPercentDisp[0] >= 1 && wheelPercentDisp[1] >= 1 && wheelPercentDisp[2] >= 1 && wheelPercentDisp[3];// (== 1?)
}//NOTE: same as above - can disp > 1?

void initializeHolonomicAuto(tMotor* motors, int* wheelHeadings)
{
	for (int i = 0; i < 4; i++)
	{
		importedDriveMotors[i] = motors[i];
		importedWheelHeadings[i] = wheelHeadings[i];
	}
}

void setDestination(float xCoord, float yCoord)
{
	tarXPos = xCoord;
	tarYPos = yCoord;
}

void driveCalculations()
{
	calcTarVector();
	calcWheelTargetDisp();
	convertInchesToTicks();
	calcWheelCrusingVelocity();
}
void runDrive()
{
	while(!hasArrived())
	{
		setdriveMotors();
		calcWheelDisp();
		calcPercentDisp();
		adjustMotor();
	}
}

void moveToDestination(float xTarget, float yTarget)
{
	setDestination(xTarget, yTarget);
	driveCalculations();
	runDrive();
}

void moveToObject(coord Object)
{
	moveToDestination(Object[0], Object[1]);
}
#endif
