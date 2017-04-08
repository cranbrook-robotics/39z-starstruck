#ifndef __HolonomicAuto__
#define __HolonomicAuto__

#pragma systemFile

float wheelTargetDisp[] = {0, 0, 0, 0}; //Wheel Target Displacement
float wheelDisp[] = {0, 0, 0, 0}; //Current Wheel Displacement
float wheelCruisingVelocity[] = {0, 0, 0, 0}; //Wheel Cruising Velocity as calculated by Program
float wheelHeadings[4]; // Wheel headings (will be imported)
tMotor importedDriveMotors[4]; // Drive Motor Array (will be imported)
float wheelPercentDisp[] = {0, 0, 0, 0}; // Wheel percent Displacement (Current Displacement / Target Displacement)
float wheelCurrentVelocity[] = {0, 0, 0, 0}; // Wheel Current Velocity (Calculated using PID loop as a reduction from Cruising Velocity
float curXPos, curYPos; // Current X and Y Coordinate (as of end of last movement OR starting position)
float tarXPos, tarYPos; // Target Location, gets input by program
float tarHead, curHead; // Target Heading = Direction of Target Vector - curHeading: curHeading will be kept track of by Gyro, Target Vector will be calculated
float kP;
float wheelError[] = {0, 0, 0, 0};

typedef struct Vector //Struct Created to Model a Vector: Has direction and magnitude
{
	float direction;
	float magnitude;
}

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
	tarHead = targetVector.direction - curHead;
	for (int i = 0; i < 4; i++)
	{
		wheelTargetDisp[i] = targetVector.magnitude * cos(wheelHeadings[i] - tarHead);
	}
}

void convertInchesToTicks() //Calculates target displacement in in ticks for each wheel
{
	for (int i = 0; i < 4; i++)
	{
		wheelTargetDisp[i] = wheelTargetDisp[i] * 392 / 4 / PI;
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

void setMotors() //Sets motors to value stored in Current Velocity Array
{
	for (int i = 0; i < 4; i++)
	{
		importedDriveMotors[i] = wheelCurrentVelocity[i];
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
	}
}

#endif
