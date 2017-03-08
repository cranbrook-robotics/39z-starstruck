#pragma config(UserModel, "39zconfig.c")
#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)
#include <Vex_Competition_Includes.c>
#include <CKVexMotorSet.h>
#include <CKHolonomic.h>
#include <ZipTiesSFCS.h>
#include <CKVexIME.h>


float initX, initY, initH; //Initial X, Y, Heading

int potVal, clawPotVal;

float clawPercentage;

typedef enum StartingColor {red, blue}; //Color of the Starting Tile (red or blue)
typedef enum StartingPosition {pole, noPole}; //Side of the Field of the Starting Tile (near the pole or away from the pole)
typdef struct DriveWheel
{
	int wheelAngle;
	int wheelRadius;
	tMotor driveMotor;
	float wheelDisp = 0;
	float wheelTargetDisp = 0;
	float pWheelDisplacement = 0;

	void InitDriveWheel (DriveWheel self, int angle, int radius, tMotor myMotor)
	{
		self.wheelAngle = angle;
		self.wheelRadius = radius;
		self.driveMotor = myMotor;
	}
}

DriveWheel rightBack, rightFront, leftFront, leftBack;
DriveWheel wheels[] = {rightBack, rightFront, leftFront, leftBack};
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

void initIME(){
	nMotorEncoder[rBack] = 0;
	nMotorEncoder[rFront] = 0;
	nMotorEncoder[lFront] = 0;
	nMotorEncoder[lBack] = 0;
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
	curHeading = 0.0;
	curXPos = initX;
	curYPos = initY;
}
/////////////////////////////////////////////////////////////////
////////****************************************************/////
////////													/////
////////	  COORDINATE SYSTEM AND LOCATION TRACKING   	/////
////////													/////
////////****************************************************/////
/////////////////////////////////////////////////////////////////


/*
1) Input Destination point
2) Calculate (for each wheel) necessary displacement
3) Set all wheel powers to +- 127
4) Calculate wheel percent displacement
5) PID loop to adjust powers, error is difference between wheel displacement percent and average displacement percent
6) repeat 4-5 until reach destination
7) Rotate robot until desired heading is reached
*/
task track()
{
	while (true)
	{
		for (int i = 0; i < 4; i++){

		}
	}
}
void moveTo(float xTar, float yTar, float hTar)
{
	float xDisp = xTar - curXPos;
	float yDisp = yTar - curYPos;
	float hDisp = hTar - curHeading;
	if (xDisp != 0)
		float vectorHeading = atan2(yDisp,xDisp) * 180 / PI;
		vectorHeading = vectorHeading > 0 ? vectorHeading : vectorHeading + 360;
	else
		float vectorHeading = yTar > 0 ? 90 : 270;
	float vectorMagnitude = sqrt(xDisp*xDisp + yDisp * yDisp);
	for (int i= 0; i < 4; i++)
	{
		wheels[i].wheelTargetDisp = xDisp * 
	}

}



/////////////////////////////////////////////////////////////////
////////****************************************************/////
////////													/////
////////	  					END 					   	/////
////////													/////
////////****************************************************/////
/////////////////////////////////////////////////////////////////

void pre_auton()
{
	bStopTasksBetweenModes = true;
	bLCDBacklight = true;
	clearLCDLine(0);
	clearLCDLine(1);
	initIME();
	initGyro();
	MotorSetInit (lift, liftMotors, 3);
	InitHolonomicBase(driveTrain, driveMotors, 4);
	InitDriveWheel(rightBack, 45, 2, rBack );
	InitDriveWheel(rightFront, 135, 2, rFront);
	InitDriveWheel(leftFront, 225, 2, lFront);
	InitDriveWheel(leftBack, 315, 2, lBack);
}

task lcdManager()
{
	string lcdBatteryVoltages;
	while(true)
	{
		sprintf(lcdBatteryVoltages, "M: %.2f P: %.2f", MainBatteryVoltage(), powerExpanderVoltage(pPowerExp));
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDString(0,0,lcdBatteryVoltages);
		delay(300);
	}
}


task autonomous()
{

}

task usercontrol()
{
	while (true)
	{
		startTask(lcdManager);
		setDriveXYR(driveTrain, vexRT[Ch4]/127., vexRT[Ch3]/127., vexRT[Ch1]/127.);
		motor[clawY] = vexRT[Btn6UXmtr2] ? 127 : vexRT[Btn6DXmtr2] ? -127 : 0;
		setPower(lift, vexRT[Btn5UXmtr2] ? 1 : vexRT[Btn5DXmtr2] ? -1 : 0);
	}
}
