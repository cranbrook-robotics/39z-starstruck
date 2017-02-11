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


int initX, initY, initH; //Initial X, Y, Heading

IME lFrontIME, rFrontIME, lBackIME, rBackIME;
IME baseEncoders[] = {lFrontIME, lBackIME, rFrontIME, rBackIME};
int wheelAngles[] = {225, 315, 135, 45};
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

void initIME(){
	IMEInit( lFrontIME, 3);
	IMEInit( rFrontIME, 9);
	IMEInit( lBackIME, 2);
	IMEInit( rBackIME, 8);
}

float dxRobot()
{
	float displacement = 0;
	int count = 0;
	for (int encoder = 0; encoder < 4; encoder++)
	{
		float wheelDisplacement = baseEncoders[encoder].position * wheelRadius;
		baseEncoders[encoder].position = 0;
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
		float wheelDisplacement = baseEncoders[encoder].position * wheelRadius;
		baseEncoders[encoder].position = 0;
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
	initH = (team == blue) ? PI : 0; //Heading is determined by which color (which direction are we facing)
	curHeading = initH;
	SensorValue(gyro) = initH;
	curXPos = initX;
	curYPos = initY;
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

void setArm(float percentage)
{
	int potTarget = percentage*19.69 + 1579;
	while (abs(potVal - potTarget) > 5)
	{
		if (potVal > potTarget)
		{
			setPower(lift, 1);
		}
		else if (potVal < potTarget)
		{
			setPower(lift, -1);
		}
	}
	setPower(lift, 0);
}

void setClaw(float percentage)
{
	int potTarget = percentage*17.4 + 1200;
	while (abs(clawPotVal - potTarget) > 5)
	{
		if (clawPotVal > potTarget)
		{
			motor[clawY] = -127;
		}
		else if (clawPotVal < potTarget)
		{
			motor[clawY] = 127;
		}
	}
	motor[clawY] = 0;
}

//Moves robot to parameters X Coordinate, Y Coordinate, and Heading
void moveTo(float xTar, float yTar, float hTar)
{
	hTar = degreesToRadians(hTar);
	bool xArrive = false;
	bool yArrive = false;
	bool hArrive = false;
	while (!xArrive || !yArrive || !hArrive){
		setDriveXYR(driveTrain, xTar - curXPos, yTar - curYPos, hTar - curHeading);
		xArrive = abs(xTar - curXPos) <= 1.5;
		yArrive = abs(yTar - curYPos) <= 1.5;
		hArrive = abs(hTar - curHeading) <= 1.5;
		delay(interval*1000);
	}
}

void moveToPoint(coord cTar, float hTar)
{
	moveTo(cTar[0], cTar[1], hTar);
}

void pre_auton()
{
	bStopTasksBetweenModes = true;
	bLCDBacklight = true;
	clearLCDLine(0);
	clearLCDLine(1);
	SensorValue(initIndicator) = 1;
	initIME();
	initGyro();
	MotorSetInit (lift, liftMotors, 3);
	SensorValue(initIndicator) = 0;
	InitHolonomicBase(driveTrain, driveMotors, 4);
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
void blueLeftAuto()
{
	team = blue;
	side = pole;
	initPos();
}

task autonomous()
{
	startTask(track);
	blueLeftAuto();
	stopTask(track);
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
