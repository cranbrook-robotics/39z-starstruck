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

void initIME(){
	nMotorEncoder[rBack] = 0;
	nMotorEncoder[rFront] = 0;
	nMotorEncoder[lFront] = 0;
	nMotorEncoder[lBack] = 0;
}

float* displacementRobot()
{
	float dispX = 0;
	float dispY = 0;
	int countCosine = 0;
	int countSine = 0;
	for (int encoder = 0; encoder < 4; encoder++)
	{
		float wheelDisplacement = nMotorEncoder[driveMotors[encoder]] * wheelRadius * 2 * PI / 627.2;
		nMotorEncoder[driveMotors[encoder]] = 0;
		float directionCosine = cos(degreesToRadians(wheelAngles[encoder]) - degreesToRadians(curHeading));
		float directionSine = sin(degreesToRadians(wheelAngles[encoder]) - degreesToRadians(curHeading));
		if (directionCosine != 0.0)
		{
			countCosine++;
			dispX += wheelDisplacement / directionCosine;
		}
		if (directionSine != 0.0)
		{
			countSine++;
			dispY += wheelDisplacement / directionCosine;
		}
	}
	dispX /= countCosine;
	dispY /= countSine;
	float displacement[2] = {dispX, dispY};
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
}

//Tracks Current Location and Heading while the track task is being run
task track()
{
	while (true)
	{
		curHeading = SensorValue(gyro)/10.0;
		float* displacement = displacementRobot();
		curXPos += displacement[0];
		curYPos += displacement[1];
		potVal = SensorValue(pot);
		clawPotVal = SensorValue(clawPot);
		delay (interval*1000);

	}
}

void setArm(float percentage)
{
	int potTarget = percentage*19.69 + 1579;
	float error = potVal - potTarget;
	bool armArrive = false;
	float kP = 0.05;
	while (abs(potVal - potTarget) > 5)
	{
		setPower(lift, error*kP);
		error = potVal - potTarget;
		armArrive = abs(error) <= 5;
		delay(30);
	}
	setPower(lift, 0);
}

void setClaw(float percentage)
{
	int potTarget = percentage*17.4 + 1200;
	float error = potTarget - clawPotVal;
	bool clawArrive = false;
	float kP = 0.05;
	while (!clawArrive)
	{
		motor[clawY] = error*kP;
		error = potTarget - clawPotVal;
		clawArrive = abs(error) <= 5;
		delay(30);
	}
	motor[clawY] = 0;
}

//Moves robot to parameters X Coordinate, Y Coordinate, and Heading
void moveTo(float xTar, float yTar, float hTar)
{
	bool xArrive = false;
	bool yArrive = false;
	bool hArrive = false;
	float kH = 0.005;
	float kP = 0.05;
	float xError = xTar - curXPos;
	float yError = yTar - curYPos;
	float hError = curHeading - hTar;
	while (!xArrive || !yArrive || !hArrive){
		setDriveXYR(driveTrain,
			xArrive ? 0 : xError * kP,
			yArrive ? 0 : yError * kP,
			hArrive ? 0 : hError * kH,
		);
		xError = xTar - curXPos;
		yError = yTar - curYPos;
		hError = curHeading - hTar;
		xArrive = abs(xError) <= 1.5;
		yArrive = abs(yError) <= 1.5;
		hArrive = abs(hError) <= 5;
		delay(interval*1000);
	}
	setDriveXYR(driveTrain, 0, 0, 0);
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
void redRightAuto()
{
	//setPower(lift, 1);
	//delay(1200);
	//setPower(lift, 0);
	//motor[clawY] = 127;
	//delay(400);
	//motor[clawY] = 0;
	//setDriveXYR(driveTrain, -0.6, 1, 0);
	//delay(4000);
	//setDriveXYR(driveTrain, 0, 0, -1);
	//delay(1000);
	//setDriveXYR(driveTrain, 0, -1, 0);
	//delay(1000);
	//setDriveXYR(driveTrain, 0, 0, 0);

	team = red; //Which side of the field is the robot starting on?
	side = pole;
	initPos();
}
void blueRightAuto()
{
	team = blue;
	side = noPole;
	initPos();
}
void redLeftAuto()
{
	team = red;
	side = pole;
	initPos();
}

task autonomous()
{
	startTask(track);
	//blueLeftAuto();
	redRightAuto();
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
