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

//Tracks Current Location, Velocity, and Heading while the track task is being run
task track()
{
	while (true)
	{
		curXPos += sqrt(2) * (lFrontIME.position + rBackIME.position - rFrontIME.position - lBackIME.position) / 4.0;
		curYPos += sqrt(2) * (lFrontIME.position + lBackIME.position + rFrontIME.position+ rBackIME.position) / 4.0;
		curHeading = SensorValue(gyro);
		delay (interval*1000);
	}
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
	SensorValue(initIndicator) = 1;
	initIME();
	initGyro();
	MotorSetInit (lift, liftMotors, 3);
	SensorValue(initIndicator) = 0;
	InitHolonomicBase(driveTrain, driveMotors, 4);
}

void blueLeftAuto()
{
	team = blue;
	side = pole;
	initPos();
	moveToPoint(ws9, 180);
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
		setDriveXYR(driveTrain, vexRT[Ch4]/127., vexRT[Ch3]/127., vexRT[Ch1]/127.);
		motor[clawY] = vexRT[Btn6UXmtr2] ? 127 : vexRT[Btn6DXmtr2] ? -127 : 0;
		setPower(lift, vexRT[Btn5UXmtr2] ? 1 : vexRT[Btn5DXmtr2] ? -1 : 0);


	}
}
