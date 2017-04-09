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
#include <HolonomicAuto.h>


float initX, initY, initH; //Initial X, Y, Heading

int wheelAngles[] = {45, 135, 225, 315};
int leftPotVal, rightPotVal;

typedef enum StartingColor {red, blue}; //Color of the Starting Tile (red or blue)
typedef enum StartingPosition {pole, driver}; //Side of the Field of the Starting Tile (near the pole or away from the pole)
StartingColor team;
StartingPosition side;

float  curHeading; //Current X Position, Y Position, Heading
float interval = 0.075; //Interval is in SECONDS

tMotor liftMotors[] = {lLiftT, rLiftT, lLiftB, rLiftB};
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


void pre_auton()
{
	bStopTasksBetweenModes = true;
	bLCDBacklight = true;
	clearLCDLine(0);
	clearLCDLine(1);
	initIME();
	initGyro();
	MotorSetInit (lift, liftMotors, 4);
	InitHolonomicBase(driveTrain, driveMotors, 4);
	initializeHolonomicAuto(driveMotors, wheelAngles);
	displayLCDString(0,0,"INITIALIZING");
}

task lcdManager()
{
	string lcdBatteryVoltages;
	while(true)
	{
		sprintf(lcdBatteryVoltages, "M: %.2f P: %.2f", MainBatteryVoltage(), SensorValue(pPowerExp)/182.4);
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDString(0,0,lcdBatteryVoltages);
		delay(300);
	}
}


void bluePoleAuto()
{
	team = blue;
	side = pole;
	initPos();
	moveToDestination(ws1[0], ws1[1]);
	moveToObject(ws1);
}
void blueDriverAuto()
{
	team = blue;
	side = driver;
	initPos();
}
void redPoleAuto()
{
	team = red;
	side = pole;
	initPos();
}
void redDriverAuto()
{
	team = red;
	side = driver;
	initPos();
}

task autonomous()
{
	if(SensorValue(autoSelection) <= 150)
		blueDriverAuto();
	else if (SensorValue(autoSelection) > 150 && SensorValue(autoSelection) <= 1650)
		bluePoleAuto();
	else if (SensorValue(autoSelection) > 1650 && SensorValue(autoSelection) <= 3250)
		redPoleAuto();
	else if (SensorValue(autoSelection) > 3250)
		redDriverAuto();
}

task clawControl()
{

	while (true)
	{
		SensorValue(clawSolenoid) = vexRT[Btn7D] ? 1 : vexRT[Btn7U] ? 0 : SensorValue(clawSolenoid);
		delay(750);
	}
}

task usercontrol()
{
	while (true)
	{
		startTask(lcdManager);
		startTask(clawControl);
		setDriveXYR(driveTrain, vexRT[Ch4]/127., vexRT[Ch3]/127., vexRT[Ch1]/127.);
		setPower(lift, vexRT[Btn5U] ? 1 : vexRT[Btn5D] ? -1 : 0);
	}
}
