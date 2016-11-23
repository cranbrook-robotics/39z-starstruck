#pragma config(UserModel, "39zconfig.c")
#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)
#include <Vex_Competition_Includes.c>
#include <CKVexMotorSet.h>
#include <CKHolonomic.h>
#include <ZipTiesSFCS.h>


int initX, initY, initH; //Initial X, Y, Heading
float potVal; //Potentiometer Value for Left Arm Tower, Right Arm Tower

typedef enum StartingColor {red, blue}; //Color of the Starting Tile (red or blue)
typedef enum StartingPosition {pole, noPole}; //Side of the Field of the Starting Tile (near the pole or away from the pole)
StartingColor team;
StartingPosition side;

float curXVel = 0, curYVel = 0, curXPos, curYPos, curHeading; //Current X Velociy, Y Velocity, X Position, Y Position, Heading
float interval = 0.075; //Interval is in SECONDS

tMotor liftMotors[] = {lLiftT, rLiftT, botLift};
MotorSet lift; //MotorSet of Right Lift Motors

tMotor driveMotors[] = {rBack, rFront, lFront, lBack}; //Array of Drive Motors
HolonomicBase driveTrain; //HolonomicBase of Drive Train Motors

float startX = 35.126; //All 4 starting tiles have +- this X Coord
float startY = 58.543; //All 4 starting tiles have +- this Y Coord

typedef enum AccAxis { XAxis, YAxis }; //Axes Accelerometer Uses
tSensors accPorts[] = {acX, acY}; //Ports Accelerometer Uses
float accBias[] = {0, 0};

//Calculate Value the Accelerometer Returns
float getAcc(AccAxis axis)
{
	return SensorValue(accPorts[axis])*0.5234 - 1071.7 - accBias[axis]; //Conversion from Accelerometer Returns to in/s^2
}

//Initializes the Accelerometer by Calculating Bias
void initAcc(){
	curXVel = 0;
	curYVel = 0;
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

//Sets arm to a position along Potentiometer
void setArm(float pos)
{
	potVal = SensorValue(pot);
	while (potVal != pos)
	{
		potVal = SensorValue(pot);
		if (potVal > pos)
		{
			setPower(lift, -1);
		}
		else if (potVal < pos)
		{
			setPower(lift, 1);
		}
	}
	setPower(lift, 0);
}

//Calculates Initial Position and Heading based on current side and team
void initPos()
{
	initX = startX * (side == pole ? 1 : -1);
	initY = startY * (team == blue ? 1 : -1);
	initH = team == blue ? PI : 0;
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
		curXVel += getAcc(XAxis)*interval; // m/s^2 * s/1000 * unit conversion
		curYVel += getAcc(YAxis)*interval;
		curXPos += curXVel*interval;
		curYPos += curYVel*interval;
		curHeading = degreesToRadians(SensorValue(gyro)/10.);
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
	initAcc();
	MotorSetInit (lift, liftMotors, 3);
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
		potVal = SensorValue(pot);
		setDriveXYR(driveTrain, vexRT[Ch4]/127., vexRT[Ch3]/127., vexRT[Ch1]/127.);
		setPower(lift, vexRT[Btn5U] ? 1 : vexRT[Btn5D] ? -1 : 0);
		motor[clawY] = vexRT[Btn6U] ? 127 : vexRT[Btn6D] ? -127 : 0;
	}
}
