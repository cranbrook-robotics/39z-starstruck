#pragma config(UserModel, "39zconfig.c")
#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)
#include <Vex_Competition_Includes.c>
#include <CKVexMotorSet.h>
#include <CKHolonomic.h>
#include <ZipTiesSFCS.h>

float potVal, clawPotVal;
tMotor liftMotors[] = {lLiftT, rLiftT, botLift};
MotorSet lift; //MotorSet of Right Lift Motors
int clawTar;
float curHeading;

tMotor driveMotors[] = {rBack, rFront, lFront, lBack}; //Array of Drive Motors
HolonomicBase driveTrain; //HolonomicBase of Drive Train Motors

void initGyro(){
	SensorType[gyro] = sensorNone; //Fixes common RobotC error with initializing Gyroscope
	wait1Msec(1000);
	SensorType[gyro] = sensorGyro;
	wait1Msec(2000);
	SensorValue(gyro) = 0; //Default Gyro heading is 0
}

//Sets arm to a position along Potentiometer
void setArm(float pos)
{
	potVal = SensorValue(pot);
	while (abs(potVal - pos) > 10)
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

task setClaw()
{
	while (true)
	{
		clawPotVal = SensorValue(clawPot);
		if (vexRT[Btn6UXmtr2])
		{
			motor[clawY] = 127;
			clawTar = clawPotVal;
		}
		else if (vexRT[Btn6DXmtr2])
		{
			motor[clawY] = -127;
			clawTar = clawPotVal;
		}
		else if ((clawPotVal - clawTar) > 30)
			motor[clawY] = 127;
		else if ((potVal - clawTar) > 30)
			motor[clawY] = -127;
		else
			motor[clawY] = 0;
		clawTar = vexRT[Btn8LXmtr2] ? 1000 : vexRT[Btn8DXmtr2] ? 1800 : vexRT[Btn8RXmtr2] ? 2800 : clawTar;
		delay(300);
	}
}

task trackGyro()
{
	while(true)
	{
		curHeading = SensorValue(gyro);
		delay(30);
	}
}
void pre_auton()
{
	bStopTasksBetweenModes = true;
	SensorValue(initIndicator) = 1;
	initGyro();
	MotorSetInit (lift, liftMotors, 3);
	SensorValue(initIndicator) = 0;
	InitHolonomicBase(driveTrain, driveMotors, 4);
}

void leftAuto()
{
	//Pick up preload star
	motor[clawY] = 127;
	delay(1500);
	motor[clawY] = 0;
	setArm(230);

	//Move forward from base tile
	setDriveXYR(driveTrain,0,1,0);
	delay(1000);

	//Turn so back is to wall
	setDriveXYR(driveTrain,0,0,1);
	delay(1250);

	//Drive backwards into the wall
	setDriveXYR(driveTrain,0,-1,0);

	//Drop star behind fence
	setPower(lift, 1);
	motor[clawY] = -127;
	delay(1000);
	motor[clawY] = 0;

	//Move to new position along fence
	setDriveXYR(driveTrain,0,1,0);
	delay(750);
	setDriveXYR(driveTrain,0,0,1);
	delay(1250);
	setDriveXYR(driveTrain,-1,1,0);
	delay(1250);

	//Move towards fence again
	setDriveXYR(driveTrain,0,1,0);
	delay(1000);
	setDriveXYR(driveTrain,0,-1,0);
	delay(2000);
	setDriveXYR(driveTrain,0,0,0);
}

void rightAuto()
{
	//Pick up preload star
	motor[clawY] = 127;
	delay(1500);
	motor[clawY] = 0;
	setArm(230);

	//Move forward from base tile
	setDriveXYR(driveTrain,0,1,0);
	delay(1000);

	//Turn so back is to wall
	setDriveXYR(driveTrain,0,0,1);
	delay(1250);

	//Drive backwards into the wall
	setDriveXYR(driveTrain,0,-1,0);

	//Drop star behind fence
	setPower(lift, 1);
	motor[clawY] = -127;
	delay(1000);
	motor[clawY] = 0;

	//Move to new position along fence
	setDriveXYR(driveTrain,0,1,0);
	delay(750);
	setDriveXYR(driveTrain,0,0,1);
	delay(1250);
	setDriveXYR(driveTrain,1,1,0);
	delay(1250);

	//Move towards fence again
	setDriveXYR(driveTrain,0,1,0);
	delay(1000);
	setDriveXYR(driveTrain,0,-1,0);
	delay(2000);
	setDriveXYR(driveTrain,0,0,0);
}

task autonomous()
{
	startTask(trackGyro);
	leftAuto();
	//rightAuto();
}

task usercontrol()
{
	clawTar = 2800;
	while (true)
	{
		startTask(setClaw);
		potVal = SensorValue(pot);
		setDriveXYR(driveTrain, vexRT[Ch4]/127., vexRT[Ch3]/127., vexRT[Ch1]/127.);

		setPower(lift, vexRT[Btn5UXmtr2] ? 1 : vexRT[Btn5DXmtr2] ? -1 : 0);
	}
}
