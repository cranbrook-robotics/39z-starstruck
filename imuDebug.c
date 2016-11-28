#pragma config(UserModel, "39zconfig.c")
#pragma platform(VEX)


int initX, initY, initH; //Initial X, Y, Heading


float curXVel = 0, curYVel = 0, curXPos, curYPos, curHeading; //Current X Velociy, Y Velocity, X Position, Y Position, Heading
float interval = 0.075; //Interval is in SECONDS


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
	float AccThresh = 0.001;
	for( int count = 0; count < 10 && abs(getAcc(XAxis)) > AccThresh && abs(getAcc(YAxis)) > AccThresh; ++count ){
		for (int i = 0; i < 50; i++)
		{
			xC += getAcc(XAxis);
			yC += getAcc(YAxis);
			delay(10);
		}
		accBias[XAxis] = xC/50;
		accBias[YAxis] = yC/50;
	}
}

//Calculates Initial Position and Heading based on current side and team
void initPos()
{
	initX = 0;
	initY = 0;
	initH = 0;
	curHeading = initH;
	SensorValue(gyro) = initH;
	curXPos = initX;
	curYPos = initY;
}

void initGyro(){
	SensorType[gyro] = sensorNone;
	wait1Msec(1000);
	SensorType[gyro] = sensorGyro;
	wait1Msec(2000);
	SensorValue(gyro) = 0;
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
		curHeading = SensorValue(gyro)/10.;
		delay (interval*1000);
	}
}


void pre_auton()
{
	SensorValue(initIndicator) = 1;
	initAcc();
	initGyro();
	SensorValue(initIndicator) = 0;
	clearDebugStream();
}

task main()
{
	pre_auton();
	initPos();
	startTask(track);
	while(true)
	{
		writeDebugStreamLine("CurXVel: %f", curXVel);
		writeDebugStreamLine("CurYVel: %f", curYVel);
		writeDebugStreamLine("CurXPos: %f", curXPos);
		writeDebugStreamLine("CurYPos: %f", curYPos);
		writeDebugStreamLine("CurHeading: %f", curHeading);
		delay(500);

	}
}
