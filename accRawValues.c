#pragma config(UserModel, "39zconfig.c")
#pragma platform(VEX)

#pragma DebuggerWindows("debugStream")


task main()
{

	//SensorType[in4] = sensorNone;
	//SensorType[in5] = sensorNone;
	//SensorType[in6] = sensorNone;
	//delay(1000);
	//SensorType[in4] = sensorAccelerometer;
	//SensorType[in5] = sensorAccelerometer;
	//SensorType[in6] = sensorAccelerometer;
	//wait1Msec(2000);

	while(true)
	{
		writeDebugStreamLine("%4d\t%4d\t%4d", SensorValue[in4], SensorValue[in5], SensorValue[in6]);
		delay(500);

	}
}
