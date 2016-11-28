#pragma config(UserModel, "39zconfig.c")
#pragma platform(VEX)

task main()
{


	while(true)
	{
		writeDebugStreamLine("CurXAcc: %f", SensorValue[acX]);
		writeDebugStreamLine("CurYAcc: %f", SensorValue[acY]);
		delay(500);

	}
}
