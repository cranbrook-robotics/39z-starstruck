#pragma config(UserModel, "C:/Users/rstudent/code/39-competition/config.c") //Cortex Configs
//#include "Vex_Competition_Includes.c" //Uses Vex stuff
#include <CKVexMotorSet.h>
task main()
{
float potValLeft = SensorValue(potLeft);
float potValRight = SensorValue(potRight);
clearDebugStream();
while(true)
{
	potValLeft = SensorValue(potLeft);
	potValRight = SensorValue(potRight);

	writeDebugStreamLine("Left: %f", potValLeft);
	writeDebugStreamLine("Right: %f", potValRight);
	delay(500);
}

}
