#pragma config(UserModel, "39zconfig.c") //Cortex Configs
//#include "Vex_Competition_Includes.c" //Uses Vex stuff
#include <CKVexMotorSet.h>
task main()
{
float potVal1 = SensorValue(leftPot);
float potVal2 = SensorValue(rightPot);
clearDebugStream();
while(true)
{
	potVal1 = SensorValue(leftPot);
	potVal2 = SensorValue(rightPot);
	writeDebugStreamLine("Left Pot: %2f; Right Pot: %2f", potVal1, potVal2);
	delay(500);
}

}
