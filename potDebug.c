#pragma config(UserModel, "39zconfig.c") //Cortex Configs
//#include "Vex_Competition_Includes.c" //Uses Vex stuff
#include <CKVexMotorSet.h>
task main()
{
float potVal = SensorValue(pot);
clearDebugStream();
while(true)
{
	potVal = SensorValue(pot);

	writeDebugStreamLine("Pot: %f", potVal);
	delay(500);
}

}
