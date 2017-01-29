#pragma config(UserModel, "39zconfig.c") //Cortex Configs
//#include "Vex_Competition_Includes.c" //Uses Vex stuff
#include <CKVexMotorSet.h>
task main()
{
float potVal = SensorValue(clawPot);
clearDebugStream();
while(true)
{
	potVal = SensorValue(clawPot);

	writeDebugStreamLine("Pot: %f", potVal);
	delay(500);
}

}
