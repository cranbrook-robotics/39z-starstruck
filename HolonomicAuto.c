float wheelTargetDisp[] = {0, 0, 0, 0};
float wheelDisp[] = {0, 0, 0, 0};
float wheelCruisingVelocity[] = {0, 0, 0, 0};
float wheelHeadings[4];
float driveMotors[4];
float wheelPercentDisp[] = {0, 0, 0, 0};
float wheelCurrentVelocity[] = {0, 0, 0, 0};
float curXPos, curYPos;
float tarXPos, tarYPos;
float tarHead, curHead;

typedef struct Vector
{
	float direction;
	float magnitude;
}

struct Vector targetVector;

void calcTarVector()
{
		float tarYDisp = tarYPos - curYPos;
		float tarXDisp = tarXPos - curXPos;
		targetVector.direction = atan2(tarYDisp, tarXDisp);
		targetVector.magnitude = sqrt(tarYDisp * tarYDisp + tarXDisp * tarXDisp);
}

void calcWheelTargetDisp()
{
	tarHead = targetVector.direction - curHead;
	for (int i = 0; i < 4; i++)
	{
		wheelTargetDisp[i] = targetVector.magnitude * cos(wheelHeadings[i] - tarHead);
	}
}
task main()
{



}
