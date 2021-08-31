/*-------------------------------------------------------------
                Project by Matthew Herber
                Concept by Nicholas Seward
-------------------------------------------------------------*/

//Include the neccesary librarys
#include <AccelStepper.h>

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::FULL4WIRE, 22, 23, 24, 25);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 26, 27, 28, 29);
AccelStepper stepper3(AccelStepper::FULL4WIRE, 30, 31, 32, 33);
//Extra Pin Definitions
int stepper1endstop = 34;
int stepper2endstop = 35;
int stepper3endstop = 36;
int ELECTROMAGNET = 37;

const int STEPPER_SHIFT_ANGLE = 70;	//this is the angle that the stepper mount is shifted by for vertical movement
const int MAX_SPEED = 100;
const int MAX_ACCEL = MAX_SPEED * 3;
const float MM_TO_STEP_RATIO = 1; //-0.75;	//This controls the ratio of steps to MM's
const int ZERO_SPEED = 20;	//how quickly should we zero the robot for initial position reset
const int PROGRAM_LINE_COUNT = 100;	//whats the maximum ammount of lines the program can be
const float ARCSPEED = 0.25; //how quickly to move during an arc command
String programList[PROGRAM_LINE_COUNT];	//maximum of 100 commands (changeable ofc, dynamic arrays arent a thing I wanted to implement here)
int lastArrayPoint = 0;	//used to know where we are in the listed program
int programCounter = 0;	//used to count what line we are on in the program
float percentSpeed = 1.0;

void setMotionAccel(int accel = MAX_ACCEL);

/*
Comments are available using #. Comments REQUIRE ';\' still as im lazy ;)

List of movement commands:
lmove(x,y,z)                       |Moves the robot EOAT to the specified X Y Z coordinates in a straight line|
jmove(x,y,z)                       |Moves the robot EOAT to the specified X Y Z coordinates as quick as possible|
delay(ms)                          |Halts program execution for a specified ammount of milliseconds|
speed(%)                           |Sets the speed of the robot based on a percentage of the max speed. This change affects all motion moves after it is called|
accel(%)                           |Sets the acceleration of the robot based on a percentage of the max acceleration. This change affects all motion moves after it is called|
Carc(x,y,z,r,startAngle,endAngle)  |Does a clockwise arc move given a center position, a radius from that center, a start angle and an end angle. always make sure your end angle is larger than your start angle|
CCarc(x,y,z,r,startAngle,endAngle) |Does a counter-clockwise arc move given a center position, a radius from that center, a start angle and an end angle. always make sure your end angle is larger than your start angle|

List of EOAT commands:
EOAT(T/F)                          |Turns on or off the EOAT, 0 for off and 1 for on|
*/
String program = "\
#basic starting program for testing;\
jmove(550,200,100);\
delay(1000);\
jmove(50,50,25);\
jmove(500,50,5);\
jmove(200,100,100);\
jmove(0,0,0);\
delay(1000);\
EOAT(1);\
;\
#Section to test speed command;\
speed(20);\
lmove(300,200,300);\
speed();\
lmove(0,0,0);\
EOAT(0);\
;\
#Section to test accel command;\
delay(1000);\
accel(5);\
lmove(300,200,300);\
accel();\
lmove(0,0,0);\
delay(1000);\
EOAT(1);\
#Rapid section testing individual axis moves;\
#X;\
lmove(200,0,0);\
delay(1000);\
jmove(100,0,0);\
#Y;\
lmove(100,100,0);\
delay(1000);\
jmove(100,0,0);\
#Z;\
lmove(100,0,100);\
delay(1000);\
jmove(100,0,0);\
#ALL AXIS;\
lmove(400,50,200);\
delay(1000);\
jmove(100,0,0);\
#Arc Testing;\
#Carc(400,200,0,200,360,0);\
lmove(0,0,0);\
#CCarc(400,200,0,200,0,360);\
lmove(0,0,0);\
EOAT(0);\
";

//This takes the program string and splits it into commands per line, putting it into programList
void splitProgram()
{
	while (program.indexOf(';') != -1)
	{
		programList[lastArrayPoint] = program.substring(0, program.indexOf(';'));
		program = program.substring(program.indexOf(';') + 1, program.length());
		lastArrayPoint++;
	}
}

void stringToArray(String *ar, String inputStr)
{
	int r = 0, t = 0;

	for (int i = 0; i < inputStr.length(); i++)
	{
		if (inputStr.charAt(i) == ',' || inputStr.charAt(i) == ')')
		{
			ar[t] = inputStr.substring(r, i);
			r = (i + 1);
			t++;
		}
	}
}

//This takes a single line command and figures out what it means and where to send input values, if any
void parseCommand(String command)
{
	//Comment Handling
	if (command.indexOf("#") != -1)
	{
		//do nothing :)
		return;
	}

	//Motion Commands
	else if (command.indexOf("lmove") != -1)
	{
		//Serial.println(command);
		String valueset = command.substring(command.indexOf("(") + 1);
		String values[3];
		stringToArray(values, valueset);
		int x = values[0].toInt();
		int y = values[1].toInt();
		int z = values[2].toInt();
		lmoveToCoordinates(x, y, z);
	}
	else if (command.indexOf("jmove") != -1)
	{
		//Serial.println(command);
		String valueset = command.substring(command.indexOf("(") + 1);
		String values[3];
		stringToArray(values, valueset);
		int x = values[0].toInt();
		int y = values[1].toInt();
		int z = values[2].toInt();
		jmoveToCoordinates(x, y, z);
	}
	else if (command.indexOf("CCarc") != -1)
	{
		String valueset = command.substring(command.indexOf("(") + 1);
		String values[6];
		stringToArray(values, valueset);
		int x = values[0].toInt();
		int y = values[1].toInt();
		int z = values[2].toInt();
		int r = values[3].toInt();
		int sangle = values[4].toInt();
		int eangle = values[5].toInt();
		arcmoveCC(x, y, z, r, sangle, eangle);
	}
	else if (command.indexOf("Carc") != -1)
	{
		String valueset = command.substring(command.indexOf("(") + 1);
		String values[6];
		stringToArray(values, valueset);
		int x = values[0].toInt();
		int y = values[1].toInt();
		int z = values[2].toInt();
		int r = values[3].toInt();
		int sangle = values[4].toInt();
		int eangle = values[5].toInt();
		arcmoveC(x, y, z, r, sangle, eangle);
	}
	else if (command.indexOf("speed()") != -1)
	{
		//Serial.println(command);
		percentSpeed = 1.0;
	}
	else if (command.indexOf("speed") != -1)
	{
		//Serial.println(command);
		String valueset = command.substring(command.indexOf("(") + 1);
		String values[1];
		stringToArray(values, valueset);
		percentSpeed = values[0].toInt() / 100.0;
	}
	else if (command.indexOf("accel()") != -1)
	{
		//Serial.println(command);
		setMotionAccel(MAX_ACCEL);	//Reset motion accel to max
	}
	else if (command.indexOf("accel") != -1)
	{
		//Serial.println(command);
		String valueset = command.substring(command.indexOf("(") + 1);
		String values[1];
		stringToArray(values, valueset);
		setMotionAccel((values[0].toInt() / 100.0) *MAX_ACCEL);	//find the value in the command, then divide it by 100 and multiply it by MAX_ACCEL
	}

	//Time Commands
	if (command.indexOf("delay") != -1)
	{
		//Serial.println(command);
		String valueset = command.substring(command.indexOf("(") + 1);
		String values[1];
		stringToArray(values, valueset);
		delay(values[0].toInt());
	}

  //EOAT Commands
  if (command.indexOf("EOAT") != -1)
  {
    String valueset = command.substring(command.indexOf("(") + 1);
    String values[1];
    stringToArray(values, valueset);
    energizeMagnet(values[0].toInt());
  }
}

void setup()
{
	pinMode(stepper1endstop, INPUT);
	pinMode(stepper2endstop, INPUT);
	pinMode(stepper3endstop, INPUT);
  pinMode(ELECTROMAGNET, OUTPUT); //Set electromagnet pin to output
	setMotionAccel();
	//Serial.begin(9600);
	splitProgram();
	//Initial test move
	reZero();
	jmoveToCoordinates(0, 0, 0);
}

void loop()
{
	stepper1.run();
	stepper2.run();
	stepper3.run();

 
	//if we have reached the position we are trying to move to, we have finished a line and should move to the next line in the program
	if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0)
	{
		parseCommand(programList[programCounter]);
		programCounter++;
	}

	//if we reach the end of the program, go into an infinite loop to stop execution
	if (programCounter == PROGRAM_LINE_COUNT && true)
	{
		//Serial.println("Program Complete!");
		//while(true);	//stop execution by getting stuck in a infinite loop
		programCounter = 0;
	}
}

void moveToCoordinates(float x, float y, float z)
{
	//initial offsets to cart position offsets
	//x = x + 140;
	//y = y + 180;
	//z = 580 - z;

  float stepper1calc = (x - z) / MM_TO_STEP_RATIO;
  float stepper2calc = (x - (y* tan(radians(90 - STEPPER_SHIFT_ANGLE)))) / MM_TO_STEP_RATIO;
  float stepper3calc = (x + z) / MM_TO_STEP_RATIO;

  //if any of the calculated stepper positions return a negative stepper location, error out
  if (stepper1calc < 0 || stepper2calc < 0 || stepper3calc < 0) {
    while (true){
      //infinite loop
    }
  }

	stepper1.moveTo(stepper1calc);
	stepper2.moveTo(stepper2calc);
	stepper3.moveTo(stepper3calc);
}

void lmoveToCoordinates(float x, float y, float z)
{
	moveToCoordinates(x, y, z);
	//abs all of these as we just want the distance, not the direction
	long s1d = abs(stepper1.distanceToGo());
	long s2d = abs(stepper2.distanceToGo());
	long s3d = abs(stepper3.distanceToGo());
	float MaxDistance = max(s1d, s3d);

	float VectorMultiplier = 1.0 / MaxDistance;
	if (s1d == 0 && s3d == 0) { VectorMultiplier = 1.0 / s2d; }	//account for the fact that if both exterior carriages dont move, we will calc NaN for vector multiplier

	stepper1.setMaxSpeed((VectorMultiplier *s1d) *MAX_SPEED *percentSpeed);
	stepper2.setMaxSpeed((VectorMultiplier *s2d) *MAX_SPEED *percentSpeed);
	stepper3.setMaxSpeed((VectorMultiplier *s3d) *MAX_SPEED *percentSpeed);
}

void jmoveToCoordinates(float x, float y, float z)
{
	moveToCoordinates(x, y, z);
	stepper1.setMaxSpeed(MAX_SPEED *percentSpeed);
	stepper2.setMaxSpeed(MAX_SPEED *percentSpeed);
	stepper3.setMaxSpeed(MAX_SPEED *percentSpeed);
}

void arcmoveCC(float x, float y, float z, float r, float sAngle, float eAngle)
{
  float calcXpos = (r* cos(radians(sAngle))) + x;
  float calcYpos = (r* sin(radians(sAngle))) + y;
  jmoveToCoordinates(calcXpos,calcYpos,z);
  float prevpercentSpeed = percentSpeed;
  percentSpeed = ARCSPEED;
	for (int angle = sAngle; angle < eAngle; angle += 15)
	{
		calcXpos = (r* cos(radians(angle))) + x;
		calcYpos = (r* sin(radians(angle))) + y;
		lmoveToCoordinates(calcXpos, calcYpos, z);
		while (stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0 && stepper3.distanceToGo() != 0)
		{
			stepper1.run();
			stepper2.run();
			stepper3.run();
		}
	}
 percentSpeed = prevpercentSpeed;
}

void arcmoveC(float x, float y, float z, float r, float sAngle, float eAngle)
{
  float calcXpos = (r* cos(radians(sAngle))) + x;
  float calcYpos = (r* sin(radians(sAngle))) + y;
  jmoveToCoordinates(calcXpos,calcYpos,z);
  float prevpercentSpeed = percentSpeed;
  percentSpeed = ARCSPEED;
	for (int angle = sAngle; angle > eAngle; angle -= 15)
	{
		calcXpos = (r* cos(radians(angle))) + x;
		calcYpos = (r* sin(radians(angle))) + y;
		lmoveToCoordinates(calcXpos, calcYpos, z);
		while (stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0 && stepper3.distanceToGo() != 0)
		{
			stepper1.run();
			stepper2.run();
			stepper3.run();
		}
	}
	percentSpeed = prevpercentSpeed;
}

void setMotionAccel(int accel)
{
	stepper1.setAcceleration(accel);
	stepper2.setAcceleration(accel);
	stepper3.setAcceleration(accel);
}

void energizeMagnet(int control)
{
  switch(control){
    case 0:
    {
      digitalWrite(ELECTROMAGNET, LOW); //Disable the Electromagnet
      break;
    }
    case 1:
    {
      digitalWrite(ELECTROMAGNET, HIGH); //Enable the Electromagnet
      break;
    }
  }
}

void reZero()
{
  //set the speed of the motors to a slow accurate speed for initial hardware zeroing
	stepper1.setSpeed(ZERO_SPEED);
	stepper2.setSpeed(ZERO_SPEED);
	stepper3.setSpeed(ZERO_SPEED);

  //Keep running till all three endstops are triggered
	while (digitalRead(stepper1endstop) == LOW || digitalRead(stepper2endstop) == LOW || digitalRead(stepper3endstop) == LOW)
	{
		if (digitalRead(stepper1endstop) == LOW)
		{
      stepper1.move(-1);
			stepper1.runSpeedToPosition();
		}

		if (digitalRead(stepper2endstop) == LOW)
		{
      stepper2.move(-1);
			stepper2.runSpeedToPosition();
		}

		if (digitalRead(stepper3endstop) == LOW)
		{
      stepper3.move(-1);
			stepper3.runSpeed();
		}
	}

  int offset = -20;
  //Register the current position of the steppers as the new zero
  stepper1.setCurrentPosition(offset);
  stepper2.setCurrentPosition(offset);
  stepper3.setCurrentPosition(offset);
}
