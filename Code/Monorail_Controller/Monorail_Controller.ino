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

const int STEPPER_SHIFT_ANGLE = 70; //this is the angle that the stepper mount is shifted by for vertical movement
const int MAX_SPEED = 200;
const int MAX_ACCEL = 100;
const float MM_TO_STEP_RATIO = 62.99;
const int ZERO_SPEED = 100; //how quickly should we zero the robot for initial position reset
const int PROGRAM_LINE_COUNT = 100; //whats the maximum ammount of lines the program can be
String programList[PROGRAM_LINE_COUNT]; //maximum of 100 commands (changeable ofc, dynamic arrays arent a thing I wanted to implement here)
int lastArrayPoint = 0; //used to know where we are in the listed program
int programCounter = 0; //used to count what line we are on in the program

void setMotionSpeed(int speed = MAX_SPEED);
void setMotionAccel(int accel = MAX_ACCEL);

/*
modify this to add motion moves to the robot
Comments are available using #. Comments REQUIRE ';\' still as im lazy ;)

List of available commands:
move(x,y,z) |Moves the robot EOAT to the specified X Y Z coordinates|
delay(ms)   |Halts program execution for a specified ammount of milliseconds|
speed(%)    |Sets the speed of the robot based on a percentage of the max speed. This change affects all motion moves after it is called|
accel(%)    |Sets the acceleration of the robot based on a percentage of the max acceleration. This change affects all motion moves after it is called|
*/
String program = "\
#basic starting program for testing;\
move(100,200,300);\
delay(3000);\
move(0,50,25);\
move(300,0,5);\
move(100,200,300);\
move(0,0,0);\
delay(1000);\
;\
#Section to test speed command;\
speed(20);\
move(100,200,300);\
speed();\
move(0,0,0);\
;\
#Section to test accel command;\
delay(1000);\
accel(5);\
move(100,200,300);\
accel();\
move(0,0,0);\
";

//This takes the program string and splits it into commands per line, putting it into programList
void splitProgram(){
  while(program.indexOf(';') != -1){
    programList[lastArrayPoint] = program.substring(0, program.indexOf(';'));
    program = program.substring(program.indexOf(';') + 1, program.length());
    lastArrayPoint++;
  }
}

//This takes a single line command and figures out what it means and where to send input values, if any
void parseCommand(String command){
  //Comment Handling
  if(command.indexOf("#") != -1){
    //do nothing :)
    return;
  }

  //Motion Commands
  if(command.indexOf("move") != -1){
    //Serial.println(command);
    String valueset = command.substring(command.indexOf("(") + 1, command.indexOf(')'));
    int x = command.substring(command.indexOf("(") + 1, command.indexOf(',')).toInt();
    int y = command.substring(command.indexOf(",") + 1, command.indexOf(',',command.indexOf(",") + 1)).toInt();
    int z = command.substring(command.indexOf(",",command.indexOf(",", command.indexOf(",")) + 1) + 1, command.indexOf(')')).toInt();
    moveToCoordinates(x,y,z);
  }
  if(command.indexOf("speed") != -1){
    //Serial.println(command);
    setMotionSpeed((command.substring(command.indexOf("(") + 1, command.indexOf(')')).toInt() / 100.0) * MAX_SPEED); //find the value in the command, then divide it by 100 and multiply it by MAX_SPEED
  }
  if(command.indexOf("speed()") != -1){
    //Serial.println(command);
    setMotionSpeed(MAX_SPEED); //Reset motion speed to max
  }
  if(command.indexOf("accel") != -1){
    //Serial.println(command);
    setMotionAccel((command.substring(command.indexOf("(") + 1, command.indexOf(')')).toInt() / 100.0) * MAX_ACCEL); //find the value in the command, then divide it by 100 and multiply it by MAX_ACCEL
  }
  if(command.indexOf("accel()") != -1){
    //Serial.println(command);
    setMotionAccel(MAX_ACCEL); //Reset motion accel to max
  }

  //Time Commands
  if(command.indexOf("delay") != -1){
    //Serial.println(command);
    delay(command.substring(command.indexOf("(") + 1, command.indexOf(')')).toInt());
  }
}

void setup()
{
    pinMode(stepper1endstop, INPUT);
    pinMode(stepper2endstop, INPUT);
    pinMode(stepper3endstop, INPUT);
    setMotionSpeed();
    setMotionAccel();
    //Serial.begin(9600);
    splitProgram();
    //Initial test move
    reZero();
    moveToCoordinates(0,0,0);
}

void loop()
{
    stepper1.run();
    stepper2.run();
    stepper3.run();

    //if we have reached the position we are trying to move to, we have finished a line and should move to the next line in the program
    if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0){
      parseCommand(programList[programCounter]);
      programCounter++;
    }

    //if we reach the end of the program, go into an infinite loop to stop execution
    if(programCounter == PROGRAM_LINE_COUNT && false){
      Serial.println("Program Complete!");
      while(true); //stop execution by getting stuck in a infinite loop
    }
}

void moveToCoordinates(float x, float y, float z){
  //initial offsets to cart position offsets
  //x = x + 140;
  //y = y + 180;
  //z = 580 - z;
  
  stepper1.moveTo((x - z) * MM_TO_STEP_RATIO);
  stepper2.moveTo((x - (y * tan(90 - STEPPER_SHIFT_ANGLE))) * MM_TO_STEP_RATIO);
  stepper3.moveTo((x + z) * MM_TO_STEP_RATIO);
}

void setMotionSpeed(int speed){
  stepper1.setMaxSpeed(speed);
  stepper2.setMaxSpeed(speed);
  stepper3.setMaxSpeed(speed);
}

void setMotionAccel(int accel){
  stepper1.setAcceleration(accel);
  stepper2.setAcceleration(accel);
  stepper3.setAcceleration(accel);
}

void reZero(){
  stepper1.setSpeed(ZERO_SPEED);
  stepper2.setSpeed(ZERO_SPEED);
  stepper3.setSpeed(ZERO_SPEED);
  while(digitalRead(stepper1endstop) == LOW || digitalRead(stepper2endstop) == LOW || digitalRead(stepper3endstop) == LOW){
    if(digitalRead(stepper1endstop) == LOW){
      stepper1.runSpeed();
    }
    if(digitalRead(stepper2endstop) == LOW){
      stepper2.runSpeed();
    }
    if(digitalRead(stepper3endstop) == LOW){
      stepper3.runSpeed();
    }
  }
}
