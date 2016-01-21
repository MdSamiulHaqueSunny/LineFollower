#include "LineFollower.h"
#include <SoftwareSerial.h>
#define led 13
#define BLINK_RATE 0

#define TURN_RIGHT_SPEED 60
#define TURN_LEFT_SPEED 60


#define SPEED_REDUCER 40

SoftwareSerial bt(10, 11);

using namespace pdlf;

byte s[] = {54, 55, 56, 57, 58, 59, 60, 61};
//byte s[] = {61, 60, 59, 58, 57, 56, 55, 54};
byte lm[] = { 2, 3 };
byte rm[] = { 5, 4 };
//Put m for mega
// and u for uno
char mcu = 'm';

/*
 * Left and Right IR subroutines
 * 
 * 
 */
//left and right sensor
byte leftIr = 63;
byte rightIr = 62;

const int tcrt_threshold = 700;
bool tcrt_inverse_logic = false;

int left_reading = 0;
int right_reading = 0;



//Returns digital reading from threshold
int leftDigitalRead(bool inverted = false){
  if (!inverted){
    if (analogRead(leftIr) > tcrt_threshold) { left_reading = 1; return 1; }
    else { left_reading = 0; return 0;}
  } else {
    if (analogRead(leftIr) < tcrt_threshold) { left_reading = 1; return 1; }
    else { left_reading = 0; return 0;}
  }
}


int rightDigitalRead(bool inverted = false){
  if (!inverted){
    if (analogRead(rightIr) > tcrt_threshold) { right_reading = 1; return 1; }
    else { right_reading = 0; return 0;}
  } else {
    if (analogRead(rightIr) < tcrt_threshold) { right_reading = 1; return 1; }
    else { right_reading = 0; return 0;}
  }
}




//Updates current reading of irs
void updateIr(void){
  left_reading = leftDigitalRead();
  right_reading = rightDigitalRead();
}

//Debug left and right sensor
void debugIr(void){
  updateIr();
  Serial.println("====== BEGIN =======");
  Serial.print("LEFT : ");
  Serial.println(analogRead(leftIr));
  Serial.print(" ");
  Serial.println(left_reading);
  Serial.print("RIGHT : ");
  Serial.println(analogRead(rightIr));
  Serial.print(" ");
  Serial.println(right_reading);
  Serial.println("====== END =======");
}

//Setting up irs
void setupIr(void){
  pinMode(leftIr, INPUT);
  pinMode(rightIr, INPUT);
}


Robot *robot;

void setup()
{
  Serial.begin(9600);
  // robot.printAnalog();
  // robot.initializeComponents();
  // robot.updateWeightedValue();
  // robot.generateWeight(8);
  // robot.printWeight();
  // robot.printDigital();
  // robot.printWeightedValue();
  
  robot = new Robot(lm, rm, s, 8, mcu);
  robot->initializeComponents();
  Robot::THRESHOLD = 600;
  robot->generateWeight(8);
  
  robot->setKp(10);
  robot->setKd(1);
  pinMode(led, OUTPUT);
  Robot::global_speed = 115;

  setupIr();
  bt.begin(9600);
}

void blink(int times){
//  for (int i = 0; i < times; i++){
//    digitalWrite(led, HIGH);
//    delay(200);
//    digitalWrite(led, LOW);
//    delay(200);
//  }
}


void debug(void){
    robot->updateAnalogRead();
  robot->updateDigitalRead();
  updateIr();

  robot->printAnalog();
  robot->printDigital();
  debugIr();

  delay(1000);  
}

void loop()
{

 
 updateIr();
 robot->updateDigitalRead();
 
 if (robot->s_digital_reading[0] == 1 && robot->s_digital_reading[3] == 1){
   robot->updateDigitalRead();
   while(robot->s_digital_reading[6] != 1){
     robot->run(0, TURN_LEFT_SPEED, Robot::Forward, Robot::Forward);
     delay(1);
     robot->updateDigitalRead();
   }
 } 

 else if (robot->s_digital_reading[7] == 1 && robot->s_digital_reading[5] == 1){
   robot->updateDigitalRead();
   while(robot->s_digital_reading[3] != 1){
     robot->run(TURN_RIGHT_SPEED, 0, Robot::Forward, Robot::Forward);
     delay(1);
     robot->updateDigitalRead();
   }
 } 

 
 
   else if ((robot->s_digital_reading[3] == 1 || robot->s_digital_reading[4] == 1 || robot->s_digital_reading[5] == 1) && right_reading == 1){
     bt.println("ACUTE DETECTED");
     
     //robot->updateDigitalRead();
     
     do {
       updateIr();
       robot->updateDigitalRead();
       robot->run(Robot::global_speed - SPEED_REDUCER, Robot::global_speed - SPEED_REDUCER, Robot::Forward, Robot::Forward);
       bt.println("FORWARD");
     } while (robot->s_digital_reading[3] == 1 || robot->s_digital_reading[4] == 1);
     
//      while (robot->s_digital_reading[3] == 0 && robot->s_digital_reading[4] == 1 && robot->s_digital_reading[5] == 1){
//        bt.println("FORWARD");
//        updateIr();
//        robot->run(Robot::global_speed, Robot::global_speed, Robot::Forward, Robot::Forward);
//      }

     bt.println("STOP FOR A WHILE");
     robot->run(Robot::Nowhere);
     blink(BLINK_RATE);

     do {
       robot->updateDigitalRead();
       robot->run(Robot::global_speed - SPEED_REDUCER , Robot::global_speed - SPEED_REDUCER, Robot::Clockwise);
     } while (robot->s_digital_reading[7] == 0);

     bt.println("STOP FOR A WHILE");
     robot->run(Robot::Nowhere);
     blink(BLINK_RATE);
   }



    else if ((robot->s_digital_reading[3] == 1 || robot->s_digital_reading[4] == 1 || robot->s_digital_reading[5] == 1) && left_reading == 1){
     bt.println("ACUTE DETECTED");
     
     //robot->updateDigitalRead();
     
     do {
       updateIr();
       robot->updateDigitalRead();
       robot->run(Robot::global_speed - SPEED_REDUCER, Robot::global_speed - SPEED_REDUCER, Robot::Forward, Robot::Forward);
       bt.println("FORWARD");
     } while (robot->s_digital_reading[3] == 1 || robot->s_digital_reading[4] == 1);
     
//      while (robot->s_digital_reading[3] == 0 && robot->s_digital_reading[4] == 1 && robot->s_digital_reading[5] == 1){
//        bt.println("FORWARD");
//        updateIr();
//        robot->run(Robot::global_speed, Robot::global_speed, Robot::Forward, Robot::Forward);
//      }

     bt.println("STOP FOR A WHILE");
     robot->run(Robot::Nowhere);
     blink(BLINK_RATE);

     do {
       robot->updateDigitalRead();
       robot->run(Robot::global_speed - SPEED_REDUCER, Robot::global_speed - SPEED_REDUCER, Robot::AntiClockwise);
     } while (robot->s_digital_reading[0] == 0);

     bt.println("STOP FOR A WHILE");
     robot->run(Robot::Nowhere);
     blink(BLINK_RATE);
   }

   
   //Stop condition
  


      else if ((robot->s_digital_reading[4] == 0 || 
               robot->s_digital_reading[5] == 0 ) && (robot->s_digital_reading[0] == 1 && robot->s_digital_reading[7] == 1)){
                while (true){
                  robot->run(Robot::Nowhere);
                }
               }
   
   
  //Cross 
  

   
   else {
    robot->pdLineFollow();
 }

 if ( robot->s_digital_reading[1] == 1 &&   robot->s_digital_reading[2] == 1 &&   robot->s_digital_reading[3] == 1 && robot->s_digital_reading[4] == 1 &&   robot->s_digital_reading[5] == 1){
    robot->run(Robot::global_speed, Robot::global_speed, Robot::Forward, Robot::Forward);
  }
 

}
