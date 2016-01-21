#include <Motor.h>
#define QTR_THRESHOLD 600
#define SETPOINT 4500

#define INTERSECT_BLINK_RATE 5
#define BLINK_RATE 3

#define REDUCE 80

#define GND 28
#define led 13

Motor *motor;

motorPins pins = {2, 3, 5, 4};

int sensor[] = {54, 55, 56, 57, 58, 59, 60, 61};
int reading[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int analogReading[8] = {0, 0, 0  , 0, 0, 0, 0, 0};


int total_active_sensor = 0;


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





int iMotorSpeed = 135;

int reduced_speed = iMotorSpeed - REDUCE;

float kp = 1;
float kd = 10;
float ki = 0;

//Init sensor
void initSensor(void){
  for (int i = 0; i < 8; i++){
    pinMode(sensor[i], INPUT);
  }
}


void blink(int times){
  for (int i = 0; i < times; i++){
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }
}


//error
float error = 0;
float previousError = 0;
float totalError = 0;
float power = 0;

int PWM_Left = 0;
int PWM_Right = 0;

int iLastRead;

int iReadArray(void){
  int iRead = 0;
  int iActive = 0;

  for (int i = 0; i < 8; i++){
    if (analogRead(sensor[i]) > QTR_THRESHOLD){
      reading[i] = 1;
      iRead += (i+1) * 1000;
      iActive++;
    } else {
      reading[i] = 0;
    }
  }

  total_active_sensor = iActive;

  iRead = map(iRead/iActive, 0, 8000, 0, 1023);
  
  if (!iRead) return iLastRead;
  else {
    iLastRead = iRead;
    return iRead;
  }
}

bool checkIntersection(void){
  updateIr();
  iReadArray();
  if ((total_active_sensor >= 7)){
    return true;
  } else {
    return false;
  }
}

void PID(void){
  int avgSensor = iReadArray();

  previousError = error;
  error = avgSensor - map(SETPOINT, 0, 8000, 0, 1023);

  totalError += error;

  power = (kp * error) + (kd*(error - previousError)) + (ki*totalError);

  Serial.println("power " + String(power));

  if (power > iMotorSpeed) { power = iMotorSpeed; }
  if (power < -1 * iMotorSpeed) { power = -1 * iMotorSpeed; }

  

  if (power < 0){
    PWM_Right = iMotorSpeed;
    PWM_Left = iMotorSpeed - abs(int(power));
  } else {
    PWM_Right = iMotorSpeed - int(power);
    PWM_Left = iMotorSpeed;
  }

  motor->go(PWM_Left, PWM_Right, FORWARD);

  Serial.println("Left speed: " + String(PWM_Left));
  Serial.println("Right speed: " + String(PWM_Right));
}


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  initSensor();
  motor = new Motor(pins);

  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);
  pinMode(led, OUTPUT);
  setupIr();
  
}

void stop(){
  motor->go(0, 0, NOWHERE);
}


void turnLeft(void){
   iReadArray();
   if (reading[0] == 1 && reading[3] == 1){
   iReadArray();
   while(reading[6] != 1){
     motor->go(0, reduced_speed, FORWARD);
     delay(1);
     iReadArray();
   }
 } 
}

void turnRight(void){
   iReadArray();
   if (reading[7] == 1 && reading[5] == 1){
   iReadArray();
   while(reading[3] != 1){
     motor->go(reduced_speed, 0, FORWARD);
     delay(1);
     iReadArray();
   }
 } 
}

void loop() {

  iReadArray();
  updateIr();

  turnLeft();
  turnRight();

  if (checkIntersection()){
        motor->go(iMotorSpeed, iMotorSpeed, FORWARD);
        delay(1);
  } 

 
  else PID();


}
