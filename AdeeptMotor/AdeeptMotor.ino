/***********************************************************
File name:  AdeeptMotor.ino
Description:  
1. Under remote control mode: 
The car is completely controlled by the remote control. 
2. Under auto-control mode: 
The ultrasonic module will keep detecting obstacles in front 
of the car. When encountering and approaching one, the car will 
go backward, turn to another angle bypassing the obstacle, and 
continue to go forward. 
Website: www.adeept.com
E-mail: support@adeept.com
Author: Tom
Date: 2016/12/26 
***********************************************************/
#include <SPI.h>
#include "RF24.h"
#include <Servo.h>

RF24 radio(9, 10);                // define the object to control NRF24L01
byte addresses[5] = "00007";      // define communication address which should correspond to remote control
int data[9]={512, 512, 0, 0, 1, 1, 512, 512, 512};  // define array used to save the communication data
int mode[1];

Servo dirServo;                  // define servo to control turning of smart car
int dirServoPin = 2;              // define pin for signal line of the last servo
float dirServoOffset = 6;         // define a variable for deviation(degree) of the servo
Servo ultrasonicServo;           // define servo to control turning of ultrasonic sensor
int ultrasonicPin = 3;            // define pin for signal line of the last servo
float ultrasonicServoOffset = 15; // define a variable for deviation(degree) of the servo

int trigPin = 0;                  // define Trig pin for ultrasonic ranging module
int echoPin = 1;                  // define Echo pin for ultrasonic ranging module
float maxDistance = 200;          // define the range(cm) for ultrasonic ranging module, Maximum sensor distance is rated at 400-500cm.
float soundVelocity = 340;        // Sound velocity = 340 m/s
float rangingTimeOut = 2 * maxDistance / 100 / soundVelocity * 1000000; // define the timeout(ms) for ultrasonic ranging module

#define FORWARD HIGH
#define BACKWARD LOW

const int dirAPin = 7;    // define pin used to control rotational direction of motor A
const int pwmAPin = 6;    // define pin for PWM used to control rotational speed of motor A
const int dirBPin = 4;    // define pin used to control rotational direction of motor B
const int pwmBPin = 5;    // define pin for PWM used to control rotational speed of motor B
const int snsAPin = 0;    // define pin for detecting current of motor A
const int snsBPin = 1;    // define pin for detecting current of motor B
const int buzzerPin = 8;  // define pin for buzzer
const int RPin = A3; 
const int GPin = A4; 
const int BPin = A5; 
int RGBVal = 0;
int automatic = 0;

#define FORWARD LOW
#define BACKWARD HIGH

void setup() {
  radio.begin();                      // initialize RF24
  radio.setRetries(0, 15);            // set retries times
  radio.setPALevel(RF24_PA_LOW);      // set power
  radio.openReadingPipe(1, addresses);// open delivery channel
  radio.startListening();             // start monitoring

//  Serial.begin(9600); // initialize serial port
  dirServo.attach(dirServoPin);  // attaches the servo on servoDirPin to the servo object
  ultrasonicServo.attach(ultrasonicPin);  // attaches the servo on ultrasonicPin to the servo object
  pinMode(dirAPin, OUTPUT);   // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);   // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);   // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);   // set pwmBPin to output mode
  pinMode(buzzerPin, OUTPUT); // set buzzerPin to output mode
  pinMode(RPin, OUTPUT);   // set RPin to output mode
  pinMode(GPin, OUTPUT);   // set GPin to output mode
  pinMode(BPin, OUTPUT);   // set BPin to output mode

  pinMode(trigPin, OUTPUT); // set trigPin to output mode
  pinMode(echoPin, INPUT);  // set echoPin to input mode
}

void loop()
{
    receiveData();
    if(automatic == 0){
      mode[0]=0;
    //  radio.write( mode, sizeof(mode) );
      // calculate the steering angle of servo according to the direction joystick of remote control and the deviation
      int dirServoDegree = map(data[0], 0, 1023, 135, 45) - (data[7] - 512) / 25; 
      int ultrasonicServoDegree = map(data[8], 0, 1023, 135, 45) - (data[7] - 512) / 25; 
      // get the steering angle and speed of servo according to the speed joystick of remote control and the deviation
      int motorSpd = data[1] - 512 + (data[6] - 512) / 10;
      bool motorDir = motorSpd > 0 ? BACKWARD : FORWARD;
      motorSpd = abs(constrain(motorSpd, -512, 512));
      motorSpd = map(motorSpd, 0, 512, 0, 255);
      // control the steering and travelling of the smart car
      ctrlCar0(dirServoDegree,ultrasonicServoDegree, motorDir, motorSpd);
      switch(RGBVal){
        case 0: digitalWrite(RPin, HIGH);digitalWrite(GPin, HIGH);digitalWrite(BPin, HIGH);break;
        case 1: digitalWrite(RPin, LOW);digitalWrite(GPin, LOW);digitalWrite(BPin, LOW);   break;
        case 2: digitalWrite(RPin, LOW);digitalWrite(GPin, HIGH);digitalWrite(BPin, HIGH); break;
        case 3: digitalWrite(RPin, HIGH);digitalWrite(GPin, LOW);digitalWrite(BPin, HIGH); break;
        case 4: digitalWrite(RPin, HIGH);digitalWrite(GPin, HIGH);digitalWrite(BPin, LOW); break;
        default: break;
      }
  }

  if(automatic == 1){
    
      byte barDistance = maxDistance; // save the minimum measured distance from obstacles
      byte barDegree;                 // save the minimum measured angel from obstacles
      byte distance;                  // save the current the measured distance from obstacles
      mode[0]=1;
     // radio.write( mode, sizeof(mode) );
      // define the initial scanning position servo of pan tilt
      ultrasonicServo.write(40 + ultrasonicServoOffset);
      delay(200);
      // start to scan distance. During this progress, we will get the distance and angle from the closest obstacle
      for (byte ultrasonicServoDegree = 40; ultrasonicServoDegree < 140; ultrasonicServoDegree += 10) {
        ultrasonicServo.write(ultrasonicServoDegree + ultrasonicServoOffset); // steer pan tilt to corresponding position
        delay(50);                // wait 50ms between pings (about 20 pingsc). 29ms should be the shortest delay between pings.
        receiveData();
        distance = getDistance(); // detect the current distance from obstacle with angle of pan tilt stable
        if (distance < barDistance) { // if the current measured distance is smaller than the previous one, save the data of current measured distance
          barDegree = ultrasonicServoDegree;       // save the measured angle
          barDistance = distance;     // save the measured distance
        }
      }
    
      // servo of pan tilt turns to default position
      delay(200);
      ultrasonicServo.write(90 + ultrasonicServoOffset);
    
      int spd = 128;  // set the speed(0-255) of smart car
      // According to the result of scanning control action of intelligent vehicles
      if (barDistance < 20) {     // if the obstacle distance is too close, reverse the travelling direction
        if (barDegree < 90)       // choose to reverse direction according to the angle with obstacle
          ctrlCar1(135,  FORWARD, spd);  // control steering and reversing smart car
        else
          ctrlCar1(45,  FORWARD, spd);   // control steering and reversing smart car
        for(int i=0;i<15;i++){
        delay(100);              // reversing time
        receiveData();
        }
      }
      else if (barDistance < 50) {// if the obstacle distance is too close, reverse the travelling direction
        if (barDegree < 90)       // choose to reverse direction according to the angle with obstacle
          ctrlCar1(135, BACKWARD, spd);   // control steering and moving on
        else
          ctrlCar1(45, BACKWARD, spd);    // control steering and moving on
        for(int i=0;i<10;i++){
        delay(100);              // reversing time
        receiveData();
        }
      }
      else {                     // if the obstacle distance is not close, move on
        ctrlCar1(90, BACKWARD, spd);      // control the smart car move on
        for(int i=0;i<10;i++){
        delay(100);              // reversing time
        receiveData();
        }
      }
      ctrlCar1(90, BACKWARD, 0);   // make the smart car stop for preparation of next scanning
   }
  // send motor current to serial port  
//  float iMotorA = analogRead(snsAPin) * 5.0 / 1024 / 30 / 0.05;
//  float iMotorB = analogRead(snsBPin) * 5.0 / 1024 / 30 / 0.05;

//  Serial.print("iMotorA: ");
//  Serial.print(iMotorA);
//  Serial.print(" A ,iMotorB: ");
//  Serial.print(iMotorB);
//  Serial.println(" A");
}
void receiveData(){
   if ( radio.available()) {             // if receive the data
    while (radio.available()) {         // read all the data
      radio.read( data, sizeof(data) ); // read data
    }
    if(!data[2]){
    RGBVal++ ;
    if(RGBVal>4){
      RGBVal=0;}
    }
    if(!data[3]){
      automatic = 0;
    }
    if(!data[4]){
      automatic = 1;
    }  
    if (!data[5])// control the buzzer
      tone(buzzerPin, 2000);
    else
      noTone(buzzerPin);
   }
}
void ctrlCar0(byte dirServoDegree,byte ultrasonicServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree + dirServoOffset);
  ultrasonicServo.write(ultrasonicServoDegree + ultrasonicServoOffset);
  digitalWrite(dirAPin, motorDir);
  digitalWrite(dirBPin, motorDir);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
}
void ctrlCar1(byte dirServoDegree, bool motorDir, byte motorSpd) {
  dirServo.write(dirServoDegree + dirServoOffset);
  digitalWrite(dirAPin, motorDir);
  digitalWrite(dirBPin, motorDir);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
}

float getDistance() {
  unsigned long pingTime; // save the high level time returned by ultrasonic ranging module
  float distance;         // save the distance away from obstacle

  // set the trigPin output 10us high level to make the ultrasonic ranging module start to measure
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // get the high level time returned by ultrasonic ranging module
  pingTime = pulseIn(echoPin, HIGH, rangingTimeOut);

  if (pingTime != 0) {  // if the measure is not overtime
    distance = pingTime * soundVelocity / 2 / 10000;  // calculate the obstacle distance(cm) according to the time of high level returned
    return distance;    // return distance(cm)
  }
  else                  // if the measure is overtime
    return maxDistance; // returns the maximum distance(cm)
}
