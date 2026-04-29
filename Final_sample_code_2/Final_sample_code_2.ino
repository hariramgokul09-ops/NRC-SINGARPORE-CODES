#include <smorphi.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
HUSKYLENS huskylens;
SoftwareSerial myHuskySerial(14, 13);

void printResult(HUSKYLENSResult result){
if (result.command == COMMAND_RETURN_BLOCK){
Serial.println(result.ID);
}

else if (result.command == COMMAND_RETURN_ARROW){
Serial.println("Wrong mode");
}
else{
Serial.println("Object unknown!");
}
}
int color_signature;
int command_block;
// Ultrasonic sensor pins
#define TRIG_FRONT 18
#define ECHO_FRONT 19
#define TRIG_LEFT  23
#define ECHO_LEFT  25
#define TRIG_RIGHT 26
#define ECHO_RIGHT 27
#define TRIG_BACK  4
#define ECHO_BACK  5

// Minimum clear distance (cm) before considering a lane change
const int SAFE_DISTANCE = 30;

// Smorphi instance
Smorphi my_robot;

// Measure distance from an HC-SR04-style sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
  long cm = (duration / 2) / 29.1;
  return cm;
}

void setup() {
  Serial.begin(115200);

  // Ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT);  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);   pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_BACK,OUTPUT);    pinMode(ECHO_BACK,INPUT);  // Start the Smorphi module
  my_robot.BeginSmorphi();
  Serial.println("Smorphi ready");
  myHuskySerial.begin(9600);
	while (!huskylens.begin(myHuskySerial)){
	Serial.println(F("Begin failed!"));
	Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
	Serial.println(F("2.Please recheck the connection."));
	delay(100);
}
}
int direction =1;
void loop() {
  // Read distances
  int x = 0;
  if (huskylens.request()) {
  if (huskylens.available())  {
  HUSKYLENSResult result = huskylens.read();
  printResult(result);
  if (result.command == COMMAND_RETURN_BLOCK) {
  x=result.ID;
  
  }
  }else{
  Serial.println("Error!");
  x=0;
  }}
  float dF = getDistance(TRIG_FRONT, ECHO_FRONT);
  delay(10);
  float dL = getDistance(TRIG_LEFT, ECHO_LEFT);
  delay(10);
  float dR = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  delay(10);
  float dB= getDistance(TRIG_BACK,ECHO_BACK);
  x=0;
  if(x==1){
    my_robot.stopSmorphi();
    delay(1000);
    my_robot.L();
    delay(2000);
    my_robot.stopSmorphi();
    delay(1000);
    
  }
  else if(x==2){
    my_robot.L();
  }
  else if(x==3){
    my_robot.I();
  }

  // Debug output
  Serial.print("F:"); Serial.print(dF);
  Serial.print(" L:"); Serial.print(dL);
  Serial.print(" R:"); Serial.println(dR);
  Serial.print(" B:"); Serial.println(dB);

  // If front is clear, go forward
  if (dF > SAFE_DISTANCE) {
    my_robot.MoveForward(80);
  } 
  else {
    // Obstacle ahead → prepare lane change
    my_robot.stopSmorphi();
    delay(200);

    // Prefer right lane if clear
    if (dR > SAFE_DISTANCE) {
      my_robot.MoveRight(80);
      delay(500);
    }
    // Else try left
    else if (dL > SAFE_DISTANCE) {
      my_robot.MoveLeft(80);
      delay(500);
    }
    // If both sides blocked, back up briefly
    else {
      my_robot.MoveBackward(80);
      delay(500);
    }
  }
  
  // Short pause before next cycle
  delay(100);
  if (direction==1){
    if(dF>SAFE_DISTANCE){
      my_robot.MoveForward(80);
      delay(10);
    }
    else{
      my_robot.MoveRight(80);
      delay(500);
      
    }
    direction=-1;
  } 

  if (direction==-1){
    if(dB>SAFE_DISTANCE+20){
    my_robot.MoveBackward(80);
      delay(10);
    }
    else{
      my_robot.MoveRight(80);
      delay(500);
      
    }} 
    direction=1;
}
