#include <smorphi.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

// --- Huskylens setup ---
HUSKYLENS huskylens;
SoftwareSerial myHuskySerial(27, 26); // RX, TX

// --- Ultrasonic sensor pins ---
#define TRIG_FRONT 18
#define ECHO_FRONT 19
#define TRIG_BACK  4
#define ECHO_BACK  5
#define TRIG_LEFT  23
#define ECHO_LEFT  25

// --- Safety distance (cm) ---
const int SAFE_DISTANCE = 30;

// --- Smorphi instance ---
Smorphi my_robot;

// --- Huskylens result print ---
void printResult(HUSKYLENSResult result){
  if (result.command == COMMAND_RETURN_BLOCK){
    Serial.print("Detected ID: ");
    Serial.println(result.ID);
  }
  else if (result.command == COMMAND_RETURN_ARROW){
    Serial.println("Wrong mode");
  }
  else{
    Serial.println("Object unknown!");
  }
}

// --- Distance measurement ---
float getDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
  long cm = duration * 0.034 / 2;
  return cm;
}

// --- Reset motors ---
void resetMotors(){
  my_robot.stopSmorphi();
  delay(300);
  my_robot.sm_reset_M1();
  my_robot.sm_reset_M2();
  my_robot.sm_reset_M3();
  my_robot.sm_reset_M4();
}

void setup() {
  Serial.begin(115200);

  // --- Ultrasonic setup ---
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);  pinMode(ECHO_BACK, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);

  // --- Initialize Smorphi ---
  my_robot.BeginSmorphi();
  Serial.println("Smorphi ready");

  // --- Initialize Huskylens ---
  myHuskySerial.begin(9600);
  while (!huskylens.begin(myHuskySerial)){
    Serial.println(F("Huskylens begin failed! Check Serial 9600 & wiring"));
    delay(100);
  }
}

void loop() {
  int shapeID = 0;

  // --- Huskylens read ---
  if (huskylens.request() && huskylens.available()) {
    HUSKYLENSResult result = huskylens.read();
    printResult(result);
    if (result.command == COMMAND_RETURN_BLOCK) shapeID = result.ID;
  }

  // --- Shape commands override normal motion ---
  if (shapeID == 1){
    resetMotors();
    my_robot.O();
  }
  else if (shapeID == 2){
    resetMotors();
    my_robot.L();
  }
  else if (shapeID == 3){
    resetMotors();
    my_robot.I();
  }
  else {
    // --- Front distance check inside else block ---
    float dF = getDistance(TRIG_FRONT, ECHO_FRONT);

    if (dF > SAFE_DISTANCE){
      // Path clear → move forward
      my_robot.MoveForward(80);
    }
    else if (getDistance(TRIG_LEFT, ECHO_LEFT) > SAFE_DISTANCE){
      // Lane shift right
      my_robot.MoveRight(80);
      delay(800);
      my_robot.stopSmorphi();

      // Move backward until obstacle detected
      float dB = getDistance(TRIG_BACK, ECHO_BACK);
      while (dB > SAFE_DISTANCE){
        my_robot.MoveBackward(80);
        delay(50);
        dB = getDistance(TRIG_BACK, ECHO_BACK);
      }
      my_robot.stopSmorphi();
      delay(300);

      // Move forward after backing up
      my_robot.MoveForward(80);
      delay(800);
      my_robot.stopSmorphi();
    }
    else {
      // No lane shift possible → stop
      my_robot.stopSmorphi();
    }
  }

  delay(50); // loop delay for sensor stability
}