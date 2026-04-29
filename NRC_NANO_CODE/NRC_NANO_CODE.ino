#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SoftwareSerial.h>

// ---------- HARDWARE ----------
MPU6050 mpu;
SoftwareSerial smorphiSerial(2, 3); // Nano TX=2 → Smorphi RX

// Ultrasonic pins
#define TRIG_FRONT 8
#define ECHO_FRONT 9

// ---------- CONSTANTS ----------
#define SAFE_DISTANCE 40
#define MAX_DISTANCE 300

// ---------- GLOBAL VARIABLES ----------
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float yaw = 0;
int direction = 1; // Used for lane shift direction

// ---------- FUNCTIONS ----------

// Initialize MPU6050
void initMPU() {
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 DMP Ready");
  } else {
    Serial.print("MPU6050 Init Failed! Code: ");
    Serial.println(devStatus);
    while (1);
  }
}



// Get distance from ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return MAX_DISTANCE;

  float distance = duration * 0.034 / 2.0;
  return constrain(distance, 2, MAX_DISTANCE);
}

// Turn robot to an exact 90°

// Move to the next lane (lawnmower pattern)
void moveToNextLane() {
  smorphiSerial.write('S');
  if(direction==1){
      for(int i=0;i<100;i++){
        smorphiSerial.write('D');
        delay(100);
      }
  }
  else if(direction==-1){
      for(int i=0;i<100;i++){
        smorphiSerial.write('A');
        delay(100);
      }

  
  }
  
  
      // Move forward into next lane
  
  smorphiSerial.write('S');
     // Turn back parallel to lanes
  direction *= -1;             // Alternate direction each time
}

// Cover lanes automatically
void coverLane() {
  if(direction==1){
  int frontDistance = getDistance(TRIG_FRONT, ECHO_FRONT);
  Serial.print("Front Distance: ");
  Serial.println(frontDistance);

  if (frontDistance > SAFE_DISTANCE) {
    smorphiSerial.write('F'); // Keep moving forward
  } 
  else {
    smorphiSerial.write('S'); // Stop at obstacle
    moveToNextLane();         // Shift to next lane
  }}
  else if(direction==-1){
    int backdistance = getDistance(11, 12);
  Serial.print("Front Distance: ");
  Serial.println(backdistance);

  if (backdistance > SAFE_DISTANCE) {
    smorphiSerial.write('B'); // Keep moving forward
  } else {
    smorphiSerial.write('S'); // Stop at obstacle
    moveToNextLane();         // Shift to next lane
  }
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(9600);
  smorphiSerial.begin(9600);
  Wire.begin();

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  initMPU();
  Serial.println("Setup Completed. Starting Mapping...");
  delay(2000);
}

// ---------- LOOP ----------
void loop() {
  coverLane();
}