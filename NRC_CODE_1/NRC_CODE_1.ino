#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <smorphi.h>
#include <SoftwareSerial.h>

Smorphi my_robot;
MPU6050 mpu;
SoftwareSerial BT(26, 27);  // Bluetooth serial

// ================== CONSTANTS ==================
#define ARENA_CM 300         
#define BLOCK_CM 20.0        
#define OVERLAP_CM 2.0
#define FORWARD_SPEED 150
#define TURN_SPEED 120
#define SAFE_DISTANCE 15

#define TRIG_FRONT 23
#define ECHO_FRONT 25
#define TRIG_LEFT 4
#define ECHO_LEFT 5
#define TRIG_RIGHT 19
#define ECHO_RIGHT 18

// Calibrate this value for your robot! (cm per second at FORWARD_SPEED)
#define ROBOT_SPEED_CM_S 20.0  

// ================== VARIABLES ==================
const int ROWS = ARENA_CM / BLOCK_CM;
const int COLS = ARENA_CM / BLOCK_CM;
bool visited[15][15] = {false};
int currentRow = 0, currentCol = 0;
float yaw = 0;
unsigned long lastPrintTime = 0;

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// ================== SETUP ==================
void setup() {
  BT.begin(9600);
  Wire.begin();

  // Ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    BT.println("✅ MPU6050 DMP Initialized");
  } else {
    BT.println("❌ MPU6050 DMP Init Failed");
    while (1);
  }

  BT.println("✅ Smorphi Arena Coverage Ready");
}

// ================== FUNCTIONS ==================
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance = duration * 0.034 / 2.0;
  if (distance == 0) distance = 300;  // Max distance fallback
  return distance;
}

float getYaw() {
  if (!dmpReady) return yaw;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180 / M_PI;
  }
  return yaw;
}

void turnToYaw(float targetYaw) {
  float error;
  BT.println("🔄 Aligning Yaw");
  do {
    float currentYaw = getYaw();
    error = targetYaw - currentYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    if (error > 2) my_robot.CenterPivotRight(TURN_SPEED);
    else if (error < -2) my_robot.CenterPivotLeft(TURN_SPEED);
    else my_robot.stopSmorphi();
  } while (abs(error) > 2);
  my_robot.stopSmorphi();
  BT.println("✅ Yaw Aligned");
  delay(200);
}

bool moveForwardCm(float distanceCm) {
  float targetDistance = distanceCm - OVERLAP_CM;
  unsigned long travelTime = (targetDistance / ROBOT_SPEED_CM_S) * 1000;

  BT.print("➡️ Moving Forward: ");
  BT.print(targetDistance);
  BT.println(" cm");

  unsigned long start = millis();
  my_robot.MoveForward(FORWARD_SPEED);

  while (millis() - start < travelTime) {
    float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    if (front < SAFE_DISTANCE) {
      BT.println("⚠️ Obstacle Detected - Backtracking");
      my_robot.stopSmorphi();
      delay(200);
      my_robot.MoveBackward(FORWARD_SPEED);
      delay(300);
      my_robot.stopSmorphi();

      float left = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
      float right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
      if (left > right) {
        BT.println("↩️ Pivoting Left");
        my_robot.CenterPivotLeft(TURN_SPEED);
        delay(400);
      } else {
        BT.println("↪️ Pivoting Right");
        my_robot.CenterPivotRight(TURN_SPEED);
        delay(400);
      }
      my_robot.stopSmorphi();
      return false;
    }
  }

  my_robot.stopSmorphi();
  BT.println("✅ Forward Complete");
  delay(100);
  return true;
}

bool moveToNextUndiscoveredLane() {
  BT.println("🔍 Searching Next Lane");
  bool leftClear = (currentCol > 0) && !visited[currentRow][currentCol - 1];
  bool rightClear = (currentCol < COLS - 1) && !visited[currentRow][currentCol + 1];

  if (leftClear && readUltrasonic(TRIG_LEFT, ECHO_LEFT) > SAFE_DISTANCE) {
    BT.println("↩️ Shifting Left");
    my_robot.CenterPivotLeft(TURN_SPEED);
    delay(300);
    moveForwardCm(BLOCK_CM / 4);
    my_robot.CenterPivotRight(TURN_SPEED);
    delay(300);
    currentCol--;
    return true;
  } else if (rightClear && readUltrasonic(TRIG_RIGHT, ECHO_RIGHT) > SAFE_DISTANCE) {
    BT.println("↪️ Shifting Right");
    my_robot.CenterPivotRight(TURN_SPEED);
    delay(300);
    moveForwardCm(BLOCK_CM / 4);
    my_robot.CenterPivotLeft(TURN_SPEED);
    delay(300);
    currentCol++;
    return true;
  }

  BT.println("❌ No Lane Found");
  return false;
}

void exploreFullArena() {
  bool movingForward = true;
  for (currentRow = 0; currentRow < ROWS; currentRow++) {
    BT.print("🟦 Exploring Row: ");
    BT.println(currentRow);

    if (movingForward) {
      for (currentCol = 0; currentCol < COLS; currentCol++) {
        BT.print("📍 At Block [");
        BT.print(currentRow);
        BT.print(",");
        BT.print(currentCol);
        BT.println("]");

        if (!visited[currentRow][currentCol]) {
          bool success = moveForwardCm(BLOCK_CM);
          if (!success) moveToNextUndiscoveredLane();
          visited[currentRow][currentCol] = true;
        }

        if (millis() - lastPrintTime >= 10000) {
          int covered = 0;
          for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
              if (visited[r][c]) covered++;
            }
          }
          float coveragePercent = (covered * 100.0) / (ROWS * COLS);
          BT.print("📊 Coverage: ");
          BT.print(coveragePercent, 1);
          BT.println("%");
          lastPrintTime = millis();
        }
      }
    } else {
      for (currentCol = COLS - 1; currentCol >= 0; currentCol--) {
        BT.print("📍 At Block [");
        BT.print(currentRow);
        BT.print(",");
        BT.print(currentCol);
        BT.println("]");

        if (!visited[currentRow][currentCol]) {
          bool success = moveForwardCm(BLOCK_CM);
          if (!success) moveToNextUndiscoveredLane();
          visited[currentRow][currentCol] = true;
        }
      }
    }

    movingForward = !movingForward;
    turnToYaw(round(getYaw() / 90) * 90);
  }
}

// ================== MAIN LOOP ==================
void loop() {
  exploreFullArena();
  my_robot.stopSmorphi();

  int covered = 0;
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      if (visited[r][c]) covered++;
    }
  }
  float coveragePercent = (covered * 100.0) / (ROWS * COLS);
  BT.print("✅ Full Coverage Complete: ");
  BT.print(covered);
  BT.print("/");
  BT.print(ROWS * COLS);
  BT.print(" | ");
  BT.print(coveragePercent, 1);
  BT.println("%");

  while (1);  // Stop forever
}
