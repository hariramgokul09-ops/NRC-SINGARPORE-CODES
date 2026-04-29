#include <Arduino.h>
#include <smorphi.h>
#include <SoftwareSerial.h>
// -------- UART SETTINGS --------
#define RX_PIN 16    // ESP32 RX2 → Nano TX
#define TX_PIN 17    // ESP32 TX2 → Nano RX (optional)
#define BAUD_RATE 9600

// -------- GLOBALS --------
SoftwareSerial NanoSerial(16,17);
Smorphi my_robot;  // Create Smorphi object

char command = 'S';
unsigned long lastCommandTime = 0;
#define COMMAND_TIMEOUT 1500  // Stop after 1.5s if no command

// -------- SETUP --------
void setup() {
  Serial.begin(115200);  // Debugging
  NanoSerial.begin(9600);
  my_robot.BeginSmorphi();
  // Initialize Smorphi
  
  
  
  Serial.println("Smorphi Receiver Ready...");
}

// -------- LOOP --------
void loop() {
  // Check for commands from Nano
  if (NanoSerial.available()) {
    command = NanoSerial.read();
    lastCommandTime = millis();

    Serial.print("Received Command: ");
    Serial.println(command);

    // Handle commands using Smorphi library
    switch (command) {
      case 'F': my_robot.MoveForward(80); break;
      case 'B': my_robot.MoveBackward(80); break;
      case 'A': my_robot.MoveLeft(80); break;
      case 'D': my_robot.MoveRight(80); break;
      case 'CL': my_robot.CenterPivotLeft(270); break;
      case 'R': my_robot.CenterPivotRight(270); break;  
      case 'S': my_robot.stopSmorphi(); break;
      case 'I': my_robot.I(); break;
      case 'O': my_robot.O(); break;
      case 'L': my_robot.L(); break;
    }
  }

  // Safety timeout → stop if no command received
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    my_robot.stopSmorphi();
  }
}