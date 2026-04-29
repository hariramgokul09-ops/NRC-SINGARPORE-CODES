#include <smorphi.h>
Smorphi my_robot;
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
HUSKYLENS huskylens;
SoftwareSerial myHuskySerial(19, 18);

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

int front_sensor_status;


int right_sensor_status;

int rear_sensor_status ;

int left_sensor_status;

int front_sensor2_status;

int rear_right_sensor;

int rear_forward_sensor;

int rear_left_sensor;
String last_DIR;

// Function to move left safely with obstacle monitoring
void MoveLeftSafely() {
    // Check that left, front-left, rear-left are free
    if (left_sensor_status == 1 && front_sensor_status == 1 && rear_left_sensor == 1) {
        my_robot.MoveLeft(60);   // Move left
        delay(300);              // Small delay to stabilize
    } else {
        // If blocked, try small pivot or stop
        my_robot.stopSmorphi();  // Stop safely
        delay(200);
    }
}


void MoveRightSafely() {
    // Check that right, front-right, rear-right are free
    if (right_sensor_status == 1 && front_sensor2_status == 1 && rear_right_sensor == 1) {
        my_robot.MoveRight(60);  // Move right
        delay(300);              // Small delay to stabilize
    } else {
        // If blocked, try small pivot or stop
        my_robot.stopSmorphi();  // Stop safely
        delay(200);
    }
}



void L_mvmts() {

  if  (rear_right_sensor==0 && left_sensor_status==1){
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();

    my_robot.MoveLeft(70);
    delay(1000);
    my_robot.CenterPivotLeft(270);
    delay(1000);

  }
  else if (front_sensor_status == 0 && right_sensor_status == 0) {
      // Obstacle ahead + right blocked → move left
      my_robot.sm_reset_M1();
      my_robot.sm_reset_M2();
      my_robot.sm_reset_M3();
      my_robot.sm_reset_M4();
      my_robot.MoveLeft(80);
      last_DIR = "DONE";
  }

  else if (front_sensor_status == 0 && left_sensor_status == 0) {
      // Obstacle ahead + left blocked → move right
      my_robot.sm_reset_M1();
      my_robot.sm_reset_M2();
      my_robot.sm_reset_M3();
      my_robot.sm_reset_M4();
      my_robot.MoveRight(80);
      last_DIR = "Right";
  }

  else if (front_sensor_status == 0) {
      // Obstacle ahead only → decide based on last direction
      my_robot.sm_reset_M1();
      my_robot.sm_reset_M2();
      my_robot.sm_reset_M3();
      my_robot.sm_reset_M4();

      if (last_DIR == "Right") {
          my_robot.MoveRight(80);
      } else {
          my_robot.MoveLeft(80);
      }
  }

  else if (right_sensor_status == 0) {
      // Right blocked → move forward
      my_robot.MoveForward(60);
      last_DIR = "Fwd";
  }

  else if (left_sensor_status == 0) {
      // Left blocked → move forward
      my_robot.MoveForward(60);
  }

  else {
      // No obstacles anywhere → keep moving forward
      my_robot.MoveForward(80);
  }

}
// Describe this function...
void movements() {
  if (left_sensor_status == 0 && right_sensor_status == 0 && (front_sensor_status==1 && front_sensor2_status==1)) {
    my_robot.MoveForward(60);
  }
  else if ((left_sensor_status == 0 && right_sensor_status == 0) && (front_sensor_status==0 || front_sensor2_status==0)) {
    my_robot.CenterPivotRight(270);
    delay(1000);
  }
  else if (front_sensor_status == 0 && right_sensor_status == 0) {
    my_robot.CenterPivotLeft(270);
    delay(1000);
  }
   else if (front_sensor_status == 0 && left_sensor_status == 0) {
    my_robot.CenterPivotRight(270);
    delay(1000);
  }
   else if (right_sensor_status == 0) {
    my_robot.CenterPivotLeft(270);
    delay(10);
  }
   else if (left_sensor_status == 0) {
    my_robot.CenterPivotRight(270);
    delay(10);
  }
  else if (front_sensor_status == 0  && front_sensor2_status==1) {

    my_robot.CenterPivotRight(270);
    delay(1000);
  }
  else if (front_sensor_status == 1  && front_sensor2_status==0) {
    
    my_robot.CenterPivotLeft(270);
    delay(1000);
  }
  else if(front_sensor_status == 0  && front_sensor2_status==0){
    my_robot.MoveBackward(80);
    delay(200);
    my_robot.CenterPivotRight(270);
    delay(1000);
  }
  else {
    my_robot.MoveForward(60);
  }

}

bool coverage_complete=false;
int direction=1;
// Describe this function...
int last_obstacle_direction=0;
void I_mvmts() {
  if (coverage_complete == false) {
    Serial.println("IN COVERAGE MODE");

    // ---------------------------
    // SWEEPING COVERAGE MOVEMENT
    // ---------------------------
    if (direction == 1) {  
      // Moving forward
      if (front_sensor_status == 0) {  
        // Obstacle ahead detected
        if (right_sensor_status == 1) {
          // Free on the right → shift right and reverse direction
          my_robot.sm_reset_M1();
          my_robot.sm_reset_M2();
          my_robot.sm_reset_M3();
          my_robot.sm_reset_M4();
          
          my_robot.MoveRight(65);
          delay(650);
          my_robot.CenterPivotLeft(270);
          delay(250);
          direction = -1;  // Start moving backward next
        }
        
        else {
          // Both sides blocked → assume coverage done
          coverage_complete = true;
        }
      }
      else {
        // Path ahead is free → move forward
        my_robot.MoveForward(80);
      }
    } 
    
    else {  
      // Moving backward
      if (rear_sensor_status == 0) {  
        // Obstacle behind detected
        if (right_sensor_status == 1) {
          // Free on the right → shift right and reverse direction
          my_robot.sm_reset_M1();
          my_robot.sm_reset_M2();
          my_robot.sm_reset_M3();
          my_robot.sm_reset_M4();

          my_robot.MoveRight(65);
          delay(650);
          direction = 1;  // Start moving forward next
        }
        
        else {
          // Both sides blocked → assume coverage done
          coverage_complete = true;
        }
      }
      else {
        // Path behind is free → move backward
        my_robot.MoveBackward(80);
      }
    }
  } 

else {
    // ---------------------------
    // OBSTACLE AVOIDANCE MODE (AFTER COVERAGE COMPLETE)
    // ---------------------------
    Serial.println("IN OBSTACLE AVOIDANCE MODE");

    // CASE 1: Both sides blocked but front fully free → move forward
    if (left_sensor_status == 0 && right_sensor_status == 0 && 
        front_sensor_status == 1 && front_sensor2_status == 1) {
        my_robot.MoveForward(60);
        delay(250);
    }

    // CASE 2: Both sides + both front sensors blocked → pivot right safely
    else if (left_sensor_status == 0 && right_sensor_status == 0 &&
             front_sensor_status == 0 && front_sensor2_status == 0) {
        my_robot.CenterPivotRight(270);
        delay(1000);
    }

    // CASE 3: Obstacle on front-left only → move right if safe
    else if (front_sensor_status == 0 && front_sensor2_status == 1) {
        if (right_sensor_status == 1 && rear_right_sensor == 1) {
            my_robot.MoveRight(60);
            delay(250);
        } else {
            my_robot.CenterPivotRight(90);
            delay(400);
        }
    }

    // CASE 4: Obstacle on front-right only → move left if safe
    else if (front_sensor_status == 1 && front_sensor2_status == 0) {
        if (left_sensor_status == 1 && rear_left_sensor == 1) {
            my_robot.MoveLeft(60);
            delay(250);
        } else {
            my_robot.CenterPivotLeft(90);
            delay(400);
        }
    }

    // CASE 5: Both front sensors blocked + right blocked → pivot left
    else if ((front_sensor_status == 0 || front_sensor2_status == 0) && right_sensor_status == 0) {
        my_robot.CenterPivotLeft(90);
        delay(400);
    }

    // CASE 6: Both front sensors blocked + left blocked → pivot right
    else if ((front_sensor_status == 0 || front_sensor2_status == 0) && left_sensor_status == 0) {
        my_robot.CenterPivotRight(90);
        delay(400);
    }

    // CASE 7: Right blocked → pivot left safely
    else if (right_sensor_status == 0) {
        last_obstacle_direction = 2;
        if (front_sensor_status == 1 && rear_right_sensor == 1) {
            my_robot.MoveForward(60);
            delay(250);
        } else {
            my_robot.CenterPivotLeft(90);
            delay(400);
        }
    }

    // CASE 8: Left blocked → pivot right safely
    else if (left_sensor_status == 0) {
        last_obstacle_direction = 1;
        if (front_sensor_status == 1 && rear_left_sensor == 1) {
            my_robot.MoveForward(60);
            delay(250);
        } else {
            my_robot.CenterPivotRight(90);
            delay(400);
        }
    }

    // CASE 9: Both front sensors blocked → reverse then pivot safely
    else if (front_sensor_status == 0 && front_sensor2_status == 0) {
        my_robot.MoveBackward(60);
        delay(250);

        if (last_obstacle_direction == 1) {
            if (left_sensor_status == 1 && rear_left_sensor == 1) {
                my_robot.MoveLeft(60);
                delay(250);
            } else {
                my_robot.CenterPivotRight(90);
                delay(400);
            }
        } else {
            if (right_sensor_status == 1 && rear_right_sensor == 1) {
                my_robot.MoveRight(60);
                delay(250);
            } else {
                my_robot.CenterPivotLeft(90);
                delay(400);
            }
        }
    }

    // CASE 10: Free path → move forward
    else {
        my_robot.MoveForward(60);
        delay(250);
    }
}






}


// Describe this function...
void sensor_initialisation() {
  if ((my_robot.sm_getShape()) == ('i')) {
    front_sensor_status = my_robot.module1_sensor_status(2);
    front_sensor2_status=my_robot.module1_sensor_status(0);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module4_sensor_status(4);
    rear_left_sensor= my_robot.module4_sensor_status(6);
    rear_right_sensor = my_robot.module4_sensor_status(0);
    left_sensor_status = my_robot.module1_sensor_status(10);
    I_mvmts();
  }

  if ((my_robot.sm_getShape()) == ('o')) {
    front_sensor_status = my_robot.module1_sensor_status(0);
    front_sensor2_status= my_robot.module4_sensor_status(4);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(6);
    left_sensor_status = my_robot.module4_sensor_status(0);
    movements();
  }
  if ((my_robot.sm_getShape()) == ('l')) {
    front_sensor_status = my_robot.module1_sensor_status(0); 
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(0);
    left_sensor_status = my_robot.module1_sensor_status(10);
    rear_forward_sensor = my_robot.module4_sensor_status(4);
    rear_right_sensor= my_robot.module4_sensor_status(6);
    L_mvmts();
  }

}


void setup() {
  Serial.begin(115200);
  my_robot.BeginSmorphi();
  myHuskySerial.begin(9600);
	while (!huskylens.begin(myHuskySerial)){
	Serial.println(F("Begin failed!"));
	Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
	Serial.println(F("2.Please recheck the connection."));
	delay(100);
}

}

void loop() {

  int x = 0;
  if (huskylens.request()) {
  if (huskylens.available())  {
  HUSKYLENSResult result = huskylens.read();
  printResult(result);
  if (result.command == COMMAND_RETURN_BLOCK) {
  x=result.ID;
  }
  }
  }else{
  Serial.println("Error!");}
  sensor_initialisation();
  last_DIR = "";

  if (x == 1 && (my_robot.sm_getShape()) != ('o')) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.O();
    front_sensor_status = my_robot.module1_sensor_status(0);
    front_sensor2_status= my_robot.module4_sensor_status(4);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(6);
    left_sensor_status = my_robot.module4_sensor_status(0);
  }

   else if (x == 2 && (my_robot.sm_getShape()) != ('i')) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.I();
    front_sensor_status = my_robot.module1_sensor_status(0);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module4_sensor_status(4);
    left_sensor_status = my_robot.module1_sensor_status(10);
  }
  else if (x == 3 && (my_robot.sm_getShape()) != ('l')) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.L();
    front_sensor_status = my_robot.module1_sensor_status(0); 
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(0);
    left_sensor_status = my_robot.module1_sensor_status(10);
    rear_right_sensor= my_robot.module4_sensor_status(6);
    rear_forward_sensor = my_robot.module4_sensor_status(4);

  }
   else {
    Serial.println("There is no colour present");
  }


}