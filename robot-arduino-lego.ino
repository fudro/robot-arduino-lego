#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define rotationL   15  //Define the analog channel to use. Digital pins 14-19 are the same as analog pins A0-A5.
#define rotationR  14
#define lightSensor    16

int countLeft = 0;
int lastCountLeft = 0;
int rotationLeft = 0;
int countRight = 0;
int lastCountRight = 0;
int rotationRight = 0;
int command = 1;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select motor ports: M1, M2, M3 or M4.
Adafruit_DCMotor *motorR = AFMS.getMotor(3);
Adafruit_DCMotor *motorL = AFMS.getMotor(1);

void setup() {
  Serial.begin(9600);
  Serial.println("LEGO Rotation Sensor Test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorR->setSpeed(255);
  motorL->setSpeed(255);
  
  motorR->run(FORWARD);
  motorL->run(FORWARD);
  // turn on motor
  motorR->run(RELEASE);
  motorL->run(RELEASE);
}

void loop() {
  UpdateRotationLeft();
  UpdateRotationRight();
//  DisplayLightSensor();

//  if (command == 1) {
//    command = 0;
//    motorR->run(FORWARD);
//    motorL->run(FORWARD);
//  }
//
//  while (rotationLeft < 100) {
//    
//  }
//  motorR->run(RELEASE);
//  motorL->run(RELEASE);
//  delay(5000);
  
}

int ReadLegoAnalog(int sensorPin) {
  pinMode(sensorPin, OUTPUT);   //Set pin to output mode
  digitalWrite(sensorPin, HIGH);    //Set pin to HIGH to charge the sensor's internal capacitor
  delay(10);    //Wait for sensor to charge up
  pinMode(sensorPin, INPUT);    //Set pin to INPUT mode to read the sensor value
  int value = analogRead(sensorPin);   //Read analog sensor value from the pin
  return value;   //Return the sensor value to the calling function
}

void DisplayLightSensor() {
  int light = ReadLegoAnalog(lightSensor); //Read the raw value of the sensor
  byte percent = map(light, 200, 700, 0, 100);   //Convert raw value to percent. The raw value range of 200 to 700 is based on raw value data from the monitor output. Make sure your range is large enough to encompass all potential raw values or the map caluclation will "max out" and return errant values around 250.
  byte displayValue = 100 - percent;
  Serial.println(displayValue);    //Print the percent value
}

void UpdateRotationLeft() {
  int leftWheel = ReadLegoAnalog(rotationL); //Read the raw value of the sensor
  //Update the count based on the returned raw values. The specific sensor used only returns 3 different raw ranges when rotated. 
  if (leftWheel > 900) {
    countLeft = 1;
  }
  else if (330 < leftWheel && leftWheel <= 340) {
    countLeft = 2;
  }
  else if (450 < leftWheel && leftWheel <= 500) {  //Compared to the raw values returned, the rotation values are "out of order" (4 before 3) to account for how the Rotation sensor is designed to return voltages. 
    countLeft = 3;
  }

  if (countLeft == lastCountLeft) {   //Check if the sensor has moved since the last count. Return if no change in count value.
    return;
  }
  else {    //Update the rotation 
    if (countLeft == 1) {   //Compare count and lastCount to determine direction of rotation. Special cases apply for values of 1 and 3 (where the sequence repeats) depending on direction of rotation.
      if (lastCountLeft == 3) {
        rotationLeft++;   //Rotation increases if the count sequence is "1234,1234..." (when lastCount == 4)
      }
      else {
        rotationLeft--;   //Rotation decreases if the count sequence is "4321, 4321..." (when lastCount == 2)
      }
    }
    else if (countLeft == 3) {
      if (lastCountLeft == 2) {
        rotationLeft++;   //Rotation increases if the count sequence is "1234,1234..." (when lastCount == 3)
      }
      else {
        rotationLeft--;   //Rotation decreases if the count sequence is "4321, 4321..." (when lastCount == 1)
      }
    }
    else if (countLeft > lastCountLeft) {   //All other comparisions follow natural ascending or descending order rules.
      rotationLeft++;
    }
    else if (countLeft < lastCountLeft) {
      rotationLeft--;
    }
/*********USE THIS SECTION TO DISPLAY THE RAW ROTATION AND COUNT VALUES FOR TESTING***************************      
    Serial.print("L  ");
    Serial.print(leftWheel);
    Serial.print("  ");
    Serial.print(countLeft);
    Serial.print("  ");
    Serial.println(rotationLeft);
 *************************************************************************************************************/
    lastCountLeft = countLeft;    //Store the last count
    DisplayRotation();            //Update display. COMMENT OUT THIS STATEMENT WHEN TESTING RAW SENSOR VALUES.
  }
}

void UpdateRotationRight() {
  int rightWheel = ReadLegoAnalog(rotationR); //Read the raw value of the sensor
  //Update the count
  if (rightWheel > 900) {
    countRight = 1;
  }
  else if (450 < rightWheel && rightWheel <= 500) {  //Compared to the raw values returned, the rotation values are "out of order" (4 before 3) to account for how the Rotation sensor is designed to return voltages. 
    countRight = 2;
  }
  if (330 < rightWheel && rightWheel <= 350) {
    countRight = 3;
  }

  if (countRight == lastCountRight) {   //Check if the sensor has moved. Return if no change in count value.
    return;
  }
  else {    //Update the rotation 
    if (countRight == 1) {   //Compare count and lastCount to determine direction of rotation. Special cases apply for values of 1 and 4 depending on direction of rotation.
      if (lastCountRight == 3) {
        rotationRight++;   //Rotation increases if the count sequence is "1234,1234..." (when lastCount == 4)
      }
      else {
        rotationRight--;   //Rotation decreases if the count sequence is "4321, 4321..." (when lastCount == 2)
      }
    }
    else if (countRight == 3) {
      if (lastCountRight == 2) {
        rotationRight++;   //Rotation increases if the count sequence is "1234,1234..." (when lastCount == 3)
      }
      else {
        rotationRight--;   //Rotation decreases if the count sequence is "4321, 4321..." (when lastCount == 1)
      }
    }
    else if (countRight > lastCountRight) {   //All other comparisions follow natural ascending or descending order rules.
      rotationRight++;
    }
    else if (countRight < lastCountRight) {
      rotationRight--;
    }
/*********USE THIS SECTION TO DISPLAY THE RAW ROTATION AND COUNT VALUES FOR TESTING***************************    
    Serial.print("R  ");
    Serial.print(rightWheel);
    Serial.print("  ");
    Serial.print(countRight);
    Serial.print("  ");
    Serial.println(rotationRight);
*************************************************************************************************************/
    lastCountRight = countRight;    //Store the last count
    DisplayRotation();              //Update display.  COMMENT OUT THIS STATEMENT WHEN TESTING RAW SENSOR VALUES.
  }
}

void DisplayRotation() {
  Serial.print("L  ");
  Serial.print(rotationLeft);
  Serial.print("  R  ");
  Serial.println(rotationRight);
}

