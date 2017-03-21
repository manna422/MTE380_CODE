#include <NewPing.h>
#include <Servo.h>

/*
 * Ultrasonic Definitions
 */
#define MAX_DISTANCE 275 // Maximum distance (in cm) to ping. width 7.5 x length 8.5 ft = 230x260
#define FRONT_TRIGGER_PIN 30
#define FRONT_ECHO_PIN 31
#define LEFT_TRIGGER_PIN 32
#define LEFT_ECHO_PIN 33
#define RIGHT_TRIGGER_PIN 34
#define RIGHT_ECHO_PIN 35

#define NUM_US_SAMPLE 5
#define CM_WALL_TO_RAMP 34
#define CM_POLE_TO_WALL 19
#define DETECT_TOLERANCE 3

/*
 * Ultrasonic Variables
 */
int leftDis_old, frontDis_old, rightDis_old;
int leftDis, frontDis, rightDis;
NewPing frontUS(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing leftUS(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing rightUS(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);


/*
 * Servo State Defintions
 */
#define lMotorPWMPin 9
#define rMotorPWMPin 6
#define lMotorDeadPos 92 // 180 reverse max
#define rMotorDeadPos 92 // 180 forward max

/*
 * Servo State Variables
 */
Servo lMotor;

Servo rMotor;

int currentLMotorSpeed = lMotorDeadPos;
int currentRMotorSpeed = rMotorDeadPos;
int targetLMotorSpeed;
int targetRMotorSpeed;

bool trackYaw = false;
float targetYaw;
float targetYawMargin = 1.00f;

float goodrspeed = 25;
float goodlspeed = 25;


/*
 * 
 * Dummy IMU Stuff
 */
float yaw;

void setup() {
  Serial.begin(115200);
  
  //detecting twice to fill old values set
  getUSDis();
  getUSDis();

  // Initialize Motors
  lMotor.attach(lMotorPWMPin);
  rMotor.attach(rMotorPWMPin);
  stopBothMotors();
  setRightMotorSpeed(10.0);
  setLeftMotorSpeed(10.0);
  setRightMotorSpeed(goodrspeed);
  setLeftMotorSpeed(goodlspeed);
}

long start = 0;
bool turning = false;
long diffturn = 1000;

void loop() {
  getUSDis();
  printAllUSDiff();
  printAllUSValues();
  
  //assuming ramp is on left side
  int leftDiff= leftDis - leftDis_old;
  if(leftDiff > -1*(CM_WALL_TO_RAMP+DETECT_TOLERANCE) && leftDiff < -1*(CM_WALL_TO_RAMP-DETECT_TOLERANCE)&& leftDis!= 0 ){
    Serial.println("ramp on left");
  }
  
  if(leftDiff + CM_POLE_TO_WALL < DETECT_TOLERANCE && leftDis!= 0 ){
    setLeftMotorSpeed(0);
    start = millis();
    turning = true;
    Serial.println("pole on left");
    Serial.println(leftDiff);
    Serial.println(-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE));
  }
  
  if((rightDis - rightDis_old) <= (-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE)) && rightDis != 0){
    //setRightMotorSpeed(0);
    start = millis();
    turning = true;
    Serial.println("pole on right");
    Serial.println((rightDis - rightDis_old));
    Serial.println(-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE));
  }

  if (turning & abs(millis() - start) > diffturn){
    setRightMotorSpeed(goodrspeed);
    setLeftMotorSpeed(goodlspeed);
    turning = false;
  }
  
  updateMotorState();
  // Other code that *DOESN'T* analyze ping results can go here.
}

void getUSDis(){
  leftDis_old = leftDis;
  rightDis_old = rightDis;
  frontDis_old = frontDis;
  leftDis = leftUS.convert_cm(leftUS.ping_median(NUM_US_SAMPLE));
  rightDis = rightUS.convert_cm(rightUS.ping_median(NUM_US_SAMPLE));
  frontDis = frontUS.convert_cm(frontUS.ping_median(NUM_US_SAMPLE));
}

void printUSValue(int value){
  if(value >= 0){Serial.print(" ");}
  if(abs(value) < 10){Serial.print(" ");}
  if(abs(value) < 100){Serial.print(" ");}  
  Serial.print(value);
}

void printAllUSValues(){
  Serial.print("Ultrasonic: Left = ");
  printUSValue(leftDis);
  Serial.print("  Front = ");
  printUSValue(frontDis);
  Serial.print("  Right = ");
  printUSValue(rightDis);
  Serial.println();
}

void printAllUSDiff(){
  Serial.print("Differeces: Left = ");
  printUSValue(leftDis - leftDis_old);
  Serial.print("  Front = ");
  printUSValue(frontDis - frontDis_old);
  Serial.print("  Right = ");
  printUSValue(rightDis - rightDis_old);
  Serial.print("   ");
}

void updateMotorState() {
  int rampStep = 10;
  
  if (targetLMotorSpeed != currentLMotorSpeed) {
    if (targetLMotorSpeed > currentLMotorSpeed) {
      if (targetLMotorSpeed >= currentLMotorSpeed + rampStep) {
        currentLMotorSpeed += rampStep;
      } else {
        currentLMotorSpeed = targetLMotorSpeed;
      }
    } else {
      if (targetLMotorSpeed <= currentLMotorSpeed - rampStep) {
        currentLMotorSpeed -= rampStep;
      } else {
        currentLMotorSpeed = targetLMotorSpeed;
      }
    }

    lMotor.write(currentLMotorSpeed);
  }
    
  if (targetRMotorSpeed != currentRMotorSpeed) {
    if (targetRMotorSpeed > currentRMotorSpeed) {
      if (targetRMotorSpeed >= currentRMotorSpeed + rampStep) {
        currentRMotorSpeed += rampStep;
      } else {
        currentRMotorSpeed = targetRMotorSpeed;
      }
    } else {
      if (targetRMotorSpeed <= currentRMotorSpeed - rampStep) {
        currentRMotorSpeed -= rampStep;
      } else {
        currentLMotorSpeed = targetLMotorSpeed;
      }
    }

    rMotor.write(currentRMotorSpeed);
  }

  if (trackYaw) {
    float rotationAngle = targetYaw - yaw;
    while (rotationAngle < -180.0f) {
      rotationAngle += 360.0f;
    }
    while (rotationAngle > 180.0f) {
      rotationAngle -= 360.0f;
    }

    if (fabs(rotationAngle) < targetYawMargin) {
      setLeftMotorSpeed(0.0f);
      setRightMotorSpeed(0.0f);
    } 
    
    else if (rotationAngle < 0) {
    // if we need to rotate clockwise(right)

      if (targetLMotorSpeed > lMotorDeadPos - 10) {
        targetLMotorSpeed--;
      } else if (targetRMotorSpeed > rMotorDeadPos - 10) {
        targetRMotorSpeed--;
      }
    }
    
    // if we need to rotate counter-clockwise(left)
    else if (rotationAngle > 0) {
      
      if (targetRMotorSpeed < rMotorDeadPos + 10) {
        targetRMotorSpeed++;
      } else if (targetLMotorSpeed < lMotorDeadPos + 10) {
        targetLMotorSpeed++;
      }
    }

  }    
}

void setLeftMotorSpeed(float speed) {
  if (speed >= 0) {
    targetLMotorSpeed = int(map(speed, 0.0f, 100.0f, lMotorDeadPos, 0));
  } else {
    targetLMotorSpeed = int(map(-1*speed, 0.0f, 100.0f, lMotorDeadPos, 180));
  }
}

void setRightMotorSpeed(float speed) {
  if (speed >= 0) {
    targetRMotorSpeed = int(map(speed, 0.0f, 100.0f, rMotorDeadPos, 180));
  } else {
    targetRMotorSpeed = int(map(-1*speed, 0.0f, 100.0f, rMotorDeadPos, 0));
  }
}

void stopBothMotors() {
  setLeftMotorSpeed(0.0f);
  setRightMotorSpeed(0.0f);
}

