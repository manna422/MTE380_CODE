#include <NewPing.h>

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define FRONT_TRIGGER_PIN 12
#define FRONT_ECHO_PIN 11
#define LEFT_TRIGGER_PIN 8
#define LEFT_ECHO_PIN 7
#define RIGHT_TRIGGER_PIN 6
#define RIGHT_ECHO_PIN 5
#define NUM_US_SAMPLE 3
#define CM_WALL_TO_RAMP 34
#define CM_POLE_TO_WALL 19
#define DETECT_TOLERANCE 3

int leftDis_old, frontDis_old, rightDis_old;
int leftDis, frontDis, rightDis;
NewPing frontUS(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing leftUS(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing rightUS(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);
  
  //detecting twice to fill old values set
  getUSDis();
  getUSDis();
}

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
    Serial.println("pole on left");
    Serial.println(leftDiff);
    Serial.println(-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE));
  }
  
  if((rightDis - rightDis_old) <= (-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE)) && rightDis != 0){
    Serial.println("pole on right");
    Serial.println((rightDis - rightDis_old));
    Serial.println(-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE));
  }
  
  delay(500);
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

