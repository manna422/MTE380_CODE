void driveToWallState() {
  setLeftMotorSpeed(L_FWD_MAX);
  setRightMotorSpeed(R_FWD_MAX);

  if (pitch > 60.0f) {
    GLOBAL_STATE = ST_UP_WALL;
  }
}

void upWallState() {
  setLeftMotorSpeed(L_FWD_MAX);
  setRightMotorSpeed(R_FWD_MAX);

  if (pitch < 30.0f) {
    GLOBAL_STATE = ST_TOP_WALL;
  }
}

void topWallState() {
  setLeftMotorSpeed(L_FWD_SLOW);
  setRightMotorSpeed(R_FWD_SLOW);

  if (pitch < -45.0f) {
    GLOBAL_STATE = ST_DOWN_WALL_1;
    STATE_START_TIME = millis();
  }
}

void downWall1State() {
  setLeftMotorSpeed(L_FWD_SLOW);
  setRightMotorSpeed(R_FWD_SLOW);

  if (millis() - STATE_START_TIME > 2000) {
    GLOBAL_STATE = ST_DOWN_WALL_2;
  }
}

void downWall2State() {
  setLeftMotorSpeed(L_FWD_MAX);
  setRightMotorSpeed(R_FWD_MAX);

  if (pitch > -10.0f) {
    GLOBAL_STATE = ST_POLE_DETECT;
  }
}

void poleDection(){
  setLeftMotorSpeed(L_FWD_50);
  setRightMotorSpeed(R_FWD_50);
  getUSDis();
  printAllUSDiff();
  printAllUSValues();
  
  int leftDiff = leftDis - leftDis_old;
  int rightDiff = rightDis - rightDis_old;
  
  if(rightDiff > -1*(CM_WALL_TO_RAMP+DETECT_TOLERANCE) && rightDiff < -1*(CM_WALL_TO_RAMP-DETECT_TOLERANCE)&& rightDis!= 0){
    Serial.println("ramp on left");
  }
  
  if(leftDiff + CM_POLE_TO_WALL < DETECT_TOLERANCE && leftDis!= 0 ){
    //track 90 - 15/2 = 82.5deg
    targetYaw = yaw + 90.0f;
    GLOBAL_STATE = ST_TURN_TOWARD_POLE;
    setLeftMotorSpeed(L_REV_100);
    setRightMotorSpeed(R_REV_100);
    getUSDis();
    Serial.println("pole on left");
    Serial.println(leftDiff);
    Serial.println(-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE));
  }
  
  if(rightDiff <= (-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE)) && rightDis != 0){
    //track -(90 - 15/2) = -82.5deg
    targetYaw = yaw - 90.0f;
    GLOBAL_STATE = ST_TURN_TOWARD_POLE;
    setLeftMotorSpeed(L_REV_100);
    setRightMotorSpeed(R_REV_100);
    getUSDis();
    Serial.println("pole on right");
    Serial.println(rightDiff);
    Serial.println(-1*(CM_POLE_TO_WALL-DETECT_TOLERANCE));
  }

  if(targetYaw > 180)
    targetYaw -= 360.0f; 
  if(targetYaw < -180)
    targetYaw += 360.0f; 
     
}

void turnTowardPole(){
  float diff = yawDiff();
  if(abs(diff)< 2.0f){
    setLeftMotorSpeed(L_STOP);
    setRightMotorSpeed(R_STOP);
    GLOBAL_STATE = ST_DRIVE_TO_POLE;
  }
  else if(targetYaw > yaw){
    Serial.print("    Turning left");
    setLeftMotorSpeed(L_REV_100);
    setRightMotorSpeed(R_FWD_MAX);
    if(abs(diff)<40){
      setLeftMotorSpeed(L_STOP);
      setRightMotorSpeed(R_FWD_SLOW);  
    }
  }
  else{
    Serial.print("    Turning Right");
    setLeftMotorSpeed(L_FWD_MAX);
    setRightMotorSpeed(R_REV_100);
    if(abs(diff)<40){
      setLeftMotorSpeed(L_FWD_SLOW);
      setRightMotorSpeed(R_STOP);  
    }
  }
  Serial.println();
}

void driveToPole(){
  float diff = yawDiff();
  if(abs(diff)< 2.0f){
//    setLeftMotorSpeed(L_STOP);
//    setRightMotorSpeed(R_STOP);
    setLeftMotorSpeed(L_FWD_MAX);
    setRightMotorSpeed(R_FWD_MAX);
  }
  else{
    GLOBAL_STATE = ST_TURN_TOWARD_POLE;
  }
}

float yawDiff(){
  float diff = targetYaw - yaw;
  Serial.print("targetYaw = ");
  Serial.print(targetYaw);
  Serial.print("  yaw = ");
  Serial.print(yaw);
  Serial.print("  difference in yaw = ");
  Serial.println(diff);
  return diff;
}

