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

  if (pitch < -60.0f) {
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
