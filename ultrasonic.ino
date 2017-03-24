unsigned int getFrontUSDist()
{
  frontDis_old = frontDis;
  frontDis = frontUS.convert_cm(frontUS.ping_median(US_NUM_SAMPLE));
  return frontDis;
}

unsigned int getLeftUSDist()
{
  leftDis_old = leftDis;
  leftDis = leftUS.convert_cm(leftUS.ping_median(US_NUM_SAMPLE));
  return leftDis;
}

unsigned int getRightUSDist()
{
  rightDis_old = rightDis;
  rightDis = rightUS.convert_cm(rightUS.ping_median(US_NUM_SAMPLE));
  return rightDis;
}

void getUSDis(){
  // getFrontUSDist();
  getLeftUSDist();
  getRightUSDist();
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


