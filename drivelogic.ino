void positionAlignToWall()
{
    unsigned int dist = sonar.getFrontDist();
    if (dist > 30) {
        motor.setLeftMotorSpeed(50);
        motor.setRightMotorSpeed(50);
    } 
}

void turnLeft(int degrees) 
{

}

void turnRight(int degrees) 
{
    
}