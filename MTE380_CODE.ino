#include <Servo.h>
#include <Wire.h>

/*
 * IMU BOILERPLATE
 */
#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

#define MPU6050_ADDRESS 0x68  // Device address when ADO = 0

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// Specify sensor full scale
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;
float aRes, gRes; // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 19;  // These can be changed, 2 and 3 are the Arduinos ext int pins
#define blinkPin 13  // Blink LED on Teensy or Pro Mini when updating
boolean blinkOn = false;

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate

// parameters for 6 DoF sensor fusion calculations
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float pitch, yaw, roll;
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;                                 // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion




/*
 * Servo State Variables
 */
Servo lMotor;
Servo rMotor;
int lMotorPWMPin = 10;
int rMotorPWMPin = 9;
int lMotorDeadPos = 92; // 180 reverse max
int rMotorDeadPos = 92; // 180 forward max
int currentLMotorSpeed = lMotorDeadPos;
int currentRMotorSpeed = rMotorDeadPos;
int targetLMotorSpeed;
int targetRMotorSpeed;

bool trackYaw = false;
float targetYaw;
float targetYawMargin = 1.00f;


void setup()
{
  Wire.begin();
  Serial.begin(38400);

  // Initialize Motors
  lMotor.attach(lMotorPWMPin);
  rMotor.attach(rMotorPWMPin);
  stopBothMotors();
  setRightMotorSpeed(10.0);
  setLeftMotorSpeed(10.0);

  trackYaw = true;
  targetYaw = 0.0f;
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, HIGH);
  

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU6050 is online...");
    
    MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
  
    calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers   
    
   initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
   }
   else
   {
    Serial.print("Could not connect to MPU6050: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
   }
  }
}


void loop()
{  
   // If data ready bit set, all data registers have new data
  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;   
    az = (float)accelCount[2]*aRes;  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   

    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
   }  
   
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    digitalWrite(blinkPin, blinkOn);

    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;

//    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    
    blinkOn = ~blinkOn;
    count = millis(); 

    updateMotorState();
  }


/*
 * Servo Helper Functions
 */


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
    Serial.println(rotationAngle,2);
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

  delay(30);
    
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


