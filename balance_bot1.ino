//#define Sprint(a) (Serial.print(a))
//#define Sprintln(a) (Serial.println(a))
#define Sprint(a) 
#define Sprintln(a) 


#define kMotorActivationMinSpeed 60
#define kXGyroOffset 0
#define kYGyroOffset 0
#define kZGyroOffset 0
#define kZAccelOffset 0



/*
 * 
 * MPU defs
 * 
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




/*
 * 
 * MPU FUNCTIONS
 * 
 */

void mpuSetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Sprintln(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Sprintln(F("Testing device connections..."));
    Sprintln(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Sprintln(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(kXGyroOffset);
    mpu.setYGyroOffset(kYGyroOffset);
    mpu.setZGyroOffset(kZGyroOffset);
    mpu.setZAccelOffset(kZAccelOffset); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Sprintln(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Sprintln(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Sprintln(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Sprint(F("DMP Initialization failed (code "));
        Sprint(devStatus);
        Sprintln(F(")"));
    }

}



void updateYawPitchRoll( double *yaw, double *pitch, double *roll )  {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) { }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Sprintln(F("FIFO overflow!"));
        
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        *yaw = (ypr[0] * 180/M_PI);
        *pitch = (ypr[1] * 180/M_PI);
        *roll = (ypr[2] * 180/M_PI);
    }
}










/*
 * 
 * Motor Drive def's
 * 
 */
#define kForward 1
#define kReverse -1
#define kStop 0

#define kLeftSideSpeed 5
#define kLeftSideForward 6
#define kLeftSideReverse 7
#define kRightSideSpeed 11
#define kRightSideForward 10
#define kRightSideReverse 9


/*
 * 
 * MOTOR DRIVE FUNCTIONS
 * 
 */
void motorDriveSetup() {
  pinMode(kLeftSideSpeed, OUTPUT);
  pinMode(kLeftSideForward, OUTPUT);
  pinMode(kLeftSideReverse, OUTPUT);
  pinMode(kRightSideSpeed, OUTPUT);
  pinMode(kRightSideForward, OUTPUT);
  pinMode(kRightSideReverse, OUTPUT);
}



void directionWithAngle(int angle) {
  int constrainedAngle = constrain(angle, -90, 90);
  
  int speed = map(abs(constrainedAngle), 0, 90, kMotorActivationMinSpeed, 255);
  int direction = (angle > 0)? kForward: kReverse;
  
  genericDrive( kLeftSideSpeed, kLeftSideForward, kLeftSideReverse, direction, speed);
  genericDrive( kRightSideSpeed, kRightSideForward, kRightSideReverse, direction, speed);
}



void genericDrive(int pinPWM, int pinForward, int pinReverse, int direction, int PWM) {
  
  if (direction == kForward) 
  {
    digitalWrite(pinForward, HIGH);
    digitalWrite(pinReverse, LOW);
    analogWrite(pinPWM, PWM);
  }
  else if (direction == kReverse) 
  {
    digitalWrite(pinForward, LOW);
    digitalWrite(pinReverse, HIGH);
    analogWrite(pinPWM, PWM);
  }
  else 
  {
    digitalWrite(pinForward, LOW);
    digitalWrite(pinReverse, LOW);
    analogWrite(pinPWM, 0);
  }  
}







void setup() {
  // basic setup
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  motorDriveSetup();
  mpuSetup();
}


void loop() {
  double yaw, pitch, roll;
  updateYawPitchRoll( &yaw, &pitch, &roll );
  
  double tiltAngle = tiltAngleFromYawPitchRoll( yaw, pitch, roll );
  directionWithAngle(tiltAngle);
}



int tiltAngleFromYawPitchRoll( double yaw, double pitch, double roll ) {
//  Serial.print("You wot mate?   ");
//  Serial.print(atan2(pitch, roll)* 57296 / 1000);
//  Serial.print(" ");
//  Serial.print(atan2(pitch, yaw)* 57296 / 1000);
//  Serial.print(" ");
//  Serial.print(atan2(roll, yaw)* 57296 / 1000);
//  Serial.println(" ");
  double rad = atan2(pitch, yaw);
  double deg = rad * 57296 / 1000;
  Sprint("Degrees??? ");
  Sprintln(deg);
  return deg; // is this all there is to it?
}

