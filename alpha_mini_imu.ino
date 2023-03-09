#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2

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

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// Define pins for motor control
// right wheel
const int motor1Pin1 = A0;
const int motor1Pin2 = A1;
// left wheel
const int motor2Pin1 = A2;
const int motor2Pin2 = A3;

const int motor1SpeedPin = 7; // left speed pin
const int motor2SpeedPin = 6; // right wheel pin

// Define PID constants
// const float Kp = 2.0;
// const float Ki = 0.5;
// const float Kd = 1.0;

const float Kp = 1.5;
const float Ki = 0.2;
const float Kd = 1.5;

// Define IMU object and variables for storing IMU readings
float imuAngle = 0.0;
float imuAnglePrev = 0.0;
float imuAngularVelocity = 0.0;

// Define PID variables
float target = 0.0;
float error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float pidOutput = 0.0;

// Define variables for controlling motors
// float motor1Speed = 0.0;
// float motor2Speed = 0.0;
float motorSpeed = 127.5;
float speed1;
float speed2;

int imu_calibration_count = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif  

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  
  // Initialize motor control pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(motor1SpeedPin, OUTPUT);
  pinMode(motor2SpeedPin, OUTPUT);
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // Serial.print("ypr\t");
      // Serial.print(ypr[0] * 180/M_PI);
      // Serial.print("\t");
      // Serial.print(ypr[1] * 180/M_PI);
      // Serial.print("\t");
      // Serial.println(ypr[2] * 180/M_PI);

      if(imu_calibration_count <= 300){
        target = ypr[0];
        imu_calibration_count++;
      }
      else {
        // Read IMU data
        imuAnglePrev = imuAngle;
        imuAngle = ypr[0];
        imuAngularVelocity = imuAngle - imuAnglePrev;

        // Calculate PID output
        error = target - imuAngle;
        integral += error;
        derivative = imuAngularVelocity;
        pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Update motor speeds based on PID output
        // motor1Speed = pidOutput;
        // motor2Speed = pidOutput;

        speed1 = motorSpeed + abs(pidOutput);
        speed2 = motorSpeed - abs(pidOutput);

        if (imuAngle > 0) {
          digitalWrite(motor1Pin1, LOW);
          digitalWrite(motor1Pin2, HIGH);
          analogWrite(motor1SpeedPin, speed1);
          // analogWrite(motor1SpeedPin, speed1 > 1023 ? 1023 : speed1);

          digitalWrite(motor2Pin1, LOW);
          digitalWrite(motor2Pin2, HIGH);
          analogWrite(motor2SpeedPin, speed2);
          // analogWrite(motor2SpeedPin, speed2 < 50 ? 50 : speed2);
        }
        else if (imuAngle < 0) {
          digitalWrite(motor1Pin1, LOW);
          digitalWrite(motor1Pin2, HIGH);
          analogWrite(motor1SpeedPin, speed2);

          digitalWrite(motor2Pin1, LOW);
          digitalWrite(motor2Pin2, HIGH);
          analogWrite(motor2SpeedPin, speed1);
        }
        else {
          digitalWrite(motor1Pin1, LOW);
          digitalWrite(motor1Pin2, HIGH);
          analogWrite(motor1SpeedPin, speed1);

          digitalWrite(motor2Pin1, LOW);
          digitalWrite(motor2Pin2, HIGH);
          analogWrite(motor2SpeedPin, speed1);
        }

        // Print debug information
        Serial.print("IMU Angle: ");
        Serial.print(imuAngle);
        Serial.print("  PID Output: ");
        Serial.print(pidOutput);
        Serial.print("  Motor Speed 1: ");
        Serial.print(speed1);
        Serial.print("  Motor Speed 2: ");
        Serial.println(speed2);

        // Wait for a short time before updating again
        // delay(10);
      }
    #endif
  }
}