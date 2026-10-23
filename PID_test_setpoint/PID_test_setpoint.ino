#include "I2Cdev.h"
#include <Wire.h>
#include <PID_v 1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
MPU6050 mpu;

// Define pins
const int IN1 = D0;
const int IN2 = D1;
const int IN3 = D2;
const int IN4 = D3;
const int MPU_INT_PIN = D6;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/************ EASY SETTINGS: CHANGE THESE VALUES FIRST ************/
// PID target angle when the robot is upright and balancing in place.
const double UPRIGHT_SETPOINT = 178.0;

// Movement command (degrees):
//   0.0  = balance in place
//  +0.5  = try a very gentle forward command
//  -0.5  = try a very gentle reverse command
// Start at 0.0. Test tiny changes (e.g. +/- 0.2 to 0.5 degrees) on a clear floor.
// If +0.5 makes the robot go backward, use -0.5 instead.
const double MOVEMENT_OFFSET_DEG = 0.0;

// The PID balances against this target angle.
double setpoint = UPRIGHT_SETPOINT + MOVEMENT_OFFSET_DEG;

// Balance PID gains (keep the values that already work for your robot).
double Kp = 21;
double Kd = 0.8;
double Ki = 140;
/**************** END EASY SETTINGS *******************************/
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // Give USB serial time to connect.

    // Configure motors before initializing the sensor.
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    Stop();

    pinMode(MPU_INT_PIN, INPUT);

    // Match your schematic: SDA = D4, SCL = D5.
    Wire.begin(D4, D5);
    Wire.setClock(100000);
    Wire.setTimeOut(100);

    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    bool connected = mpu.testConnection();
    Serial.println(connected
        ? "MPU6050 connection successful"
        : "MPU6050 connection failed");

    if (!connected) {
        // Repeat the error so it remains visible.
        while (true) {
            Serial.println("Check MPU6050 power, SDA, SCL and address");
            delay(1000);
        }
    }

    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-479);
    mpu.setYGyroOffset(84);
    mpu.setZGyroOffset(15);
    mpu.setZAccelOffset(1638); 
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        //setup PID
        Serial.print("PID target setpoint: ");
        Serial.println(setpoint);
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

// Initially turn off both motors.
analogWrite(IN1, 0);
analogWrite(IN2, 0);
analogWrite(IN3, 0);
analogWrite(IN4, 0);

// Set the Interrupt pin
pinMode(MPU_INT_PIN, INPUT);

}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) {
    Serial.print("DMP not ready. Error code: ");
    Serial.println(devStatus);
    delay(1000);
    return;
}
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();
        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
        if (input>150 && input<200){//If the Bot is falling 
        if (output>0) //Falling towards front 
        Forward(); //Rotate the wheels forward 
        else if (output<0) //Falling towards back
        Reverse(); //Rotate the wheels backward 
        }
        else //If Bot not falling
        Stop(); //Hold the wheels still
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
        input = ypr[1] * 180/M_PI + 180;
   }
}

void Forward()
{
    analogWrite(IN1, (int)output);
    analogWrite(IN2, 0);
    analogWrite(IN3, (int)output);
    analogWrite(IN4, 0);
    Serial.print("F");
}

void Reverse()
{
    analogWrite(IN1, 0);
    analogWrite(IN2, (int)(-output));
    analogWrite(IN3, 0);
    analogWrite(IN4, (int)(-output));
    Serial.print("R");
}

void Stop()
{
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
    Serial.print("S");
}