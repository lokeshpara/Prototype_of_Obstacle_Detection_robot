#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define trigpin 6 // Ultersonic sensor 
#define echopin 7
int distance;
#define revleft 11 // motor driver
#define fwdleft 10
#define fwdright 12
#define revright 13
#define enable 9
// buffer
#define buzzer 9
int value;
int point_A = 1;
int point_B = 0;
int point_C = 0;   
bool reached_destination = false;
// angles
int Point_A_angle = 0;
int Point_B_angle = 90;
int Point_B_to_Start_angle = -135;
// delays 
int point_A_delay = 400;
int point_A_to_B_delay = 200;
int point_b_to_start_delay = 200;

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
// ===                      INITIAL SETUP                       ===
// ================================================================
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
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
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
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  pinMode(enable,OUTPUT);  // Set Motor Pins As O/P and speed control enable
  pinMode(revleft, OUTPUT); 
  pinMode(fwdleft, OUTPUT);
  pinMode(revright, OUTPUT);
  pinMode(fwdright, OUTPUT);
  pinMode(trigpin, OUTPUT); // Set Trig Pin As O/P To Transmit Waves
  pinMode(echopin, INPUT);
  pinMode(buzzer, OUTPUT); //buzzer
}
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
     //if success, don't try to do anything 
     if(obstacle()) {
        stop_Robot();
        buzzer_sound();
        delay(5000);
     }else if(!obstacle()) {
        if(reached_destination) {
          stop_Robot();
        }else if(point_A) {
            move_towards_destination(Point_A_angle,0,point_A_delay,0);   // parameters target angle, to move another destination true or false, allocate time takes in straight line movement
            delay(3000);
        }else if(point_B) {
            move_towards_destination(Point_B_angle,1,point_A_to_B_delay,0);
            delay(3000);
        }else if(point_C) {
            move_towards_destination(Point_B_to_Start_angle,2,point_b_to_start_delay,0);
        }
     }

}

//GryoScope
int present_robot_angle() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // Get the Latest packet      
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      value = ypr[0]*180/M_PI;
    }  
    return value;
}

//Robot movement to destination points
void move_towards_destination(int angle_of_destination,int next_point,int time_for_forward,int done) {
//   with maintaining the angle of velhicle need to move to the destination
  int i = 0;
  while(!done) {
    if(i<time_for_forward) {
      while(i<time_for_forward) {
        if(obstacle()) {
          stop_Robot();
          break; 
        }
        if(!(present_robot_angle()<=(angle_of_destination+2) && present_robot_angle()>=(angle_of_destination-2))) {
          while (!(present_robot_angle()<=(angle_of_destination+2) && present_robot_angle()>=(angle_of_destination-2))) {
          if(obstacle()) {
            stop_Robot();
            break;
          }
            if(present_robot_angle()<angle_of_destination-2) {
              move_right();
            } else if(present_robot_angle()>angle_of_destination+2) {
                move_left();
            } 
          }
        } else if((present_robot_angle()<=(angle_of_destination+2) && present_robot_angle()>=(angle_of_destination-2))) {
              while(i<time_for_forward) {
                  if(((!(present_robot_angle()<=(angle_of_destination+2) && present_robot_angle()>=(angle_of_destination-2) )) or (obstacle()) ) ) {
                    break;
                  }
                  move_forward();
                  i=i+1;
              };        
          }
      }
    }else {
      stop_Robot();
      if(!next_point) {
          point_A = 0;
          point_B = 1;
          point_C = 0;
          stop_Robot();
          reached_destination= false;
          stop_Robot();
          done = 1;
      }
      else if(next_point == 1){
          point_A = 0;
          point_B = 0;
          point_C = 1;
          stop_Robot();
          reached_destination = false;
          stop_Robot();
          done = 1;
      }else if(next_point == 2) {
          point_A = 0;
          point_B = 0;
          point_C = 0;
          stop_Robot();
          reached_destination = true;
          done = 1;
      }
    }
  }
}


int obstacle(){
  //write ultersonic-sensor code 
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);
  float duration = pulseIn(echopin, HIGH);
  distance= duration*0.034/2;
  if(distance<15) {
    return 1;
  }return 0;
}

void buzzer_sound() {
  tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(500);        // ...for 1sec
}

void move_forward(){
    digitalWrite(fwdright, HIGH);                 
    digitalWrite(revright, LOW);
    digitalWrite(fwdleft, HIGH);                                
    digitalWrite(revleft, LOW);
    analogWrite(enable,170);
}

void stop_Robot(){
    digitalWrite(fwdright, LOW);                 
    digitalWrite(revright, LOW);
    digitalWrite(fwdleft, LOW);                                
    digitalWrite(revleft, LOW);
}

void move_left(){
    digitalWrite(fwdright, HIGH);                 
    digitalWrite(revright, LOW);
    digitalWrite(fwdleft, LOW);                                
    digitalWrite(revleft, LOW);
    analogWrite(enable,170);
}

void move_right(){
    digitalWrite(fwdright, LOW);                 
    digitalWrite(revright, LOW);
    digitalWrite(fwdleft, HIGH);                                
    digitalWrite(revleft, LOW);
    analogWrite(enable,170);
}
