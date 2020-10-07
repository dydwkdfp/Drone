#define REMOTEXY_MODE__SOFTSERIAL
#include <Servo.h>
#include "I2Cdev.h"
#include <SoftwareSerial.h>
#include <RemoteXY.h>
 
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 
MPU6050 mpu;
 

#define OUTPUT_READABLE_YAWPITCHROLL
 
 #define REMOTEXY_SERIAL_RX 4
#define REMOTEXY_SERIAL_TX 5
#define REMOTEXY_SERIAL_SPEED 9600
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Servo m1;
Servo m2;
Servo m3;
Servo m4;
 
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
 
 
 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
 
 #pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,5,0,0,0,38,0,10,13,0,
  2,0,38,14,22,11,2,26,31,31,
  79,78,0,79,70,70,0,5,32,5,
  24,30,30,2,26,31,5,32,63,24,
  30,30,2,26,31 };

struct {
 uint8_t switch_1; // =1 if switch ON and =0 if OFF 
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  int8_t joystick_2_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_2_y; // =-100..100 y-coordinate joystick position 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
 
void setup() {
    RemoteXY_Init (); 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    m1.attach(3);
    m2.attach(9);
    m3.attach(10);
    m4.attach(11);
    delay(1);
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
 
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
 
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
    // wait for ready 여기가 테스트 끝나면 삭제 할 부분 ( 통신 모듈 여기다 넣을 예정 )
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
 
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
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
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
 
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
 
 
 
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
 
void loop() {
  RemoteXY_Handler ();
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
 
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
 
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
 
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
 
            int val1=0;
            int val2=0;
            int val3=0;
            int val4=0;

            //컨트롤러 1의 잡신호 무시

            int8_t ypr_x_abs=abs(ypr[1] * 180/M_PI);
            int8_t ypr_y_abs=abs(ypr[2] * 180/M_PI);
            
            if(abs(RemoteXY.joystick_1_x)<50){
              RemoteXY.joystick_1_x=0;
            }
            if(abs(RemoteXY.joystick_1_y)<50){
              RemoteXY.joystick_1_y=0;
            }
            if(abs(RemoteXY.joystick_2_x)<50){
              RemoteXY.joystick_2_x=0;
            }
            if(abs(RemoteXY.joystick_2_y)<50){
              RemoteXY.joystick_2_y=0;
            }

            //자이로센서와 컨트롤러에 따른 값 변화

            if(RemoteXY.connect_flag==0){
                  Serial.print("Not Connected");
            }else{
                if(RemoteXY.switch_1 == 0){
                  Serial.print("Drone not on");                  
                }else{
                     val1=1500;
                     val2=1500;
                     val3=1500;
                     val4=1500;
                  
                     if(ypr_x_abs>10||ypr_y_abs>10){
                        if((ypr[1]*180/M_PI)>10){
                           if(abs(ypr[2]*180/M_PI)>10){
                              val1=val1+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val2=val2+abs(ypr[2]*180/M_PI)*5;
                              val3=val3+abs(ypr[1]*180/M_PI)*5;
                           }else if(abs(ypr[2]*180/M_PI)<-10){
                              val1=val1+abs(ypr[2]*180/M_PI)*5;
                              val2=val2+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val4=val4+abs(ypr[1]*180/M_PI)*5;                    
                           }else{
                              val4=val4+abs(ypr[1]*180/M_PI)*5;
                              val2=val2+abs(ypr[1]*180/M_PI)*5;
                           }
                       }else if((ypr[1]*180/M_PI)<-10){
                           if(abs(ypr[2]*180/M_PI)>10){
                              val1=val1+abs(ypr[1]*180/M_PI)*5;
                              val3=val3+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val4=val4+abs(ypr[2]*180/M_PI)*5;                    
                           }else if(abs(ypr[2]*180/M_PI)<-10){
                              val2=val2+abs(ypr[1]*180/M_PI)*5;
                              val4=val4+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val3=val3+abs(ypr[2]*180/M_PI)*5;                  
                           }else{
                              val3=val3+abs(ypr[1]*180/M_PI)*5;
                              val1=val1+abs(ypr[1]*180/M_PI)*5;
                           }
                       }else{
                           if(abs(ypr[2]*180/M_PI)>10){
                              val1=val2+abs(ypr[2]*180/M_PI)*5;
                              val2=val4+abs(ypr[2]*180/M_PI)*5;                    
                           }else if(abs(ypr[2]*180/M_PI)<-10){
                              val3=val1+abs(ypr[2]*180/M_PI)*5;
                              val4=val3+abs(ypr[2]*180/M_PI)*5;                  
                           }
                       }
                    }else{
                      if(RemoteXY.joystick_1_x==0&&RemoteXY.joystick_1_y==0&&RemoteXY.joystick_2_x==0&&RemoteXY.joystick_2_y==0){//입력이 없을경우 수평유지
                        if(ypr_x_abs>5||ypr_y_abs>5){
                        if((ypr[1]*180/M_PI)>5){
                           if(abs(ypr[2]*180/M_PI)>5){
                              val1=val1+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val2=val2+abs(ypr[2]*180/M_PI)*5;
                              val3=val3+abs(ypr[1]*180/M_PI)*5;
                           }else if(abs(ypr[2]*180/M_PI)<-5){
                              val1=val1+abs(ypr[2]*180/M_PI)*5;
                              val2=val2+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val4=val4+abs(ypr[1]*180/M_PI)*5;                    
                           }else{
                              val4=val4+abs(ypr[1]*180/M_PI)*5;
                              val2=val2+abs(ypr[1]*180/M_PI)*5;
                           }
                       }else if((ypr[1]*180/M_PI)<-5){
                           if(abs(ypr[2]*180/M_PI)>5){
                              val1=val1+abs(ypr[1]*180/M_PI)*5;
                              val3=val3+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val4=val4+abs(ypr[2]*180/M_PI)*5;                    
                           }else if(abs(ypr[2]*180/M_PI)<-5){
                              val2=val2+abs(ypr[1]*180/M_PI)*5;
                              val4=val4+abs(ypr[1]*180/M_PI)*5+abs(ypr[2]*180/M_PI)*5;
                              val3=val3+abs(ypr[2]*180/M_PI)*5;                  
                           }else{
                              val3=val3+abs(ypr[1]*180/M_PI)*5;
                              val1=val1+abs(ypr[1]*180/M_PI)*5;
                           }
                       }else{
                           if(abs(ypr[2]*180/M_PI)>5){
                              val1=val2+abs(ypr[2]*180/M_PI)*5;
                              val2=val4+abs(ypr[2]*180/M_PI)*5;                    
                           }else if(abs(ypr[2]*180/M_PI)<-5){
                              val3=val1+abs(ypr[2]*180/M_PI)*5;
                              val4=val3+abs(ypr[2]*180/M_PI)*5;                  
                           }
                       }
                    }
                            
                       }else if(RemoteXY.joystick_1_y>50){
                         
                             if(RemoteXY.joystick_1_x>50){
                                val1=1500;
                                val2=1600;
                                val3=1500;
                                val4=1600;
                              }else if(RemoteXY.joystick_1_x<-50){
                                val1=1600;
                                val2=1500;
                                val3=1600;
                                val4=1500;
                              }else if(RemoteXY.joystick_1_x==0){
                                val1=1600;
                                val2=1600;
                                val3=1600;
                                val4=1600;
                              }
                       }else if(RemoteXY.joystick_1_y<-50){
                        
                             if(RemoteXY.joystick_1_x>50){
                                val1=1400;
                                val2=1500;
                                val3=1400;
                                val4=1500;
                              }else if(RemoteXY.joystick_1_x<-50){
                                val1=1500;
                                val2=1400;
                                val3=1500;
                                val4=1400;
                              }else if(RemoteXY.joystick_1_x==0){
                                val1=1400;
                                val2=1400;
                                val3=1400;
                                val4=1400;
                              }
                       }else if(RemoteXY.joystick_2_x>50){
                              val1=val1+50;
                              val2=val2-50;
                              val3=val3+50;
                              val4=val4-50;
                       }else if(RemoteXY.joystick_2_x<-50){
                              val1=val1-50;
                              val2=val2+50;
                              val3=val3-50;
                              val4=val4+50;
                       }else if(RemoteXY.joystick_2_y<50){
                              val1=val1+50;
                              val2=val2+50;
                              val3=val3-50;
                              val4=val4-50;
                       }else if(RemoteXY.joystick_2_y<-50){
                              val1=val1-50;
                              val2=val2-50;
                              val3=val3+50;
                              val4=val4+50;
                       }

                }
                
                    if(val1>1700){
                        val1=1700;
                    }
                    if(val2>1700){
                        val2=1700;
                    }
                    if(val3>1700){
                        val3=1700;
                    }
                    if(val4>1700){
                        val4=1700;
                    }
            }
            }
            

            
            Serial.print("\t");
            Serial.print("val1 : ");
            Serial.print(val1);
            Serial.print("\t");
            Serial.print("val2 : ");
            Serial.print(val2);
            Serial.print("\t");
            Serial.print("val3 : ");
            Serial.print(val3);
            Serial.print("\t");
            Serial.print("val4 : ");
            Serial.println(val4);
 
            m1.writeMicroseconds(val1);
            m2.writeMicroseconds(val2);
            m3.writeMicroseconds(val3);
            m4.writeMicroseconds(val4);
 
            
        #endif
 
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
