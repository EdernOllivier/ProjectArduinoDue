/*
  This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
  It won't work with v1.x motor shields! Only for the v2's with built in PWM
  control

  For use with the Adafruit Motor Shield v2
  ---->	http://www.adafruit.com/products/1438
*/
//#include "I2CScanner.h"
//#include <Wire.h>
//#include <Adafruit_MotorShield.h>
#define Wire Wire1
//I2CScanner scanner;
// Create the motor shield object with the default I2C address
//Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
//Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
// You can also make another stepper motor on port M3 and M4
//Adafruit_StepperMotor *myNextMotor = AFMS.getStepper(200, 2);

const int
PWM_A   = 3,
DIR_A   = 12,
BRAKE_A = 9,
SNS_A   = A0;

const int
PWM_B   = 11,
DIR_B   = 13,
BRAKE_B = 8,
SNS_B   = A1;

int sensorPin1 = A2;    // select the input pin for the gyrometer // gyrometer wiper (middle terminal) connected to analog pin 1
// outside leads to ground and +3.3V
int sensorValue1 = 0;   // variable to store the value coming from the sensor
/*
  int sensorPin2 = A3;    // select the input pin for the inclinometer
  int sensorValue2 = 0;  // variable to store the value coming from the sensor
*/
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect VDD to 3-5V DC (depending on your board's logic level)
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

//#include <QuadratureEncoder.h>
// must also have enableInterrupt.h library
//#include <avr/interrupt.h>

// Use any 2 pins for interrupt, this utilizes EnableInterrupt Library.
// Even analog pins can be used. A0 = 14,A1=15,..etc for arduino nano/uno

// Max number of Encoders object you can create is 4. This example only uses 2.

// digital pins 2, 4 have an encoder attached to it. Give it a name:
//int EncoderA = 2;
//int EncoderB = 4;

//Encoders leftEncoder(EncoderA, EncoderB);  // Create an Encoder object name leftEncoder, using digitalpin 2 & 4
//Encoders rightEncoder(15,14); // Encoder object name rightEncoder using analog pin A0 and A1

#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_LSM9DS1.h>
//#include <Adafruit_Sensor.h>  // not used in this demo but required!
/*
  // i2c
  Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

  #define LSM9DS1_SCK A5
  #define LSM9DS1_MISO 12
  #define LSM9DS1_MOSI A4
  #define LSM9DS1_XGCS 6
  #define LSM9DS1_MCS 5
  // You can also use software SPI
  //Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
  // Or hardware SPI! In this case, only CS pins are passed in
  //Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
*/
/*
  void setupSensor()
  {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  }
*/
void setup() {
  // Configure the A output
  pinMode(BRAKE_A, OUTPUT);  // Brake pin on channel A
  pinMode(DIR_A, OUTPUT);    // Direction pin on channel A

  // Configure the B output
  pinMode(BRAKE_B, OUTPUT);  // Brake pin on channel B
  pinMode(DIR_B, OUTPUT);    // Direction pin on channel B

  // Open Serial communication
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor shield DC motor Test:\n");
/*
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myOtherMotor->setSpeed(150);
  myOtherMotor->run(FORWARD);
  // turn on motor
  myOtherMotor->run(RELEASE);

  Serial.println("Stepper test!");

  //  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  myNextMotor->setSpeed(10);  // 10 rpm

  //  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
*/
  //  scanner.Init();
  /* Initialise the sensor */

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();
//  displayCalStatus();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
  /*
    //////////////////////////////////////////////
    // make the pushbutton's pin an input:
    pinMode(EncoderA, INPUT);
    pinMode(EncoderB, INPUT);

    while (!Serial) {
      delay(1); // will pause Zero, Leonardo, etc until serial console opens
    }

    Serial.println("LSM9DS1 data read demo");

    // Try to initialise and warn if we couldn't detect the chip
    if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");

    // helper to just set the default scaling we want, see above!
    setupSensor();
  */
}

unsigned long lastMilli = 0;

void loop() {
// Set the outputs to run the motor forward

//  digitalWrite(BRAKE_A, LOW);  // setting brake LOW disable motor brake
//  digitalWrite(DIR_A, HIGH);   // setting direction to HIGH the motor will spin forward

//  analogWrite(PWM_A, 255);     // Set the speed of the motor, 255 is the maximum value

//  delay(5000);                 // hold the motor at full speed for 5 seconds
//  Serial.print("current consumption at full speed: ");
//  Serial.println(analogRead(SNS_A));

// Brake the motor

//  Serial.println("Start braking\n");
  // raising the brake pin the motor will stop faster than the stop by inertia
//  digitalWrite(BRAKE_A, HIGH);  // raise the brake
//  delay(5000);

// Set the outputs to run the motor backward

//  Serial.println("Backward");
//  digitalWrite(BRAKE_A, LOW);  // setting againg the brake LOW to disable motor brake
//  digitalWrite(DIR_A, LOW);    // now change the direction to backward setting LOW the DIR_A pin

//  analogWrite(PWM_A, 255);     // Set the speed of the motor

//  delay(5000);
//  Serial.print("current consumption backward: ");
//  Serial.println(analogRead(SNS_A));

  // now stop the motor by inertia, the motor will stop slower than with the brake function
//  analogWrite(PWM_A, 0);       // turn off power to the motor

//  Serial.print("current brake: ");
//  Serial.println(analogRead(A0));
//  Serial.println("End of the motor shield test with DC motors. Thank you!");


//  while(1);  

  // read the value from the gyrometer sensor:
  sensorValue1 = analogRead(sensorPin1); // - 512;
  Serial.println("From the gyrometer : ");
  Serial.print(sensorValue1, DEC);
  Serial.println("");
  //  scanner.Scan();

  // a little delay to not hog Serial Monitor
//  delay(100);

  // change the resolution to 12 bits and read A1
  //  analogReadResolution(12);
  //  Serial.print(", 12-bit : ");
  //  Serial.print(analogRead(A1));/* Get a new sensor event */
  //  Serial.println("");

  // a little delay to not hog Serial Monitor
  //  delay(100);

  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */

  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
  /*
    ///////////////////////////////////////////////////////////////////////////////////
    // put your main code here, to run repeatedly:
    // print encoder count every 50 millisecond
    if(millis()-lastMilli > 50){

      long currentLeftEncoderCount = leftEncoder.getEncoderCount();
    //    long currentRightEncoderCount = rightEncoder.getEncoderCount();

      // read the value from the odometer sensor:
      Serial.println("From the odometer : ");
      Serial.print(currentLeftEncoderCount);
    //    Serial.print(" , ");
    //    Serial.println(currentRightEncoderCount);
      Serial.println("");

      lastMilli = millis();
    }
    //////////////////////////////////////////////////////////////////////////////////
  */

  //uint8_t i;

  // read the value from the gyrometer sensor:
//  sensorValue1 = analogRead(sensorPin1); // - 512;
//  Serial.println("From the gyrometer : ");
//  Serial.print(sensorValue1, DEC);
//  Serial.println("");
  /*
    // read the value from the inclinometer sensor:
    sensorValue2 = analogRead(sensorPin2);
    Serial.println("From the inclinometer : ");
    Serial.print(sensorValue2, DEC);
    Serial.println("");
  */
  //  Serial.print("tick");

  //  uint8_t i;
  /*
    Serial.print("tick");

    myOtherMotor->run(FORWARD);
    for (i=0; i<200; i++) {
      myOtherMotor->setSpeed(i);
      delay(10);
    }
    for (i=200; i!=0; i--) {
      myOtherMotor->setSpeed(i);
      delay(10);
    }

    Serial.print("tock");

    myOtherMotor->run(BACKWARD);
    for (i=0; i<200; i++) {
      myOtherMotor->setSpeed(i);
      delay(10);
    }
    for (i=200; i!=0; i--) {
      myOtherMotor->setSpeed(i);
      delay(10);
    }

    Serial.print("tech");
    myOtherMotor->run(RELEASE);
    delay(1000);

    //////////////////////////////////////////////////////////////////////////////////////////
  */

  //  lsm.read();  /* ask it to read in the data */

  /* Get a new sensor event */
  /*
    sensors_event_t a, m, g, temp;

    lsm.getEvent(&a, &m, &g, &temp);

    Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
    Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
    Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

    Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
    Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
    Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

    Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
    Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
    Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");

    Serial.println();
  */
  //  delay(200);
  ////////////////////////////////////////////////////////////////////////////////////////////

  // Set the speed to start, from 0 (off) to 255 (max speed)
//  myMotor->run(FORWARD);
//  myMotor->setSpeed(191);
  // turn on motor
//  myMotor->run(RELEASE);

  // here the stab. has been done with the inclinometer
//    Serial.print("\n\tY: ");
//    Serial.print(event.orientation.y, 4);
    if (event.orientation.y > 0)
    {
//      myMotor->run(BACKWARD);
        Serial.println("Backward");
        digitalWrite(BRAKE_A, LOW);  // setting againg the brake LOW to disable motor brake
        digitalWrite(DIR_A, LOW);    // now change the direction to backward setting LOW the DIR_A pin
//      myMotor->setSpeed(event.orientation.y *2);
        analogWrite(PWM_A, event.orientation.y *10);
//    myNextMotor->step(4, FORWARD, MICROSTEP);
    }
    else
    {
//      myMotor->run(FORWARD);
        Serial.println("forward");
        digitalWrite(BRAKE_A, LOW);  // setting brake LOW disable motor brake
        digitalWrite(DIR_A, HIGH);   // setting direction to HIGH the motor will spin forward
//      myMotor->setSpeed(-1 *event.orientation.y *2);
        analogWrite(PWM_A, -1 *event.orientation.y *10);
    //    myNextMotor->step(4, BACKWARD, MICROSTEP);
    }

    if (event.orientation.z > 0)
    {
//      myMotor->run(BACKWARD);
        Serial.println("Backward");
        digitalWrite(BRAKE_B, LOW);  // setting againg the brake LOW to disable motor brake
        digitalWrite(DIR_B, LOW);    // now change the direction to backward setting LOW the DIR_A pin
//      myMotor->setSpeed(event.orientation.z *2);
        analogWrite(PWM_B, event.orientation.z *10);
//    myNextMotor->step(4, FORWARD, MICROSTEP);
    }
    else
    {
//      myMotor->run(FORWARD);
        Serial.println("forward");
        digitalWrite(BRAKE_B, LOW);  // setting brake LOW disable motor brake
        digitalWrite(DIR_B, HIGH);   // setting direction to HIGH the motor will spin forward
//      myMotor->setSpeed(-1 *event.orientation.z *2);
        analogWrite(PWM_B, -1 *event.orientation.z *10);
    //    myNextMotor->step(4, BACKWARD, MICROSTEP);
    }
}
    // here the stab. has been done with the inclinometer
//    if (event.orientation.z > 0)
//    {
//      myOtherMotor->run(BACKWARD);
//      myOtherMotor->setSpeed(event.orientation.z *2);
    //    myNextMotor->step(4, FORWARD, MICROSTEP);
//    }
//    else
//    {
//      myOtherMotor->run(FORWARD);
//      myOtherMotor->setSpeed(-1 *event.orientation.z *2);
    //    myNextMotor->step(4, BACKWARD, MICROSTEP);
//    }

    // here the stab. has been done with the gyrometer
//    if (sensorValue1 > 0)
//    {
//      myMotor->run(FORWARD);
//        Serial.println("forward");
//        digitalWrite(BRAKE_A, LOW);  // setting brake LOW disable motor brake
//        digitalWrite(DIR_A, HIGH);   // setting direction to HIGH the motor will spin forward
//      myMotor->setSpeed(sensorValue1);
//        analogWrite(PWM_A, 255); //sensorValue1);
//    }
//    else
//    {
//      myMotor->run(BACKWARD);
//        Serial.println("Backward");
//        digitalWrite(BRAKE_A, LOW);  // setting againg the brake LOW to disable motor brake
//        digitalWrite(DIR_A, LOW);    // now change the direction to backward setting LOW the DIR_A pin
//      myMotor->setSpeed(-1 *sensorValue1);
//        analogWrite(PWM_A, 255); //-1 *sensorValue1);
//    }
//#ifdef TRUE
  /*
    Serial.println("Single coil steps");
    myNextMotor->step(100, FORWARD, SINGLE);
    myNextMotor->step(100, BACKWARD, SINGLE);

    Serial.println("Double coil steps");
    myNextMotor->step(100, FORWARD, DOUBLE);
    myNextMotor->step(100, BACKWARD, DOUBLE);

    Serial.println("Interleave coil steps");
    myNextMotor->step(100, FORWARD, INTERLEAVE);
    myNextMotor->step(100, BACKWARD, INTERLEAVE);

    Serial.println("Microstep steps");
    myNextMotor->step(50, FORWARD, MICROSTEP);
    myNextMotor->step(50, BACKWARD, MICROSTEP);
  */
  /*
    // for the stepper motor to be actionned
    //#else
    //  Serial.print("tick");
    myMotor->run(FORWARD);
    for (i=0; i<50; i++) {
      myMotor->setSpeed(i);
      delay(10);
    }
    for (i=50; i!=0; i--) {
      myMotor->setSpeed(i);
      delay(10);
    }

    //  Serial.print("tock");
    myMotor->run(BACKWARD);
    for (i=0; i<50; i++) {
      myMotor->setSpeed(i);
      delay(10);
    }
    for (i=50; i!=0; i--) {
      myMotor->setSpeed(i);
      delay(10);
    }

    //  Serial.print("tech");
    myMotor->run(RELEASE);
    delay(1000);
    //#endif
  */
