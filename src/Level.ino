#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Wire.h>
#include<Adafruit_PWMServoDriver.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

//Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define USMIN  500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2150 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint32_t timestamp;

/*
float k1=.5;
float k2=55;
float k3=.00001;
*/
int milliOld;
int milliNew;
int dt;
 
float pitchTarget=0;

float pitchError=0;
float pitchErrorOld;
float pitchErrorChange;
float pitchErrorSlope=0;
float pitchErrorArea=0;
float pitchServoVal=90;

#define ELEVATOR 0


void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  milliNew=millis();
}

void loop() {
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    //Serial.print("Printing");
    //Serial.println(counter);
    return;
  }
  // reset the counter
  counter = 0;
  /*
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  //Serial.print(pitch);

  milliOld=milliNew;
  milliNew=millis();
  dt=milliNew-milliOld;

  pitchErrorOld=pitchError;
  pitchError=pitchTarget-pitch;
  pitchErrorChange=pitchError-pitchErrorOld;
  pitchErrorSlope=pitchErrorChange/dt;
  pitchErrorArea=pitchErrorArea+pitchError*dt;

  pitchServoVal=pitchServoVal+pitchError/5;
  
  //pitchServoVal=pitchServoVal+k1*pitchError+k2*pitchErrorSlope+k3*pitchErrorArea;
  float pulselength = map(pitchServoVal, 0, 180, USMIN, USMAX);
  if (pulselength > USMAX){
    pulselength = USMAX;
  }
  else if (pulselength < USMIN){
    pulselength = USMIN;
  }

  pwm.writeMicroseconds(ELEVATOR, pulselength);
  
  //pulselength = (pulselength > USMAX)? USMAX : pulselength;
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(pitchServoVal);
  Serial.print(", ");
  Serial.println(pulselength);
*/
}
