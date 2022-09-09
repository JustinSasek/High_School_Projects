#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <SparkFunLSM6DS3.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>

//Settings
//initial values
float alt0Actual = 100;  //m  //launch site book altitude
float alt0;  //m  //initial calculated altitude
float presSea;  //initial calculated pressure at sealevel
double accelX0;
double accelY0;
double accelZ0;
double gyroX0;
double gyroY0;
double gyroZ0;

//0 -> 1
int chuteThreshold = 90;  //deg  //gyroscope value for droug chute

//1 -> 2
int solenoidThreshold = 750;  //m above ground level to activate solenoid

//deployment
int onDuration = 10;  //sec  //duration of charge and solenoid
int flickerPeriod = 200;  //msec  //on and off time

//loops
int loopDelay = 0;  //msec
int samples = 3;

//Global Variables
int status = 0;
//Num   Name        Start       End
//0     accent      on          droug
//1     decent      droug       main
//2     landing     main

//Pins
int chutePin = 8;
int solenoidPin = 6;
int ledPin = 7;
int sdPin = 9;

//Objects
File myFile;
Adafruit_BMP085_Unified presSensor = Adafruit_BMP085_Unified(10085);
LSM6DS3 accelSensor(I2C_MODE, 0x6A);
//300.00hPa to 1100 hPa 0.01 hPa accuracy

void setup() {
  //Outputs
    pinMode(chutePin, OUTPUT);
    pinMode(solenoidPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    
    int i = 0;
    while(1) {
      if(i%20 >= 10) {
        digitalWrite(solenoidPin, HIGH);
      }
      else 
      {
        digitalWrite(solenoidPin, LOW);
      }
      if(i%2 >= 1) {
        digitalWrite(ledPin, HIGH);
      }
      else 
      {
        digitalWrite(ledPin, LOW);
      }
      delay(400);
      i++;
    }

  //Serial
    Serial.begin(9600);
    while (!Serial) {
      delay(100);
    }
  
  //Pres
    Serial.print("pres: ");
    Serial.print(presSensor.begin());
    float pres0;
    float temp0;
    
    presSensor.getPressure(&pres0);  //hPa
    presSensor.getTemperature(&temp0);  //C
    presSea = presSensor.seaLevelForAltitude(alt0Actual, pres0, temp0);  //hPa
    alt0 = presSensor.pressureToAltitude(presSea, pres0, temp0); //m

  //Accel
    Serial.print(" accel: ");
    Serial.print(accelSensor.begin() == 0);
    accelSensor.settings.gyroEnabled = 1;  //Can be 0 or 1
    accelSensor.settings.gyroRange = 500;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
    accelSensor.settings.gyroSampleRate = 833;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
    accelSensor.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
    accelSensor.settings.gyroFifoEnabled = 0;  //Set to include gyro in FIFO
    accelSensor.settings.gyroFifoDecimation = 0;  //set 1 for on /1
    
    accelSensor.settings.accelEnabled = 1;
    accelSensor.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
    accelSensor.settings.accelSampleRate = 833;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
    accelSensor.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
    accelSensor.settings.accelFifoEnabled = 0;  //Set to include accelerometer in the FIFO
    accelSensor.settings.accelFifoDecimation = 0;  //set 1 for on /1
    accelSensor.settings.tempEnabled = 0;
    
    //Non-basic mode settings
    accelSensor.settings.commMode = 1;  
    
    //FIFO control settings
    accelSensor.settings.fifoThreshold = 100;  //Can be 0 to 4096 (16 bit bytes)
    accelSensor.settings.fifoSampleRate = 50;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
    accelSensor.settings.fifoModeWord = 0;  //FIFO mode.
    //FIFO mode.  Can be:
    //  0 (Bypass mode, FIFO off)
    //  1 (Stop when full)
    //  3 (Continuous during trigger)
    //  4 (Bypass until trigger)
    //  6 (Continous mode)
    
    //Accelerometer range is set at -4 |+4 g with -/+0.122 mg resolution
    //Gyroscope range is set at -2000 | +2000 dps with +/-70 mdps resolution
    //Output data rate is fixed at 104 Hz*/
    for(int i = 0; i < 100; i++) {
      accelX0 += accelSensor.readFloatAccelX();
      accelY0 += accelSensor.readFloatAccelY();
      accelZ0 += accelSensor.readFloatAccelZ();
      gyroX0 += accelSensor.readFloatGyroX();
      gyroY0 += accelSensor.readFloatGyroY();
      gyroZ0 += accelSensor.readFloatGyroZ();
    }
    accelX0 /= 100;
    accelY0 /= 100;
    accelZ0 /= 100;
    gyroX0 /= 100;
    gyroY0 /= 100;
    gyroZ0 /= 100;

  //SD
    Serial.print(" sd: ");
    Serial.print(SD.begin(sdPin));
    
    Serial.print(" sd open: ");
    Serial.print(SD.open("flight.csv", FILE_WRITE));
    
    Serial.print(" file: ");
    myFile = SD.open("flight.csv", FILE_WRITE);
    Serial.print(myFile);
  
    myFile.println("initial values");
    myFile.print(millis());
    myFile.print(",");
    myFile.print(status);
    myFile.print(",");
    myFile.print(pres0);
    myFile.print(",");
    myFile.print(alt0);
    myFile.print(",");
    myFile.print(temp0);
    myFile.print(",");
    myFile.print(accelX0);
    myFile.print(",");
    myFile.print(accelY0);
    myFile.print(",");
    myFile.print(accelZ0);
    myFile.print(",");
    myFile.print(gyroX0);
    myFile.print(",");
    myFile.print(gyroY0);
    myFile.print(",");
    myFile.print(gyroZ0);
    myFile.print(",presSea:,");
    myFile.println(presSea);
    myFile.println();
    myFile.flush();

}

void loop() {
  static long lastOnTime = millis();
  if(millis() - lastOnTime > flickerPeriod) {
    lastOnTime = millis();
  }
  if(millis() - lastOnTime < flickerPeriod / 2) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
  
  float pres = 0;
  float alt = 0;
  float temp = 0;
  float accelX = 0;
  float accelY = 0;
  float accelZ = 0;
  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;
  
  static double angleX = 0;
  static double angleY = 0;
  static double angleZ = 0;
  //Update sensors

  presSensor.getPressure(&pres);  //hPa
  presSensor.getTemperature(&temp);  //C
  alt = presSensor.pressureToAltitude(presSea, pres, temp) - alt0; //m relative to launch stand
  
  accelX = accelSensor.readFloatAccelX() - accelX0;
  accelY = accelSensor.readFloatAccelY() - accelY0;
  accelZ = accelSensor.readFloatAccelZ() - accelZ0;
  gyroX = accelSensor.readFloatGyroX() - gyroX0;
  gyroY = accelSensor.readFloatGyroY() - gyroY0;
  gyroZ = accelSensor.readFloatGyroZ() - gyroZ0;


  //Calulcations
  angleX += gyroX;
  angleY += gyroY;
  angleZ += gyroZ;

  //Update Status
  static long lastStatusChangeTime = millis();
  switch (status) {
    case 0:

      break;
    case 1:
      if (alt <= solenoidThreshold) {
        status = 2;
      }
      break;
    case 2:

      break;
  }
  digitalWrite(chutePin, HIGH);  //on


  //Store Data
  myFile.print(millis());
  myFile.print(",");
  myFile.print(status);
  myFile.print(",");
  myFile.print(pres);
  myFile.print(",");
  myFile.print(alt);
  myFile.print(",");
  myFile.print(temp);
  myFile.print(",");
  myFile.print(accelX);
  myFile.print(",");
  myFile.print(accelY);
  myFile.print(",");
  myFile.print(accelZ);
  myFile.print(",");
  myFile.print(gyroX);
  myFile.print(",");
  myFile.print(gyroY);
  myFile.print(",");
  myFile.print(gyroZ);
  myFile.println();
  myFile.flush();
  
  
  static double velocityX = 0;
  static double velocityY = 0;
  static double velocityZ = 0;
  
  velocityX += accelX;
  velocityY += accelY;
  velocityZ += accelZ;
  
  Serial.print("X: ");
  Serial.print(velocityX);
  Serial.print(" Y: ");
  Serial.print(velocityY);
  Serial.print(" Z: ");
  Serial.print(velocityZ);
  Serial.print("X: ");
  Serial.print(angleX);
  Serial.print(" Y: ");
  Serial.print(angleY);
  Serial.print(" Z: ");
  Serial.print(angleZ);
  Serial.println();
  
  //Print Outs
  delay(loopDelay);
}
