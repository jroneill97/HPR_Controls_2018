//                                        //
//  High-Powered Rocket MQP 2018 - 2019   //
//  Test Launch flight controller         //
//                                        //

////////// Include all necessary libraries //////////
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h>
#include <Wire.h>
#include <SPI.h>    // SPI communication for SD card
#include <SD.h>     // SD card library
#include "TMRpcm.h" // Music player library


#define BNO055_SAMPLERATE_DELAY_MS (100) // Time (ms) delay between loop iterations

////////// Define all digital pins here //////////
const int SDpin        = 30; // Pin for SD card (change this to whatever it is set to)
const int ejectionPin  = 33; // Pin number for the ejection charge pin    
const int RFpin        = 32; // Pin number for the RF transmitter data out

////////// Define the Strain Gague Pins here //////////
const int strain1      = A0;
const int strain2      = A2;
const int strain3      = A4;

////////// Definable Variables Critical to Mission //////////
unsigned long emergencyEjectionTime                   = 11000; // Milliseconds after launch where the ejection charge will go off regardless
unsigned long minimumEjectionTime                     = 6000;  // Minimum time of ejection
double        launchDetectHeight                      = 5;     // Height where launch is detected (meters)
double        linearAccelerationLaunchDetectThreshold = 10;    // Linear acceleration threshold (ms^-2) to help detect launch
double        apogeeAltDifferenceThreshold            = 1.5;   // Altitude difference for apogee detection (m)

////////// Define variables here //////////
String        dataString;                       // String of data which is written to SD Card
unsigned long adjustedTime;                     // Time read by arduino. "Resets" to 0 at launch using timeDifference variable
unsigned long timeDifference        = 0;        // Difference in time which "resets" time to 0 at launch
double        linearAccelerationX   = 0;        // Z component of linear acceleration

////////// logic variables describing events during flight //////////
boolean launch    = false; 
boolean apogee    = false; 
boolean mainChute = false;

////////// Variables read from sensors //////////
float altDifference;  // Subtraction number for altitude creep
float correctedAlt;   //
float measuredAlt;    // Current altitude
float lastAlt;        // Previous altitude
float groundAlt = 0;  // Altitude measured at the ground

////////// Define the sensors here //////////
Adafruit_MPL3115A2 altimeter = Adafruit_MPL3115A2();
Adafruit_BNO055    IMU       = Adafruit_BNO055();
TMRpcm tmrpcm; // Music player object
/*-----------------------------------------------------------------------------------------------------------------------------------                                 
 * 
 *                                                          SETUP
 *
 *-----------------------------------------------------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(9600);  
// Begins Sensors
  altimeter.begin();
  IMU.begin();
  SD.begin(SDpin);
  Serial.println("Sensor and SD card driver setup complete");
  
  pinMode(ejectionPin, OUTPUT);      // Sets the ejection pin to be an output
  pinMode(RFpin, OUTPUT);            // Sets the RF pin to be an output
  
  IMU.setExtCrystalUse(true);        // Use external crystal for better accuracy
  Serial.println("Acquiring Ground Altitude...");
  altDifference = altimeter.getAltitude();       // Uses an average of 1000 readings to find the ground altitude
  lastAlt       = altDifference;                 // Sets the "last" altitude to the ground altitude
  Serial.println("Setup Complete");
}
/*-----------------------------------------------------------------------------------------------------------------------------------                                 
 * 
 *                                                          LOOP
 *
 *-----------------------------------------------------------------------------------------------------------------------------------*/
void loop() { 
  ////////// Get a new sensor event for the IMU ////////// 
  sensors_event_t event;
  IMU.getEvent(&event);
  
  ////////////////////////////////////// Read data from sensors /////////////////////////////////////////////////////
  float measuredAlt          = altimeter.getAltitude(); // Function from altimeter library which aquires current altitude in meters
    
  imu::Quaternion quat       = IMU.getQuat();
  imu::Vector<3> accel       = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> mag         = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> gyro        = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linearAccel = IMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gravity     = IMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  ////////// Aquire Calibration States //////////
  uint8_t system, gyroCal, accelCal, magCal = 0;
  IMU.getCalibration(&system, &gyroCal, &accelCal, &magCal);
  
  ////////// Create the data string //////////
  String   dataString  = "Launch: "              +(String)launch  +" Apogee: " + (String)apogee+" Main Chute: "+(String)mainChute+"\t";                               // Event Booleans
           dataString += "Time: "                +(String)((double)adjustedTime/1000)+"  \t";                                                                           // Current Time
           dataString += "Corrected alt (m): "   +(String)correctedAlt+"   \t";                                                                    // Current Corrected altitude
           dataString += "Cal: Sys: "            +(String)system  +" Gyro: "+ (String)gyroCal+" Accel: " + (String)accelCal+ " Mag: " + (String)magCal+"  \t";      // Calibration values 
           dataString += "Quat: "                +(String)quat.w() +" "+(String)quat.x() +" "+(String)quat.y() +" "+(String)quat.z() +"  \t";             // Quaternions
           dataString += "LinAccel: "            +(String)linearAccel.x()+" "+(String)linearAccel.y()+" "+(String)linearAccel.z()+"  \t";                 // Linear Acceleration
           dataString += "Accel: "               +(String)accel.x()+" "+(String)accel.y()+" "+(String)accel.z()+"  \t";                                   // Accelerometer
           dataString += "Mag: "                 +(String)mag.x()  +" "+(String)mag.y()  +" "+(String)mag.z()  +"  \t";                                   // Magnetometer
           dataString += "Gyro: "                +(String)gyro.x() +" "+(String)gyro.y() +" "+(String)gyro.z() +"   \t";                                  // Gyroscopic sensor
           dataString += "Measured Alt: "        +(String)measuredAlt+"   \t";
           dataString += "Strain Gague: "        +(String)analogRead(strain1)+"\t"+(String)analogRead(strain2)+"\t"+(String)analogRead(strain3);  // Strain gague readings

  ////////// Write the data string to the SD card //////////
  writeToSD(dataString);
  
  ////////// Adjust the time and altitude for when rocket is sitting on launch pad //////////
  correctedAlt = measuredAlt - altDifference; // Every loop this subtracts the measured altitude from the altimeter by the difference variable

  ////////// Set launch event triggers according to states measured from sensors //////////  
  linearAccelerationX = (double)linearAccel.x();
  if (launch == false){
  launch = detectLaunch(measuredAlt, groundAlt, correctedAlt, launchDetectHeight, linearAccelerationX);
  }
  
  adjustedTime = millis() - timeDifference;   // Every loop this subtracts millis() by the time difference variable
  apogee = detectApogee(measuredAlt, lastAlt, adjustedTime, launch);
  
  //If apogee is acheived, deploy the main chute

  if (apogee == true) {
    mainChute = true;
    deployMainChute();
  }
  if(mainChute == true){
    deployRF();
    deployMainChute();
  }
  ////////// Write everything to the SD card, and set the current height measurement to the "last" measurement
  lastAlt = correctedAlt; // Sets the "memory" altitude
delay(BNO055_SAMPLERATE_DELAY_MS);
}

/*-----------------------------------------------------------------------------------------------------------------------------------                                 
 * 
 *                                                          FUNCTIONS
 *
 *-----------------------------------------------------------------------------------------------------------------------------------*/
////////// Function which detects whether launch happened and sets isLaunch accordingly //////////
boolean detectLaunch(float measuredAlt,float groundAlt, float correctedAlt, float launchDetectHeight, double linearAccelerationX){
  if((linearAccelerationX < linearAccelerationLaunchDetectThreshold) && (launch == false)){
    altDifference = measuredAlt;
    groundAlt     = correctedAlt;
  }
  if(((correctedAlt - groundAlt) > launchDetectHeight)) {
    timeDifference = millis();  // Sets the time difference to current time - "resets" the clock to 0sec at time of launch
    return true;    
  }
  return false;
}

////////// Detects apogee and sets isApogee accordingly //////////
boolean detectApogee(float measuredAlt, float lastAlt, unsigned long adjustedTime, boolean launch) {
  // If launch has occurred and the main chute hasn't deployed, start searching whether the altitude is beginning to decrease
  if(launch == true) {
    if ((adjustedTime > minimumEjectionTime) && (((lastAlt - correctedAlt) > apogeeAltDifferenceThreshold) || (adjustedTime > emergencyEjectionTime))) {
      return true;
    }
  }
  return false;
}

////////// Function for all events necessary to deploy the parachute //////////
void deployMainChute(){
  // This is where we code whatever the servo motor does to deploy the main parachute
  // add printout for calling this
  digitalWrite(ejectionPin,HIGH);
  digitalWrite(13,HIGH);
}

////////// Function which turns the RF transmitter on //////////
void deployRF(){
  // Code for activating the RF transmitter goes here
  int delayTime = 350 + (int)(correctedAlt*0.25);
  
  tone(RFpin, 691/2,delayTime);
  delay(delayTime+5);
  
  tone(RFpin, 691/2,delayTime);
  delay(delayTime+5);
  
  tone(RFpin, 750/2,delayTime);
  delay(delayTime+5);
  
  tone(RFpin, 800/2,delayTime);
  delay(delayTime+5);
}

////////// Function writes all data to the SD card when called //////////
void writeToSD(String stringToWrite){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("tstlnch.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    Serial.println(stringToWrite);
    dataFile.println(stringToWrite);
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

