#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoBLE.h>
/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)
float tommy = 0.0;
float oldTommy = 0.0;
float nom = 0.0;
const int numReadings = 10;
float readings [numReadings];
int readIndex = 0;
float total = 0;
float aisVal = 0;
float smoother = 0.0;
float timerNow = 0.0;
 float jism = 0.0;
 bool trap = 0;
 float timer = 0.0;
 float finalSmoother = 0.0;
 float angle = 0.0;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
// Full list of Arduino BLE commands: https://github.com/arduino-libraries/ArduinoBLE
BLEService myService("0000ffe0-0000-1000-8000-00805f9b34fb"); // BLExAR BLE service
BLECharacteristic myCharacteristic("FFE1", BLEWrite | BLENotify,0x10); 


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
 if (!BLE.begin()) {
    while (1); // wait for BLE
  }
  //while (!Serial) delay(10);  // wait for serial port to open!

  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  
  myService.addCharacteristic(myCharacteristic); // add BLE characteristic
  BLE.addService(myService); // add BLE service
  BLEAdvertisingData scanData;
  scanData.setLocalName("Elevation"); // set name
  BLE.setDeviceName("Elevation"); // set name

  BLE.setScanResponseData(scanData);// set data for scanners (BLE apps)
  BLE.advertise(); // advertise BLE device
  
  // raw values for fastest response times
  // full list: https://github.com/adafruit/Adafruit_BMP280_Library/blob/master/keywords.txt
  /*
  /* Display the current temperature */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  int8_t temp = bno.getTemp();
  //Serial.print("Current Temperature: ");
  //Serial.print(temp);
  //Serial.println(" C");
  //Serial.println("");
 
  bno.setExtCrystalUse(true);
  getZero();
  
  timerNow = millis();
  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
 
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
   // wait for a BLE central connection
  BLEDevice central = BLE.central();
  
  // if a central is connected to the peripheral:
  if((millis() - timerNow ) > 10000 ){
    timerNow = millis();
    writeBNO055Str();
  }


    
 
    
   
    
  
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
  nom = laccel.z();  //need to transfer info from imu to variable while statement wont work with function in it
  /* Display the floating point data */
  //Serial.print("laccel.z:  ");
  //Serial.println(nom);
 
  angle = euler.z();
  //gets rid of zero outputs
    //all accelerations in pos direction are stopped (neg accel is downward)
   
   timer = millis();
    
  while(abs(nom) <= 0.5){ 
    trap = 1;  
    oldTommy = smoother;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //grabs another euler angle
    imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  //grabs another accel wo gravity
    nom = laccel.z();
    smoother = KALMAN(euler.z());
    float timerTemp = millis() - timer;
    if( timerTemp >500 ) {
    smooth();    
   } 
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  //Serial.print("time locked");
  
   
 
  //Serial.print("smooth()");
  //Serial.println(smooth()); 
 

  /* Display calibration status for each sensor. */
  /*
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
*/
 
 
   
    
}
float smooth() {
  float average;
  trap = 0;
  total = total - readings[readIndex];
  readings[readIndex] = oldTommy;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if(readIndex >= numReadings) {
    readIndex = 0;
  }
  average = total / numReadings;
  return average;
  }
  
  void writeBNO055Str(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 

    
    String strToPrint = ""; // string to print 

    

//    strToPrint+= String(temp,2); strToPrint+=","; // uncomment to send temp
//    strToPrint+= String(pres,2); strToPrint+=","; // uncomment to send pressure
    
    //strToPrint+= String(smooth(),2); strToPrint+="\n"; // uncomment to send altitude
    strToPrint+= String(smooth() -jism,2); strToPrint+="\n"; // uncomment to send altitude
    writeBLE(strToPrint); // send string over BLE
  
  }


void writeBLE(String message){
  byte plain[message.length()]; // message buffer
  message.getBytes(plain, message.length()); // convert to bytes
  myCharacteristic.writeValue(plain,message.length()); // writing to BLE
}
void getZero(){
 
    long templeTime = millis();
    while((millis() - templeTime)  < 10000){
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
       jism = euler.z();
       Serial.print("jiz");
       Serial.print(jism);
       delay(50);
    }
    
}
float KALMAN(float U){
  static const float R = 40;
  static const float H = 1.00;
  static float Q = 10;
  static float P = 0;
  static float U_hat= 0;
  static float K = 0;
  trap = 0;
  K = P* H/(H * P * H + R);
  U_hat = U_hat + K * (U - H*U_hat);
  P = ( 1- K * H) * P + Q;
  return U_hat;
}
  
