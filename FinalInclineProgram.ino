#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoBLE.h>


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)
float average;
float tommy = 0.0;
float oldTommy = 0.0;
float nom = 0.0;
const int numReadings = 5;
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
  

  /* Initialise the sensor */
  
  myService.addCharacteristic(myCharacteristic); // add BLE characteristic
  BLE.addService(myService); // add BLE service
  BLEAdvertisingData scanData;
  //you can call your device anything you want to find it 
  scanData.setLocalName("Elevation"); // set name
  BLE.setDeviceName("Elevation"); // set name

  BLE.setScanResponseData(scanData);// set data for scanners (BLE apps)
  BLE.advertise(); // advertise BLE device
  //This is the pin that your momentary zero switch is connected to 
  pinMode(D10,INPUT_PULLUP);
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
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 
 //This initially zeros out the unit in the first ten seconds

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
  //This timer determines how often the data is uploaded via bluetooth to your phone...in this case every 5 seconds
  if((millis() - timerNow ) > 5000 ){
    timerNow = millis();
    writeBNO055Str();
  }
//this interupts to zero out the machine if you hold the zero button down
if (!digitalRead(D10))getZero();
    
 
    
   
    
  
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
  //nom is holding the acceleration in z or up and down direction
  nom = laccel.z();  //need to transfer info from imu to variable while statement wont work with function in it
  /* Display the floating point data */
  //Serial.print("laccel.z:  ");
  //Serial.println(nom);
  timer = millis();
  //euler.z() is holding the angle relative to the flat plane of the shoe that the unit is tipped
  angle = euler.z();
  while(abs(nom) < 0.2){
    //while the absolute amt of acceleration in the z direction is low new values are chosen to monitor the angle
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //grabs another euler angle
    imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  //grabs another accel wo gravity
    nom = laccel.z();
    
    float timerTemp = millis() - timer;
    //if more than 100 ms has passed with little acceleration the angle value is sent to a kalman filter 
    if(timerTemp > 100 ){
      smoother = KALMAN(euler.z());
      trap = 1;
      //Serial.print("smoother: ");
      //Serial.println(smoother);
    }
    //If nothing is happening for a second break out of the loop
    if(timerTemp > 1000)break;
    delay(20);
  }
  //if it's caught a step in the above section send the Kalman average into a smoother function that averages 10 values
  if(trap == 1){
    oldTommy = smoother;
    float serialSmooth = smooth();
    Serial.print("smooth:  ");
    Serial.println(serialSmooth);
    trap = 0;
  }
  //delay the next reading for 50 ms
  delay(BNO055_SAMPLERATE_DELAY_MS);
    
}
//Smoother function is runnibng average of 10 readings
float smooth() {
  
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
  //Does the Bluetooth connection
  void writeBNO055Str(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 

    
    String strToPrint = ""; // string to print 

    

//    strToPrint+= String(temp,2); strToPrint+=","; // uncomment to send temp
//    strToPrint+= String(pres,2); strToPrint+=","; // uncomment to send pressure
    
    //strToPrint+= String(smooth(),2); strToPrint+="\n"; // uncomment to send altitude
    strToPrint+= String(average -jism,2); strToPrint+="\n"; // uncomment to send altitude
    writeBLE(strToPrint); // send string over BLE
  
  }


void writeBLE(String message){
  byte plain[message.length()]; // message buffer
  message.getBytes(plain, message.length()); // convert to bytes
  myCharacteristic.writeValue(plain,message.length()); // writing to BLE
}
 //Does the Zero our level
void getZero(){
 
    long templeTime = millis();
    while((millis() - templeTime)  < 10000){
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
       jism = euler.z();
       Serial.print("jiz");
       Serial.print(jism);
       delay(50);
    }
  Serial.print(" system= ");
  //Serial.println(system,DEC);
  //Serial.println(system);
}
//Kalman filter for the raw readings
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
  
