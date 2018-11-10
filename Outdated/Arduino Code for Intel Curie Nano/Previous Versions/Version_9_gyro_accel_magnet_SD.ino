#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <CurieBLE.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <SD.h>


const int chipSelect = 4; //pin connected to chip select line of SD card

HMC5883L compass;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;


void setup() {
  Serial.begin(9600);
   while (!Serial);    // wait for the serial port to open 

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25); //set sampling rate 25Hz
  CurieIMU.setAccelerometerRate(25); //set sampling rate 25Hz
  filter.begin(25); //converts raw data into 4-dimensional numbers
  
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2); //acceleration is +/- 2G i.e. 1g = 9.8m/s2
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250); //angular range is +/- 250 degrees per second

/*  // Initialize Initialize HMC5883L
   Serial.println("Initialize HMC5883L");
   while (!compass.begin())
   {
     Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
     delay(500);
   } */
   
/*  // Set magnetometer measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set magnetometer measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set magnetometer data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set magnetometer number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set magnetometer calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0); */

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25; //micro seconds needed per reading (period)
  microsPrevious = micros(); //micros() returns number of microseconds since Arduino start running current program. goes back to 0 (overflow) in arond 70 mins
  
  // see if the card is present and can be initialized
  //chip select: the pin connected to the chip select line of the SD card; defaults to the hardware SS line of the SPI bus
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long microsNow;
  float headingdegrees;
  // make a string for assembling the data to log
  String dataString = "";
  
 /* Vector norm = compass.readNormalize(); 

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Singapore declination angle is 0'13E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (0 + (13.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }                             */


  // check if it's time to read data and update the filter
  microsNow = micros();

 
    microsNow = micros();
    
      if(microsNow - microsPrevious >= microsPerReading) 
      {

        // read raw data from CurieIMU
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

       // convert from raw data to gravity and degrees/second units
        ax = convertRawAcceleration(aix); 
        ay = convertRawAcceleration(aiy); 
        az = convertRawAcceleration(aiz); 
        gx = convertRawGyro(gix); 
        gy = convertRawGyro(giy); 
        gz = convertRawGyro(giz); 
    
       
         
    /*  // Convert to magnetometer reading to degrees
         float headingDegrees = heading * 180/M_PI; */

      //record data into string
       dataString += String(ax); dataString += ",";
       dataString += String(ay); dataString += ",";
       dataString += String(az); dataString += ",";
       dataString += String(gx); dataString += ",";
       dataString += String(gy); dataString += ",";
       dataString += String(gz); dataString += ",";
    /*   dataString += String(headingDegrees); */

       // open the file. note that only one file can be open at a time,
       // so you have to close this one before opening another.
       File dataFile = SD.open("datalog.txt", FILE_WRITE);
    
       // if the file is available, write to it:
       if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
        // print to the serial port too:
        Serial.println(dataString);
       }
       // if the file isn't open, pop up an error:
       else {
        Serial.println("error opening datalog.txt");
       }
       
        // These statements are for debugging puposes only and can be commented out to increase the efficiency of the sketch.
         Serial.print("ax="); Serial.print(ax); Serial.print(" "); 
         Serial.print("ay="); Serial.print(ay); Serial.print(" "); 
         Serial.print("az="); Serial.print(az); Serial.print("\n"); 
         Serial.print("gx="); Serial.print(gx); Serial.print(" "); 
         Serial.print("gy="); Serial.print(gy); Serial.print(" "); 
         Serial.print("gz="); Serial.print(gz); Serial.print("\n"); 
     /*    Serial.print(" Degress = "); Serial.print(headingDegrees); Serial.print("\n"); */
       

         microsPrevious = microsPrevious + microsPerReading;
        
      }
   }   


float convertRawAcceleration(int aRaw) {   //convert raw acceleration into m/s2
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {     //convert raw angular acceleration into degrees/s
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


