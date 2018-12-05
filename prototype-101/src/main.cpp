#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <CurieBLE.h>
#include <SD.h>

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedIntCharacteristic SensorStatus1("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite  );
BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService GaitSensor1("19B10010-E8F2-537E-4F6C-D104768A1214"); // BLE AnalogRead Service

const int SD_CARD_PIN = 4; //pin connected to chip select line of SD card
const int ACC_SAMPLING_RATE = 25;
const int GYR_SAMPLING_RATE = 25;
const int ACC_RANGE = 2; //acceleration is +/- 2G i.e. 1g = 9.8m/s2
const int GYR_RANGE = 250; //angular range is +/- 250 degrees per second

int switchcontrol = 0;
int filecontrol = 0;

File datafile;
const uint8_t BASE_NAME_SIZE = sizeof("log") - 1;
char fileName[] = "log00.csv";

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

/*
  Converts raw acceleration values into m/s2, for range of 2G
  -2g maps to a raw value of -32768 | +2g maps to a raw value of 32767
*/
float convertRawAcceleration(int aRaw) { 
  return (aRaw * ACC_RANGE) / 32768.0;
}
/*
  Converts raw angular acceleration values into degrees/s, for range of 250 degrees/s
  -250 maps to a raw value of -32768 | +250 maps to a raw value of 32767
*/
float convertRawGyro(int gRaw) {
  return (gRaw * GYR_RANGE) / 32768.0;
}

void setup() {
  CurieIMU.begin();   // start the IMU and filter
  CurieIMU.setGyroRate(ACC_SAMPLING_RATE); //set sampling rate 25Hz
  CurieIMU.setAccelerometerRate(GYR_SAMPLING_RATE); //set sampling rate 25Hz
  filter.begin(25); //converts raw data into 4-dimensional numbers
  CurieIMU.setAccelerometerRange(ACC_RANGE); 
  CurieIMU.setGyroRange(GYR_RANGE); 

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25; //micro seconds needed per reading (period)
  microsPrevious = micros(); //micros() returns number of microseconds since Arduino start running current program. goes back to 0 (overflow) in arond 70 mins
  
  // wait for SD card to be detected in slot
  while (!SD.begin(SD_CARD_PIN)) {}
  filecontrol = 1;

  // set advertised local name and service UUID:
  blePeripheral.setLocalName("GaitSensor1");
  blePeripheral.setAdvertisedServiceUuid(GaitSensor1.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(GaitSensor1);
  blePeripheral.addAttribute(SensorStatus1);

  // begin advertising BLE Light service:
  blePeripheral.begin();
}

void loop() {
  int aix, aiy, aiz, gix, giy, giz;
  float ax, ay, az, gx, gy, gz;
  unsigned long microsNow, seconds;
  String dataString = ""; //TODO: convert to buffer

  microsNow = micros();
  BLECentral central = blePeripheral.central();
  if(central) { // if a central is connected to peripheral:
    if(filecontrol == 1) {
      while (SD.exists(fileName)) {
        if (fileName[BASE_NAME_SIZE + 1] != '9') {
          fileName[BASE_NAME_SIZE + 1]++;
        } else if (fileName[BASE_NAME_SIZE] != '9') {
          fileName[BASE_NAME_SIZE + 1] = '0';
          fileName[BASE_NAME_SIZE]++;
        } else {
          //Serial.println(F("Can't create file name"));
          return;
        }
      }
      datafile = SD.open(fileName, FILE_WRITE); // create a new file. File name cannot be too long!
      if (!datafile) {return;}
      datafile.close(); //close file
      filecontrol = 0;
  }
     
  if(microsNow - microsPrevious >= microsPerReading) {        
    if(SensorStatus1.written()) {
      switchcontrol = SensorStatus1.value();
    }

    if(switchcontrol == 1) {
      microsPrevious = microsNow;
      CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
      ax = convertRawAcceleration(aix); 
      ay = convertRawAcceleration(aiy); 
      az = convertRawAcceleration(aiz); 
      gx = convertRawGyro(gix); 
      gy = convertRawGyro(giy); 
      gz = convertRawGyro(giz); 
      seconds = micros();
      dataString += String(ax); dataString += ",";
      dataString += String(ay); dataString += ",";
      dataString += String(az); dataString += ",";
      dataString += String(gx); dataString += ",";
      dataString += String(gy); dataString += ",";
      dataString += String(gz); dataString += ",";
      dataString += String(seconds); 
      // dataString += String(headingDegrees);
      datafile = SD.open(fileName, FILE_WRITE);
      datafile.println(dataString);
      datafile.flush();
      datafile.close();
        // These statements are for debugging puposes only and can be commented out to increase the efficiency of the sketch.
      /*   Serial.print("ax="); Serial.print(ax); Serial.print(" "); 
         Serial.print("ay="); Serial.print(ay); Serial.print(" "); 
         Serial.print("az="); Serial.print(az); Serial.print("\n"); 
         Serial.print("gx="); Serial.print(gx); Serial.print(" "); 
         Serial.print("gy="); Serial.print(gy); Serial.print(" "); 
         Serial.print("gz="); Serial.print(gz); Serial.print("\n"); 
         Serial.print(" Degress = "); Serial.print(headingDegrees); Serial.print("\n"); */
    }
  } else {//if central not connected
      filecontrol = 1;
    } 
  }   