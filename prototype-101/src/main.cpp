#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <CurieBLE.h>
#include <SD.h>


const int SD_CARD_PIN = 4; //pin connected to chip select line of SD card
const int ACC_SAMPLING_RATE = 25;
const int GYR_SAMPLING_RATE = 25;
const int ACC_RANGE = 2; //acceleration is +/- 2G i.e. 1g = 9.8m/s2
const int GYR_RANGE = 250; //angular range is +/- 250 degrees per second
const uint8_t BASE_NAME_SIZE = sizeof("log") - 1;

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
// BLEUnsignedIntCharacteristic SensorStatus1("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite  );
BLEService GaitSensor1("19B10010-E8F2-537E-4F6C-D104768A1214"); // BLE AnalogRead Service

BLECharacteristic ga_acc("AA00", BLERead | BLENotify , 12 ); //acc X,Y,Z | BLENotify
BLECharacteristic ga_gyr("AB00", BLERead | BLENotify , 12 ); //gyr X,Y,Z + time maybe | BLENotify

int switchcontrol = 0;
int filecontrol = 1;

File datafile;
char fileName[] = "log00.csv";

//Madgwick filter;
unsigned long microsPerReading = 1000000 / 25; //low frames for stability
unsigned long microsPrevious;

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

void initSDCardFile() {
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
}

void SDCardFileWrite(float dataBuffer) {
  datafile = SD.open(fileName, FILE_WRITE);
  datafile.println(dataBuffer);
  datafile.flush();
  datafile.close();
}

void populateBuffer(float buffer[], float values[]) {
  int j = 0;
  for(unsigned int i = 0; i<= 6; i++) {
    buffer[i] = values[j++];
  }
}

/*
 Event handlers for BLE
*/
void BLESensorWrittenHandler(BLECentral &central, BLECharacteristic &characteristic) {
  Serial.print(F("Data received from central"));
  switchcontrol = 1;
}

void BLEServiceConnectHandler(BLECentral& central) {
  // central connected event handler
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void BLEServiceDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
  switchcontrol = 0;
}

void configureSDReader() {
  // wait for SD card to be detected in slot
  while (!SD.begin(SD_CARD_PIN)) {}
  filecontrol = 1; //this stupid flag just signals when to restart writing to a new file, if SET means need to write new file
}

void configureCurie() {
  CurieIMU.begin();   // start the IMU and filter
  CurieIMU.setGyroRate(ACC_SAMPLING_RATE); //set sampling rate 25Hz
  CurieIMU.setAccelerometerRate(GYR_SAMPLING_RATE); //set sampling rate 25Hz
  CurieIMU.setGyroRange(GYR_RANGE); 
  CurieIMU.setAccelerometerRange(ACC_RANGE); 

  //filter.begin(25); //raw yaw and pitch generator, use with .updateIMU() and .getX
  pinMode(13, OUTPUT);
}

void configureBLE() {
  BLE.begin();

  BLE.setDeviceName("P21"); //works?
  BLE.setAppearance(0x0080);
  // set advertised local name and service UUID:
  BLE.setLocalName("GaitSensor1");
  BLE.setAdvertisedService(GaitSensor1);
  // add service and characteristic:
  GaitSensor1.addCharacteristic(ga_acc);
  GaitSensor1.addCharacteristic(ga_gyr);
  BLE.addService(GaitSensor1);

  BLE.advertise();
}

void copyIntoByteArray(unsigned char buffer[], int nBytesOffset, int value) {
  buffer[nBytesOffset] = (value >> 24) & 0xFF; 
  buffer[nBytesOffset+1] = (value >> 16) & 0xFF; 
  buffer[nBytesOffset+2] = (value >> 8) & 0xFF; 
  buffer[nBytesOffset+3] = value & 0xFF; 
}

void setup() {
  //Serial.begin(115200);
  configureCurie();
  //configureSDReader();
  configureBLE();
  microsPrevious = micros(); //micros() returns number of microseconds since Arduino start running current program. goes back to 0 (overflow) in arond 70 mins
}

void loop() {
  int aix, aiy, aiz, gix, giy, giz;
  //float ax, ay, az, gx, gy, gz;
  unsigned long microsNow;
  //float dataBuffer[6]; 

  unsigned char accBuffer[12];
  unsigned char gyrBuffer[12];

  //Serial.println("yas");
  BLEDevice central = BLE.central(); //refactor into event handler

  if(central) { // if a central is connected to peripheral:
    digitalWrite(13, HIGH);
    while(central.connected()) {
      microsNow = micros();
      if(microsNow - microsPrevious >= microsPerReading) { 
        microsPrevious = microsNow;

        noInterrupts();
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
        interrupts();

        copyIntoByteArray(accBuffer, 0, aix);
        copyIntoByteArray(accBuffer, 4, aiy);
        copyIntoByteArray(accBuffer, 8, aiz);
        copyIntoByteArray(gyrBuffer, 0, gix);
        copyIntoByteArray(gyrBuffer, 4, giy);
        copyIntoByteArray(gyrBuffer, 8, giz);

        // ax = convertRawAcceleration(aix);
        // ay = convertRawAcceleration(aiy);
        // az = convertRawAcceleration(aiz);
        // gx = convertRawGyro(gix);
        // gy = convertRawGyro(giy);
        // gz = convertRawGyro(giz);
        // seconds = micros();

        // float values[] = {ax, ay, az, gx, gy, gz};
        // populateBuffer(dataBuffer, values);

        //SDCardFileWrite(dataBuffer);
        //Write to connected bluetooth device
        //Serial.println(microsNow);
        if(ga_acc.canNotify()) { ga_acc.setValue(accBuffer, 12) ? Serial.println("acc done") : Serial.println("acc crashed?");}
        if(ga_gyr.canNotify()) { ga_gyr.setValue(gyrBuffer, 12) ? Serial.println("gyr done") : Serial.println("gyr crashed?");}
        // ga_acc.setValue(accBuffer, 12);
        // ga_gyr.setValue(gyrBuffer, 12);
      } 
    }

    digitalWrite(13, LOW);
  } 
  // else {//if central not connected
  //     filecontrol = 1;
  // }
}