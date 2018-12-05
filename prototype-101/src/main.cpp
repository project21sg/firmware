#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <CurieBLE.h>
#include <SD.h>

File datafile;
const uint8_t BASE_NAME_SIZE = sizeof("log") - 1;
char fileName[] = "log00.csv";

const int chipSelect = 4; //pin connected to chip select line of SD card

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService GaitSensor1("19B10010-E8F2-537E-4F6C-D104768A1214"); // BLE AnalogRead Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedIntCharacteristic SensorStatus1("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite  );

int switchcontrol;
int filecontrol;


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


void setup() {
  //Serial.begin(9600);
   //while (!Serial);    // wait for the serial port to open  

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25); //set sampling rate 25Hz
  CurieIMU.setAccelerometerRate(25); //set sampling rate 25Hz
  filter.begin(25); //converts raw data into 4-dimensional numbers
  
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2); //acceleration is +/- 2G i.e. 1g = 9.8m/s2
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250); //angular range is +/- 250 degrees per second


  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25; //micro seconds needed per reading (period)
  microsPrevious = micros(); //micros() returns number of microseconds since Arduino start running current program. goes back to 0 (overflow) in arond 70 mins
  
  // see if the card is present and can be initialized
  //chip select: the pin connected to the chip select line of the SD card; defaults to the hardware SS line of the SPI bus
  
  SD.begin(chipSelect);

  
  while (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
  }
  //Serial.println("card initialized."); 


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
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long microsNow;
  unsigned long seconds;
  // make a string for assembling the data to log
  String dataString = "";


  // check if it's time to read data and update the filter
  microsNow = micros();

     BLECentral central = blePeripheral.central();
    // if a central is connected to peripheral:
    if (central) {   

      if(filecontrol == 1)
      {
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
        
            // create a new file. File name cannot be too long!
           datafile = SD.open(fileName, FILE_WRITE);
            if (!datafile) {
              //Serial.println("couldnt create datalog");
              return;
            }
          //Serial.print("opened: ");
          //Serial.println(fileName);
          datafile.close(); //close file

          filecontrol = 0;
      }
     
      if(microsNow - microsPrevious >= microsPerReading) 
      {     
             
        if(SensorStatus1.written())
        {
          switchcontrol = SensorStatus1.value();
        }

        if(switchcontrol == 1)
        {

        microsPrevious = microsNow;

        // read raw data from CurieIMU
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

       // convert from raw data to gravity and degrees/second units
        ax = convertRawAcceleration(aix); 
        ay = convertRawAcceleration(aiy); 
        az = convertRawAcceleration(aiz); 
        gx = convertRawGyro(gix); 
        gy = convertRawGyro(giy); 
        gz = convertRawGyro(giz); 
    
        seconds = micros();
       
      //record data into string
       dataString += String(ax); dataString += ",";
       dataString += String(ay); dataString += ",";
       dataString += String(az); dataString += ",";
       dataString += String(gx); dataString += ",";
       dataString += String(gy); dataString += ",";
       dataString += String(gz); dataString += ",";
       dataString += String(seconds); 
    /*   dataString += String(headingDegrees); */


       // open the file. note that only one file can be open at a time,
       // so you have to close this one before opening another.
       datafile = SD.open(fileName, FILE_WRITE);
       if ( !datafile )
        {
     //     Serial.println("error opening datalog.csv");
    //      return;
        }


         // if the file is available, write to it:
        datafile.println(dataString);
        // print to the serial port too:
        //   Serial.println(dataString);

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

      else if(switchcontrol == 0)
      {
        ;
      }
      
      }

    }
        
     else //if central not connected
     { 
      filecontrol = 1;
     }
     
   }   