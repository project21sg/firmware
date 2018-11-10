#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <CurieBLE.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

struct AccelerometerData {
  float x;
  float y;
  float z;
};

volatile struct AccelerometerData accel;


BLEPeripheral blePeripheral;       // BLE Peripheral Device
BLEService imuService("E95D0100-251D-470A-A062-FA1922DFA9A7"); // UUID for service and characteristics


//Acc and Gyro characteristic are custom uuids slightly varied from the one above
//arduino BLE is peripheral, mobile is central system. read: ask peripheral to send back current value of characteristic. 
//write: modify value of characteristic. notify: notify when data has changed
//BLECharacteristic characteristicname(UUID, properties, max length)

BLECharacteristic accelerometerReadChar("E95D0101-251D-470A-A062-FA1922DFA9A7", BLERead | BLENotify, sizeof(AccelerometerData));

BLEService GyroService("0000180F-0000-1000-8000-00805f9b34fb"); // BLE Battery Service
BLEIntCharacteristic GyroChar("00002a19-0000-1000-8000-00805f9b34fb", BLERead | BLENotify);

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

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25; //micro seconds needed per reading (period)
  microsPrevious = micros(); //micros() returns number of microseconds since Arduino start running current program. goes back to 0 (overflow) in arond 70 mins
  
  // prepare & initialize BLE. service is a collection of related data and characteristics are the values.

  blePeripheral.setLocalName("imu"); //add advertised local name
  blePeripheral.setAdvertisedServiceUuid(imuService.uuid());  // add the service UUID
  blePeripheral.addAttribute(imuService);  //add service
  blePeripheral.addAttribute(GyroService);
  blePeripheral.addAttribute(accelerometerReadChar); //add characteristic
  blePeripheral.addAttribute(GyroChar);

 //set value of characteristic to 0 
 const unsigned char initialize[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };
  accelerometerReadChar.setValue(initialize, 12);
  GyroChar.setValue(0);

  blePeripheral.begin();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  unsigned long microsNow;

  

  // check if it's time to read data and update the filter
  microsNow = micros();

  //connect to central i.e. mobile. serial print and run loop if connected 
  BLECentral central = blePeripheral.central();

  
  if (central) {
    
    Serial.print("Connected to central: "); Serial.println(central.address());

    while (central.connected()) {
    microsNow = micros();
    
      if(microsNow - microsPrevious >= microsPerReading) 
      {

        // read raw data from CurieIMU
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

       
       // These statements are for debugging puposes only and can be commented out to increae the efficiency of the sketch.
         Serial.print(gix); Serial.print("\n");

        accel.x = aix;
        accel.y = aiy;
        accel.z = aiz;

         
       //setting new values trigger notification. set value of characteristic to be transmitted to mobile
        accelerometerReadChar.setValue((byte *)&accel, sizeof(accel));
        GyroChar.setValue(gix);

    microsPrevious = microsPrevious + microsPerReading;
        
      }
   }   
 }

  else //this is to make sure that even if mobile not connected, microsPrevious is updated as microsNow still increasing.
  {
    Serial.print("Not Connected to central \n");
    if(microsNow - microsPrevious >= microsPerReading) 
    {
      microsPrevious = microsPrevious + microsPerReading;
    }
  }
}


