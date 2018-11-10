#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <CurieBLE.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

BLEPeripheral blePeripheral;       // BLE Peripheral Device
BLEService imuService("917649A0-D98E-11E5-9EEC-0002A5D5C51B"); // Custom UUID 128bit format for a set of data. UUID for service and characteristics

//12 bytes of data for 3 floats for acc and gyro each. Acc and Gyro characteristic are custom uuids slightly varied from the one above
//app button characteristic allows arduino to receive button inputs from mobile app. a single byte can represent 256 values i.e. 256 buttons
//arduino BLE is peripheral, mobile is central system. read: ask peripheral to send back current value of characteristic. 
//write: modify value of characteristic. notify: notify when data has changed
//BLECharacteristic characteristicname(UUID, properties, max length)

BLECharacteristic imuAccCharacteristic("917649A1-D98E-11E5-9EEC-0002A5D5C51B", BLERead | BLENotify, 12 );
BLECharacteristic imuGyroCharacteristic("917649A2-D98E-11E5-9EEC-0002A5D5C51B", BLERead | BLENotify, 12 );
BLEUnsignedCharCharacteristic appButtonCharacteristic("917649A7-D98E-11E5-9EEC-0002A5D5C51B", BLERead | BLEWrite );

BLEService AccService("0000180F-0000-1000-8000-00805f9b34fb"); // BLE Battery Service
BLEUnsignedCharCharacteristic AccChar("00002a19-0000-1000-8000-00805f9b34fb", BLERead | BLENotify);

//descriptor defines attribute that describe a characteristic. BLEDescriptor(const char* uuid of descriptor , const char* string value);
BLEDescriptor imuAccDescriptor("2902", "block");
BLEDescriptor imuGyroDescriptor("2902", "block");

//union allows 3 variables to share memory location. float array of 3 string of values. unsigned character 12 bytes as each value maxmimum 4 bytes
//needed as data can only be transmitted in character format not float.
union 
 {
  float a[3];
  unsigned char bytes[12];      
 } accData;

 union 
 {
  float g[3];
  unsigned char bytes[12];         
 } gyroData;

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

  // prepare & initiazlie BLE. service is a collection of related data and characteristics are the values.

  blePeripheral.setLocalName("imu"); //add advertised local name
  blePeripheral.setAdvertisedServiceUuid(imuService.uuid());  // add the service UUID
  blePeripheral.addAttribute(imuService);  //add service
  blePeripheral.addAttribute(imuAccCharacteristic); //add characteristic
  blePeripheral.addAttribute(imuAccDescriptor); //add descriptor
  blePeripheral.addAttribute(imuGyroCharacteristic);
  blePeripheral.addAttribute(imuGyroDescriptor);
  blePeripheral.addAttribute(appButtonCharacteristic);

  // All characteristics should be initialized to a starting value prior to using them
  const unsigned char initializerAcc[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };
  const unsigned char initializerGyro[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };

 //set value of characteristic to 0 using above defined array
  imuAccCharacteristic.setValue( initializerAcc, 12);
  imuGyroCharacteristic.setValue( initializerGyro, 12 );
  appButtonCharacteristic.setValue(0);

  AccChar.setValue(0);
  
  blePeripheral.begin();
  

}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
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

        // convert from raw data to gravity and degrees/second units 
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
    
        //save data into float array
        accData.a[0] = ax;
        accData.a[1] = ay;
        accData.a[2] = az;
    
        gyroData.g[0] = gx;
        gyroData.g[1] = gy;
        gyroData.g[2] = gz;

       // These statements are for debugging puposes only and can be commented out to increae the efficiency of the sketch.
       Serial.print( "(ax,ay,az): " ); 
       Serial.print("("); Serial.print(accData.a[0]); Serial.print(","); Serial.print(accData.a[1]); Serial.print(","); Serial.print(accData.a[2]); Serial.print(")");Serial.println();
       Serial.print( "(gx,gy,gz): " ); 
       Serial.print("("); Serial.print(gyroData.g[0]); Serial.print(","); Serial.print(gyroData.g[1]); Serial.print(","); Serial.print(gyroData.g[2]); Serial.print(")");Serial.println();
  
       //convert float array accData into unsigned char string acc. &accData gives address. * is an operator that returns value of the address.
       unsigned char *acc = (unsigned char *)&accData;
       unsigned char *gyro = (unsigned char *)&gyroData;

       //setting new values trigger notification. set value of characteristic to be transmitted to mobile
       imuAccCharacteristic.setValue( acc, 12 );
       imuGyroCharacteristic.setValue( gyro, 12 );
       AccChar.setValue(ax);
  
        /**
        * When a button is pressed on the mobile app, the value of the characteristic is changed and
        * sent over BLE to the arduino/genuino101. A change in the characteristic is indicated
        * by a true value from the written() function and the value transmitted from the button 
        * on the mobile app is read here with the value() function.
        * The values change here essentially do nothing but print out a line to the serial port.
        * You can make them do anything here.
        * 
       */    
       /*if ( appButtonCharacteristic.written() ) {
  
          int appButtonValue = appButtonCharacteristic.value();
  
          switch(appButtonValue) {
  
            case 0:
              Serial.println( "App Input Value(0): " + appButtonValue );
              break;
            case 1:
              Serial.println( "App Input Value(1): " + appButtonValue );
              break;
            case 2:  
               Serial.println( "App Input Value(2): " + appButtonValue );
              break;  
          }
          
      }*/

    // update the filter, which computes orientation. to convert raw data into a dimensional position value
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll. get roll, pitch and yaw i.e. posistion in 3D
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();


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



//things to settle
//2. bluetooth + battery
//3. cover
//4. raw data to meaningful stuff. foot step length, angle up and down, left and right?
