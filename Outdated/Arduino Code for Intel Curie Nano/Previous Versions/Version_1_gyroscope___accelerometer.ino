#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

float rollstart, pitchstart, headingstart;

void setup() {
  Serial.begin(9600);

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

}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;
  
  
  int aixstart, aiystart, aizstart;
  int gixstart, giystart, gizstart;
  float axstart, aystart, azstart;
  float gxstart, gystart, gzstart;


  // check if it's time to read data and update the filter
  microsNow = micros();

  if(microsNow < 5000000)
  {if(microsNow - microsPrevious >= microsPerReading)
    {
     microsPrevious = microsPrevious + microsPerReading;
    }
  }
  
  if(microsNow > 5000000 && microsNow < 8000000)
  {
      if(microsNow - microsPrevious >= microsPerReading) {
        
        // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aixstart, aiystart, aizstart, gixstart, giystart, gizstart);

    // convert from raw data to gravity and degrees/second units
    axstart = convertRawAcceleration(aixstart);
    aystart = convertRawAcceleration(aiystart);
    azstart = convertRawAcceleration(aizstart);
    gxstart = convertRawGyro(gixstart);
    gystart = convertRawGyro(giystart);
    gzstart = convertRawGyro(gizstart);

    // update the filter, which computes orientation. to convert raw data into a dimensional position value
    filter.updateIMU(gxstart, gystart, gzstart, axstart, aystart, azstart);

    // print the heading, pitch and roll. get roll, pitch and yaw i.e. posistion in 3D
    rollstart = filter.getRoll();
    pitchstart = filter.getPitch();
    headingstart = filter.getYaw();

    Serial.print("Orientation: ");
    Serial.print(headingstart);
    Serial.print(" ");
    Serial.print(pitchstart);
    Serial.print(" ");
    Serial.println(rollstart);
    Serial.println("LOL\n");


    microsPrevious = microsPrevious + microsPerReading;
        
      }

  }   
  if(microsNow > 5000000 && microsNow > 8000000)
    if(microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix); 
    ay = convertRawAcceleration(aiy); 
    az = convertRawAcceleration(aiz); 
    gx = convertRawGyro(gix); 
    gy = convertRawGyro(giy); 
    gz = convertRawGyro(giz); 

  
    // update the filter, which computes orientation. to convert raw data into a dimensional position value
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll. get roll, pitch and yaw i.e. posistion in 3D
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    Serial.println("\n");

    
    Serial.print("Orientation: ");
    Serial.print(headingstart);
    Serial.print(" ");
    Serial.print(pitchstart);
    Serial.print(" ");
    Serial.println(rollstart);
    Serial.println("\n");
    
    roll = roll - rollstart;
    pitch = pitch - pitchstart;
    heading = heading - headingstart;
    
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    Serial.println("\n");

    // increment previous time, so we keep proper pace
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



//things to settle
//1. where is 0 pos.ition?
//2. bluetooth + battery
//3. cover
