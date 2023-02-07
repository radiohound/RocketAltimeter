// Full orientation sensing using NXP's advanced sensor fusion algorithm.
//
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// added altitude reading - this requires new sparkfun mpl3115 library to be added to arduino ide

#include <NXPMotionSense.h>     //https://github.com/PaulStoffregen/NXPMotionSense
#include <SparkFunMPL3115A2.h>  //https://github.com/sparkfun/SparkFun_MPL3115A2_Breakout_Arduino_Library
#include <Wire.h>
#include <EEPROM.h>
#include <kalman.h>             // https://www.andico.org/2013/06/1-dimensional-kalman-filter-arduino.html?m=1
#include <CircularBuffer.h>     // https://github.com/rlogiacco/CircularBuffer

//Create an instance of the object
MPL3115A2 myPressure;
NXPMotionSense imu;
NXPSensorFusion filter;
KalmanFilter kf(0, 0.008, 10); //was (0, .01, 1.0) 

//launch and other event detection variables
float altGround = 0; // ground level
float maxHeight = 0;  // highest altitude measured
float totalHeightAboveGround = 0; // totalHeightAboveGround = maxHeight - altGround
bool altGroundLevelTaken = 0;
bool launch = 0;
int launchDetectCounter = 0;
bool apogee = 0;
int apogeeCounter = 0;
bool landed = 0;
int landedCounter = 0;

CircularBuffer<float, 10> buffer;  // setup a circular buffer that holds 10 values
// if you change the sampleHertz away from 10hz, your circular buffer will not hold 1 second worth of data 
float minVal = 32767; //max out minimum. This is the lowest altitude in the latest second in circular buffer readings
float maxVal = 0;     // this is the max altitude in the latest second of circular buffer readings
float avg;            // average altitude in last second of circular buffer readings

void setup() {
  //Serial.begin(9600); // Not using the usb serial ... 
  //Begin HW serial
  Serial.begin(230400); //set to highest baud rate for xbee pro module
  //XBees need to be programmed with the above baud setting. XBee Pro S1 max 115200
  //XBee pro SC2 max at 230400 baud. Need to make sure firmware in xbee is set to 802.15.4 TH PRO setting
  imu.begin();
  filter.begin(100); //this is for NXP Sensor Fusion filter
  
  myPressure.begin(); // Get mpl3115 sensor online
  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(4); // Set Oversample to the recommended 128 - 
  // sampling intervals of 1=6 ms , 2=10, 4=18, 8=34, 16=66, 32=130, 64=258, and 128=512 
  //oversampling set to 128 takes half a second to perform. Even when set to 8 there is some lag for the orientation. 
  //Seems to be ok when set to 2. There is pretty good filtering with the simplified Kalman filter
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
}

// simple Kalman filter for altitude sensor only
// borrowed from http://orlygoingthirty.blogspot.com/2013/06/1-dimensional-kalman-filter-arduino.html?m=1
// adapted from C code by Adrian Boeing, www.adrianboeing.com 
// simplistic Kalman filter for encoder readings
// KalmanFilter kf(0, 0.01, 1.0);
KalmanFilter::KalmanFilter(float estimate, float initQ, float initR)
{
  Q = initQ;
  R = initR;

  // initial values for the kalman filter
  x_est_last = 0;
  P_last = 0;

  // initialize with a measurement
  x_est_last = estimate;
}

// add a new measurement, return the Kalman-filtered value
float KalmanFilter::step(float z_measured)
{
  // do a prediction
  x_temp_est = x_est_last;
  P_temp = P_last + R*Q;

  // calculate the Kalman gain
  K = P_temp * (1.0/(P_temp + R));

  // correct
  x_est = x_temp_est + K * (z_measured - x_temp_est); 
  P = (1- K) * P_temp;

  // update our last's
  P_last = P;
  x_est_last = x_est;

  return (x_est);
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) { //The imu waits here for about 9.5ms here to populate new values
    
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    
    float alt = (float) myPressure.readAltitudeFt(); // gets float of altitude with two decimal places for kalman filtering
    float alt_kalman = kf.step(alt); //filter noise from altimeter sensor, and get readings to settle
    
    //Circular buffer part from https://github.com/rlogiacco/CircularBuffer/tree/master/examples/CircularBuffer 
    //I have it set for holding the last 10 readings (currently 1 second of readings) and then we calulate maxAlt minAlt and avg
    //in the buffer after each new reading is taken. Oldest readings are discarded each time a new one is pushed
    //(in their example they have an error in calculating avg, this is fixed here)    
    buffer.push(alt_kalman); // Place new kalman filtered reading in altitude buffer
    minVal = 32767;      // zero maxVal and avg, but max out minimum, so that it gives true minimum
    maxVal = 0;
		avg = 0;
	  using index_t = decltype(buffer)::index_t; // sets up circularbuffer
    for (index_t i = 0; i < buffer.size(); i++) {
			avg += buffer[i]; 
      if (buffer[i] > maxVal) maxVal = buffer[i];
      if (buffer[i] < minVal) minVal = buffer[i];
		}
    avg = avg / buffer.size();

    if (maxVal > maxHeight) maxHeight = maxVal; //if altitude has increased, update max height for highest reading of altMax 

    if (altGroundLevelTaken != true and buffer[9] != 0){
      altGround = avg;              //after power up, take average ground level alt reading
      altGroundLevelTaken = true;
    }    

    // print the heading, pitch and roll to the xbee serial port
    // kept decimals in for these variables because I liked the smoother movement in the processing display
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("O: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.println(avg);

    //***** Launch Detection *********
    if(launch == false and ax > 1.3 and avg - altGround > 0) { // 1.3 vertical g's - for testing only! For flight this should be >3g's  and 20m above ground at least
      launchDetectCounter = launchDetectCounter + 1; //increment lauchdetectcounter
    }else {
      launchDetectCounter = 0; //reset launch detect counter if a lower gravity reading or too low 
    }
    if(launch == false and launchDetectCounter > 1) { //(set to 1 for test, set to 4 for Launch) Launch detected! after 2 or 5 consecutive passing tests 
      launch = true;
      //voice.say(spLAUNCH); //The voice.say blocks other code. Not recommended exept prior to launch, or after landing - or just for testing
      Serial.println("Launch");     
    }

    //****  Apogee Detection  ****
    if(launch == true and apogee == false) {  //then perform check for apogee
      if (avg - minVal < 0.5 and pitch <= 0 and maxHeight > maxVal) {         //note that pitch will be =< 0 when apogee reached. 
        //above checks that within the last second the avg height is less than .5 meters higher than minVal and pitch has reached horizontal and maxVal is lower than maxheight 
        apogeeCounter = apogeeCounter +1; 
      }
      else {
        apogeeCounter = 0; //reset apogee detect counter if above checks not true
      }
      if(apogeeCounter > 2) { // Apogee detected! after 3 consecutive passing tests
        apogee = true;
        //voice.say(spROLL_OUT);
        Serial.println("Apogee");  
      }
    }

    //****  Landing Detection  ****
    if(apogee == true and landed != true and avg - altGround < 100) {  // if we have apogee and are less than 100m above ground then check for Landing event

      if(maxVal - minVal < 0.35) {   //alt reading within 2 feet (note that pitch measurement should also be resting once ground is reached. Could add this to check)
        landedCounter = landedCounter + 1;  //dif in min and max alt (in 1 sec buffer) has reached a minimum
      }
      else {
        landedCounter = 0; //reset landing detect counter if still descending at more than 2 feet per second
      }
      if(landedCounter > 5) { // Landing detected after 6 consecutive passing tests
        landed = true;
        //voice.say(spTOUCHDOWN); 
        Serial.println("Landed"); 
      }
    }    
  }
}
