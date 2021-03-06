/* 
 This is originally based on
 MPU9250 Basic Example Code by Kris Winer and Brett Wilkins 
 which had some boilerplate for 10DOF sensor communication and
 Madgwick/Mahony algorithms. Original description:

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"

#ifdef LCD
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);
#endif // LCD

#define AHRS true          // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging
#define TCPcomms false      // Set to false if you don't want to steer over WiFi
#define UDPcomms true

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int ledPin = D7;
// PS. I think pin 12 on photon is A0?

#define FRONT_LEFT_PIN RX
#define FRONT_RIGHT_PIN WKP
#define REAR_LEFT_PIN A5
#define REAR_RIGHT_PIN D3

Servo front_left;
Servo front_right;
Servo rear_left;
Servo rear_right;

bool attitude_control_enabled = false;

float pitch_reference = 0.0f;
float roll_reference = 0.0f;
float yaw_reference = 0.0f;
float throttle_reference = 0.0f; //not used anywhere yet

int max_pid_output = 400;

float P_pitch =  6.0f;
float P_roll =  P_pitch;

float D_pitch = 1.0f;
float D_roll = D_pitch;

float I_yaw = 0.000f;
float I_pitch = 0.010f;
float I_roll = I_pitch;

int P_pitch_int = (int) P_pitch; // just for manual montoring
int D_pitch_int = (int) D_pitch; // just for manual montoring
int I_pitch_int = (int) I_pitch; // just for manual montoring

int converge_cnt = 0;            // helper counter to let IMU converge after spin-up

float pitch_error = 0.0f;
float pitch_error_derivative = 0.0f;
float prev_pitch_error = 0.0f;
float pitch_i_mem = 0.0f;

float roll_error = 0.0f;
float roll_error_derivative = 0.0f;
float prev_roll_error = 0.0f;
float roll_i_mem = 0.0f;

float yaw_error = 0.0f;
float yaw_i_mem = 0.0f;

float PID_pitch = 0.0f;
float PID_roll = 0.0f;
float PID_yaw = 0.0f;

float pitch_trim = 0.f;
float roll_trim = 0.f;

int pitch_control = 0;
int roll_control = 0;
int u_0 = 1150;        // 15% idle

int pitch_in_tenths = 0;  // just for manual monitoring
int roll_in_tenths = 0;   // just for manual monitoring

unsigned long last_joystick_update = 0;
unsigned long mytime = 0;

MPU9250 myIMU;
TCPClient client;
TCPServer server = TCPServer(12345);
UDP Udp;


void setup()
{
  digitalWrite(ledPin, HIGH);   // signalize the beginning of setup

  // Motor servos attachments
  front_left.attach(FRONT_LEFT_PIN, 1000, 2000);
  front_right.attach(FRONT_RIGHT_PIN, 1000, 2000);
  rear_left.attach(REAR_LEFT_PIN, 1000, 2000);
  rear_right.attach(REAR_RIGHT_PIN, 1000, 2000);

  Particle.function("motor_commands", motor_commands);
  Particle.function("P gain", set_P_gain);
  Particle.function("D gain", set_D_gain);
  Particle.function("I gain", set_I_gain);
  Particle.function("pitch trim", set_pitch_trim);
  Particle.function("roll trim", set_roll_trim);
  Particle.function("joystick", joystick_input);
  Particle.variable("pitch_in_tenths", pitch_in_tenths);
  Particle.variable("roll_in_tenths", roll_in_tenths);
  Particle.variable("P_gain", P_pitch_int);
  Particle.variable("I_gain", I_pitch_int);
  Particle.variable("D_gain", D_pitch_int);
  
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  // not sure about this intPin -- does it do anything? Find out.
  pinMode(ledPin, OUTPUT);
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  digitalWrite(ledPin, LOW);


  if(TCPcomms){

    server.begin();   
    Particle.publish(WiFi.localIP().toString());
    delay(200);
  }

  if(UDPcomms){

    Udp.begin(8000);
    Particle.publish(WiFi.localIP().toString());
    delay(200);
  }

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);


  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    delay(3000);
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1){delay(1000);} ; // Loop forever if communication doesn't happen
  }

  digitalWrite(ledPin, LOW);  // signalize the end of setup
}

void loop()
{
  //delay(1); //can thottle refresh rate here

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();

    // THIS MAGBIAS CAN BE IMPROVED IN FUTURE, NOW IT'S KINDA DUMB AND RANDOM
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
               
    
    //Spark.publish("gpsloc", szInfo); //Publish Data

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(myIMU.ax, myIMU.az, myIMU.ay, 
                          -myIMU.gx*DEG_TO_RAD, -myIMU.gz*DEG_TO_RAD, -myIMU.gy*DEG_TO_RAD, 
                          -myIMU.my,  myIMU.mz, -myIMU.mx, myIMU.deltat); 

  // About Madgwick: swapping middle coordinates changes roll sign, but pitch keeps being 
  // reverse of what I want (negative nose up)

  // MahonyQuaternionUpdate(myIMU.ax, myIMU.az, myIMU.ay, myIMU.gx*DEG_TO_RAD,
  //                       myIMU.gz*DEG_TO_RAD, myIMU.gy*DEG_TO_RAD, myIMU.mz,
  //                       myIMU.mx, myIMU.my, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
      }

      myIMU.count = millis();
      
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    myIMU.delt_t = millis() - myIMU.count; // time since TCP/UDP packet

  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
    myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                  *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                  *(getQ()+2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                  *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));

    myIMU.pitch *= -RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    //myIMU.yaw   -= 6.1;
    myIMU.roll  *= RAD_TO_DEG;

    // I don't know why did I have to put this here but seems to be working fine now:
    // 0 roll is level, right wing down is negative (around 0.2deg dead zone should be fine)
    if (myIMU.roll > 0)
      myIMU.roll -= 180.0f;
    else
      myIMU.roll += 180.0f;

    myIMU.pitch += pitch_trim;
    myIMU.roll += roll_trim;

    pitch_in_tenths = (int) (myIMU.pitch * 10);
    roll_in_tenths = (int) (myIMU.roll * 10);

    P_pitch_int = (int) P_pitch;
    I_pitch_int = (int) I_pitch;
    D_pitch_int = (int) D_pitch;

    // While the engines spin up, IMU readings go crazy for a while
    // This counter lets it converge to actual predictions
    // without triggering safety shut-down, or crazy motor action
    if(converge_cnt > 0){
      converge_cnt--;
      if(converge_cnt == 0){
        digitalWrite(ledPin, LOW);
      }
    }

    // Safety precaution for now -- cut off the engines 
    // at an attitude outside some bracket
    if(converge_cnt == 0 && (abs(myIMU.roll) > 55.0f || abs(myIMU.pitch) > 55.0f)){ 
      attitude_control_enabled = false;

      front_left.writeMicroseconds(1000);
      front_right.writeMicroseconds(1000);
      rear_left.writeMicroseconds(1000);
      rear_right.writeMicroseconds(1000);
      
      Serial.println("got into safety thing");
      Particle.publish("got into safety thingy");
      delay(1000);
    }

    if(attitude_control_enabled && converge_cnt == 0){
     
      // MOTOR CONTROL

      pitch_error = pitch_reference - myIMU.pitch; 
      roll_error = roll_reference - myIMU.roll;
      yaw_error = yaw_reference - myIMU.yaw;

      // ! WARNING !
      // If you ever move the following two lines outside of 
      // current 'if' clause (attitude_control_enabled),
      // integral errors will quickly accumulate without being corrected
      // ------------
      pitch_i_mem += I_pitch * pitch_error;
      roll_i_mem += I_roll * roll_error;

      pitch_error_derivative = (pitch_error - prev_pitch_error) / myIMU.deltat; 
      roll_error_derivative = (roll_error - prev_roll_error) / myIMU.deltat;

      prev_pitch_error = pitch_error;
      prev_roll_error = roll_error;
    
      // Calculate PID outputs
      PID_pitch = P_pitch * pitch_error + D_pitch * pitch_error_derivative + pitch_i_mem;
      pitch_control = (int) PID_pitch;
      if (pitch_control > max_pid_output) pitch_control = max_pid_output;
      else if (pitch_control < -max_pid_output) pitch_control = -max_pid_output;

      PID_roll = P_roll * roll_error + D_roll * roll_error_derivative + roll_i_mem;
      roll_control = (int) PID_roll;
      if (roll_control > max_pid_output) roll_control = max_pid_output;
      else if (roll_control < -max_pid_output) roll_control = -max_pid_output;

      yaw_i_mem += I_yaw * yaw_error;
      // I honestly don't know if there are any good practices as to where should I add saturation.
      // I add it at the very end obviously but should every PID branch have one?
      // Pitch and roll have 400 for now, yaw has 50
      yaw_i_mem = max(-50, min(50, yaw_i_mem)); 

      PID_yaw = yaw_i_mem;

      // Add everything together and clip the final signal
      front_left.writeMicroseconds(max(1100, min(u_0 + pitch_control - roll_control - PID_yaw, 1900)));
      front_right.writeMicroseconds(max(1100, min(u_0 + pitch_control + roll_control + PID_yaw, 1900)));
      rear_left.writeMicroseconds(max(1100, min(u_0 - pitch_control - roll_control + PID_yaw, 1900)));
      rear_right.writeMicroseconds(max(1100, min(u_0 - pitch_control + roll_control - PID_yaw, 1900)));      

    }

    // Custom frequency to debug something etc.
    // if (millis() - mytime >= 500){
    //   Serial.println("Something got sent by UDP.");
    //   Serial.print("Available bytes: "); Serial.println(Udp.available());
    //   Serial.print("roll_reference: "); Serial.println(roll_reference, 3);
    //   Serial.print("pitch_reference: "); Serial.println(pitch_reference, 3);
    //   Serial.print("throttle_reference: "); Serial.println(throttle_reference, 3);
    //   Serial.println("\n");
    //   Serial.print("pitch_error: "); Serial.println(pitch_error, 3);
    //   Serial.print(" roll_error: "); Serial.println(roll_error, 3);
    //   mytime = millis();
    // }

    // Read network packets independently of sensor data rates
    if (myIMU.delt_t >= 10)
    {          

      if (TCPcomms && client.connected()){

        // Check for 12 bytes (3 floats) from joystick input --
        // if we have them, update roll, pitch and throttle references        
        if(client.available() >= 12){
          
          byte tempBuff[4];
          float newInput[3];

          for(int j = 0; j < 3; j++){
            for(int i = 0; i < 4; i++)
            {
              tempBuff[i] = client.read();                     
            }
            newInput[j] = *((float*)(tempBuff));            
          }
          roll_reference = newInput[0];
          pitch_reference = newInput[1];
          throttle_reference = newInput[2];

          last_joystick_update = millis();      

        }
        else{
          
          /* 
          This is a weak spot of the entire loop.
          TCP delays can spike to over 3s when a packet gets retransmitted
          (especially when checking something via particle funtion call or using the internet)
          but in those 3 seconds the drone can easily crash...
          I don't know what would be a good safety mechanism that won't disconnect often.
          Is bluetooth more reliable in this regard? Radio?
          Using UDP for now
          */

          // 3 seconds seemed like a good middle ground
          // if(millis() - last_joystick_update > 5000){
          //   // Serial.print("Last joystick update: "); Serial.println(millis() - last_joystick_update);
          //   // Serial.println("Suspected dead client. Closing connnection.");
          //   pitch_reference = 0.0f;
          //   roll_reference = 0.0f;
          //   throttle_reference = 0.0f;
          //   client.stop();
          //   Particle.publish("Lost client");
          //   delay(1000);          
          // }
          
        }
      }

      if(UDPcomms && Udp.parsePacket() > 0){
        
        if(Udp.available() >= 12){
          byte tempBuff[4];
          float newInput[3];

          for(int j = 0; j < 3; j++){
            for(int i = 0; i < 4; i++)
            {
              tempBuff[i] = Udp.read();                     
            }
            newInput[j] = *((float*)(tempBuff));            
          }

          // Because UDP is not reliable, I think there is a need for some sanity check
          if(newInput[0] > -10.f && newInput[0] < 10.f)
            roll_reference = newInput[0];
          if(newInput[1] > -10.f && newInput[1] < 10.f)
            pitch_reference = newInput[1];
          if(newInput[2] > -10.f && newInput[2] < 10.f)
            throttle_reference = newInput[2];
        }        
      }

      if(SerialDebug)
      {
        Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = "); Serial.print( myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print( myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print( myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = "); Serial.print( (int)myIMU.mx );
        Serial.print(" my = "); Serial.print( (int)myIMU.my );
        Serial.print(" mz = "); Serial.print( (int)myIMU.mz );
        Serial.println(" mG");

        Serial.print("q0 = "); Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));

        Serial.print("deltat = "); Serial.println(myIMU.deltat);
        Serial.print("delt_t = "); Serial.println(myIMU.delt_t);
      
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
        Serial.println(" Hz");
        
      }
      
     
      // Check if TCP clients are available if already not connected
      if (TCPcomms && !client.connected())
      {
        //Serial.println("No TCP connection.");

        client = server.available();        

        if(client.connected()){

          Serial.print("Connected with client with address: ");
          Serial.println(client.remoteIP().toString());         
          last_joystick_update = millis();
        }
      }
      


    // With these settings the filter is updating at a ~145 Hz rate using the
    // Madgwick scheme and >200 Hz using the Mahony scheme. 
    // The filter update rate is determined mostly
    // by the mathematical steps in the respective algorithms, the
    // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
    // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
    // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
    // presumably because the magnetometer read takes longer than the gyro or
    // accelerometer reads. This filter update rate should be fast enough to
    // maintain accurate platform orientation for stabilization control of a
    // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050
    // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
    // well!

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;      
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}


// =================
// Below are some Particle specific functions. Most are called from a web console, 
// the last one (joystick_input) is called from a python joystick script when buttons are pressed.
// Particle functions always take a string as an argument and return an integer.
// =================


int set_P_gain(String command) {

    P_pitch = command.toFloat();
    P_roll = P_pitch;
    return 1;
}

int set_D_gain(String command) {

    D_pitch = command.toFloat();
    D_roll = D_pitch;
    return 1;
}

int set_I_gain(String command) {

    I_pitch = command.toFloat();
    I_roll = I_pitch;
    return 1;
}

int set_pitch_trim(String command) {

    pitch_trim = command.toFloat();
    return 1;
}

int set_roll_trim(String command) {

    roll_trim = command.toFloat();
    return 1;
}

int motor_commands(String command) {   

    if (command=="off") {
        
        attitude_control_enabled = false;

        front_left.writeMicroseconds(1000);
        front_right.writeMicroseconds(1000);
        rear_left.writeMicroseconds(1000);
        rear_right.writeMicroseconds(1000);
        
        return 1;
    }
    else if (command=="slow"){

      front_left.writeMicroseconds(1080);
      front_right.writeMicroseconds(1080);
      rear_left.writeMicroseconds(1080);
      rear_right.writeMicroseconds(1080);

      return 1;
    }
    else if (command=="spin up"){
      
      for (int i = 1000; i <= u_0; i += 5) {

        front_left.writeMicroseconds(i);
        front_right.writeMicroseconds(i);
        rear_left.writeMicroseconds(i);
        rear_right.writeMicroseconds(i);
        
        delay(250);
      }
      
      return 1;
    }
    else if (command=="enable"){
        attitude_control_enabled = true;
        return 1;
    }
    else if (command=="disable"){
        attitude_control_enabled = false;
        return 1;
    }
    else
      return -1;   
}


// This is for joystick buttons 
int joystick_input(String command) {

  if (command=="Trim down") {
    pitch_trim += 2.f;
    // D_pitch += 0.1f;
    // D_roll = D_pitch;
  }
  else if (command=="Trim up"){
    pitch_trim -= 2.f;
    // D_pitch -= 0.1f;
    // D_roll = D_pitch;
  }
  else if (command=="Trim left"){
    roll_trim += 2.f;
    //P_pitch -= 1.0f;  
  }
  else if (command=="Trim right"){
    roll_trim -= 2.f;
    //P_pitch += 1.0f;
  }
  else if (command=="Power down"){
    attitude_control_enabled = false;

    front_left.writeMicroseconds(1000);
    front_right.writeMicroseconds(1000);
    rear_left.writeMicroseconds(1000);
    rear_right.writeMicroseconds(1000);
    
  }
  else if (command=="Spin up"){
    digitalWrite(ledPin, HIGH);

    for (int i = 1000; i <= u_0; i += 10) {
              
      front_left.writeMicroseconds(i);
      front_right.writeMicroseconds(i);
      rear_left.writeMicroseconds(i);
      rear_right.writeMicroseconds(i);
      
      delay(200);
      // This is so that we won't disconnect on spin up
      last_joystick_update = millis();  
    }
    converge_cnt = 1000;
  }
  else if (command=="Enable control"){
    yaw_reference = myIMU.yaw;
    attitude_control_enabled = true;
  }
  return 1;
}